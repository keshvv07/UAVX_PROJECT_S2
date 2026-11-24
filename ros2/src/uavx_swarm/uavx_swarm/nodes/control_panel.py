"""Centralised control panel - the evaluator's seat at the simulation (organisers' Q32-Q35).

The organisers asked for exactly this: a single place from which an individual UAV can be
switched off during the mission, and a slider that varies the communication bit error rate
from 0 to 1, with the communication model responding live rather than the run being
restarted for every parameter change (Q35). This node serves that panel over HTTP and
turns every control into a `/scenario/inject_request`. This panel is the ONLY way into a
run: the scenario files script no disturbance, the swarm nodes generate none, and the
scenario manager refuses any request that does not come from here. Each change is logged
as a DISTURBANCE event and scored by the referee.

    ros2 launch uavx_swarm uavx.launch.py scenario:=gc1        # panel comes up with the run
    http://localhost:8089

What it shows: the mission clock, every UAV (state, role, battery, route cost, next hop,
hops to the GCS, buffered reports, sector progress), the live link graph with the relay
chain, the PoIs with their report latency against the organisers' 10 s, and a top view of
the search area. What it changes: UAV kill switch, bit error rate, packet loss, radio
outage, jamming zone, battery level, endurance, and dropping a new emergency PoI anywhere
on the map.

It is a read-and-inject panel, not a second autopilot: it cannot command a UAV anywhere.
Every control is a disturbance, which is the only thing an evaluator should be able to do
to a run they are grading.

Binding. It listens on 0.0.0.0 by default so the page is reachable when the simulation runs
in the evaluators' container (`docker run -p 8089:8089`). On a shared machine, set
`panel_host: 127.0.0.1` in the scenario to keep it local.
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..common import EVENT_QOS, LATCHED_QOS, EventBus, dumps, load_scenario, now_s, spin
from ..world_map import WorldMap

# Only these may be injected from the panel: every one is a disturbance the scenario format
# already defines. Anything else is refused, so the panel can never invent a command.
ALLOWED = {
    "uav_failure": {"target"},
    "comm_outage": {"target", "duration_s"},
    "packet_loss": {"target", "loss", "duration_s"},
    "ber": {"target", "ber", "duration_s"},
    "jamming": {"center_m", "radius_m", "attenuation_db", "duration_s"},
    "battery": {"target", "pct"},
    "endurance": {"target", "minutes"},
    "new_poi": {"id", "pos_m", "reported_m", "priority", "survey_s"},
    "clear_impairments": set(),
}
MAX_EVENTS = 600


class ControlPanel(Node):
    def __init__(self):
        super().__init__("control_panel")
        self.declare_parameter("scenario", "gc1")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("port", 0)
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.run_dir = self.get_parameter("run_dir").value
        port = int(self.get_parameter("port").value) or int(self.sc.get("panel_port", 8089))
        host = str(self.sc.get("panel_host", "0.0.0.0"))
        self.events = EventBus(self, "panel")
        self.map = WorldMap.for_scenario(self.sc)
        self.gcs_pos = tuple(self.sc["gcs"]["position_m"])
        self.uav_ids = [u["id"] for u in self.sc["uavs"]]
        self.colors = {u["id"]: [round(c, 3) for c in list(u["color"])[:3]] for u in self.sc["uavs"]}

        self.status: dict[str, dict] = {}
        self.links: list[dict] = []
        self.radio_stats: dict = {}
        self.pois: dict[str, dict] = {}
        self.log: list[dict] = []
        self.t0 = None
        self.injections = 0
        self._lock = threading.Lock()

        self.inject_pub = self.create_publisher(String, "/scenario/inject_request", EVENT_QOS)
        for uid in self.uav_ids:
            self.create_subscription(String, f"/{uid}/companion/status",
                                     lambda m, u=uid: self._on_status(u, m), EVENT_QOS)
        self.create_subscription(String, "/uavx/events", self._on_event, EVENT_QOS)
        self.create_subscription(String, "/scenario/poi", self._on_poi, EVENT_QOS)
        self.create_subscription(String, "/scenario/clock", self._on_clock, LATCHED_QOS)
        self.create_subscription(String, "/radio/stats", self._on_radio_stats, EVENT_QOS)
        try:
            from uavx_interfaces.msg import NetworkState
            self.create_subscription(NetworkState, "/radio/network_state", self._on_network, EVENT_QOS)
        except ImportError:                                            # pragma: no cover
            pass

        self.httpd = ThreadingHTTPServer((host, port), _make_handler(self))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        shown = "localhost" if host in ("0.0.0.0", "") else host
        self.get_logger().info(f"control panel on http://{shown}:{port}  "
                               f"(UAV kill switch, bit error rate, loss, jamming, battery, new PoI)")

    # ---------------------------------------------------------------- inputs
    def _on_status(self, uid: str, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.status[uid] = body

    def _on_network(self, msg) -> None:
        with self._lock:
            self.links = [{"a": l.a, "b": l.b, "d": round(l.distance_m, 1), "pdr": round(l.pdr, 3),
                           "channel_pdr": round(l.channel_pdr, 3), "up": bool(l.up),
                           "ber": l.ber, "loss_db": round(l.extra_loss_db, 1),
                           "impairment": l.impairment, "latency_ms": round(l.latency_ms, 1)}
                          for l in msg.links]

    def _on_radio_stats(self, msg: String) -> None:
        try:
            with self._lock:
                self.radio_stats = json.loads(msg.data)
        except ValueError:
            pass

    def _on_poi(self, msg: String) -> None:
        try:
            p = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.pois.setdefault(p["id"], {"id": p["id"], "pos": p["reported_m"][:2],
                                           "priority": p.get("priority", 1), "status": "PENDING",
                                           "latency_s": None, "hops": None})

    def _on_clock(self, msg: String) -> None:
        self.t0 = json.loads(msg.data).get("t0")

    def _on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.log.append(ev)
            del self.log[:-MAX_EVENTS]
            poi = self.pois.get(ev.get("poi_id") or "")
            if poi is not None:
                if ev["type"] == "POI_RECEIVED":
                    poi["latency_s"] = ev.get("latency_s")
                    poi["hops"] = ev.get("hops")
                    poi["status"] = "REPORTED"
                elif ev["type"] == "POI_COMPLETED":
                    poi["status"] = "DONE"
                elif ev["type"] == "ASSIGNED":
                    poi["status"] = "ASSIGNED"

    # ---------------------------------------------------------------- outputs
    def snapshot(self) -> dict:
        with self._lock:
            uavs = []
            for uid in self.uav_ids:
                st = dict(self.status.get(uid) or {})
                st["id"] = uid
                st["color"] = self.colors.get(uid)
                uavs.append(st)
            area = list(self.map.mission_area) if self.map and self.map.mission_area else None
            chain = next((e for e in reversed(self.log) if e.get("type") == "CHAIN_PLANNED"), None)
            return {
                "mission_t": None if self.t0 is None else round(now_s(self) - self.t0, 1),
                "sim_t": round(now_s(self), 1),
                "uavs": uavs,
                "gcs": {"id": self.sc["gcs"].get("id", "gcs"), "pos": list(self.gcs_pos)},
                "links": list(self.links),
                "pois": list(self.pois.values()),
                "radio": self.radio_stats,
                "chain": chain,
                "area": area,
                "geofence": list(self.map.geofence) if self.map else None,
                "trees": self.map.canopy.count if self.map else 0,
                "scenario": {"name": self.sc.get("name"), "level": self.sc.get("level"),
                             "duration_s": self.sc.get("duration_s"), "map": self.sc.get("map")},
                "injections": self.injections,
                "events": self.log[-80:],
            }

    def inject(self, body: dict) -> tuple[int, dict]:
        kind = body.get("type")
        if kind not in ALLOWED:
            return 400, {"error": f"not an injectable disturbance: {kind!r}",
                         "allowed": sorted(ALLOWED)}
        extra = set(body) - {"type"} - ALLOWED[kind]
        if extra:
            return 400, {"error": f"unexpected fields for {kind}: {sorted(extra)}"}
        target = body.get("target")
        if "target" in ALLOWED[kind] and target not in (None, "*") and target not in self.uav_ids:
            return 400, {"error": f"unknown UAV {target!r}", "uavs": self.uav_ids}
        if kind == "ber" and not 0.0 <= float(body.get("ber", 0.0)) <= 1.0:
            return 400, {"error": "ber must be between 0 and 1"}
        if kind == "packet_loss" and not 0.0 <= float(body.get("loss", 0.0)) <= 1.0:
            return 400, {"error": "loss must be between 0 and 1"}
        msg = String()
        msg.data = dumps(dict(body, source="control_panel"))
        self.inject_pub.publish(msg)
        with self._lock:
            self.injections += 1
        self.events.emit("PANEL_INJECT", **{k: v for k, v in body.items() if k != "type"}, kind=kind)
        self.get_logger().warn(f"panel injected {kind} {json.dumps({k: v for k, v in body.items() if k != 'type'})}")
        return 200, {"ok": True, "injected": kind}

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:                                              # pragma: no cover
            pass
        return super().destroy_node()


# ------------------------------------------------------------------------- HTTP
def _make_handler(panel: ControlPanel):
    class Handler(BaseHTTPRequestHandler):
        server_version = "uavx-control-panel"

        def log_message(self, *args):          # keep the ROS log readable
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, obj) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):                      # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(200, panel.snapshot())
            elif path == "/api/events":
                since = int((parse_qs(urlparse(self.path).query).get("since") or ["0"])[0])
                with panel._lock:
                    self._json(200, {"events": panel.log[since:], "next": len(panel.log)})
            elif path == "/api/health":
                self._json(200, {"ok": True, "sim_t": round(now_s(panel), 1)})
            else:
                self._json(404, {"error": "no such endpoint"})

        def do_POST(self):                     # noqa: N802
            if urlparse(self.path).path != "/api/inject":
                self._json(404, {"error": "no such endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(length) or b"{}")
            except (ValueError, TypeError):
                self._json(400, {"error": "body must be JSON"})
                return
            if not isinstance(body, dict):
                self._json(400, {"error": "body must be a JSON object"})
                return
            code, out = panel.inject(body)
            self._json(code, out)

    return Handler


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>UAV-X mission control</title>
<style>
  :root{--bg:#0e1116;--panel:#161b22;--line:#273039;--text:#e6edf3;--dim:#8b949e;
        --ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--text);
       font:14px/1.45 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
  header{display:flex;gap:16px;align-items:baseline;flex-wrap:wrap;
         padding:10px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
  h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.2px}
  .clock{font-variant-numeric:tabular-nums;color:var(--acc);font-weight:600}
  .muted{color:var(--dim)}
  main{display:grid;grid-template-columns:minmax(320px,1fr) 360px;gap:12px;padding:12px}
  @media (max-width:900px){main{grid-template-columns:1fr}}
  section{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px}
  h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:var(--dim);
     margin:0 0 10px;font-weight:600}
  canvas{width:100%;height:auto;background:#0b0e13;border-radius:6px;display:block}
  table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
  th,td{text-align:left;padding:4px 6px;border-bottom:1px solid var(--line);font-size:13px}
  th{color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase}
  .kill{background:#2d1518;color:var(--bad);border:1px solid #5c1f23;border-radius:5px;
        padding:3px 9px;cursor:pointer;font:inherit;font-size:12px}
  .kill:hover{background:#4a1d22}.kill:disabled{opacity:.35;cursor:default}
  .row{display:flex;gap:8px;align-items:center;margin:8px 0;flex-wrap:wrap}
  .row label{min-width:104px;color:var(--dim);font-size:12px}
  input[type=range]{flex:1;min-width:120px;accent-color:var(--acc)}
  input[type=number],select{background:#0b0e13;color:var(--text);border:1px solid var(--line);
       border-radius:5px;padding:3px 6px;font:inherit;width:84px}
  button.go{background:#13281c;color:var(--ok);border:1px solid #1f4428;border-radius:5px;
            padding:4px 10px;cursor:pointer;font:inherit;font-size:12px}
  button.go:hover{background:#1b3a26}
  .val{min-width:76px;text-align:right;color:var(--acc);font-size:12px}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px}
  #log{height:148px;overflow:auto;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
       font-size:11px;line-height:1.5}
  #log div{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
  .t{color:var(--dim)}.ev{color:var(--acc)}
  .warnrow{color:var(--warn)}.badrow{color:var(--bad)}
  .pill{border:1px solid var(--line);border-radius:999px;padding:1px 7px;font-size:11px;color:var(--dim)}
</style></head><body>
<header>
  <h1>UAV-X mission control</h1>
  <span class="muted" id="scn">-</span>
  <span class="clock" id="clk">t = -</span>
  <span class="pill" id="net">-</span>
  <span class="pill" id="chain">-</span>
  <span class="muted" style="margin-left:auto" id="conn">connecting...</span>
</header>
<main>
  <div style="display:flex;flex-direction:column;gap:12px">
    <section><h2>Search area</h2><canvas id="map" width="900" height="640"></canvas></section>
    <section><h2>Fleet</h2>
      <table><thead><tr><th>UAV</th><th>state</th><th>role</th><th>batt</th><th>route</th>
      <th>buf</th><th>sweep</th><th></th></tr></thead><tbody id="fleet"></tbody></table>
    </section>
    <section><h2>Points of interest</h2>
      <table><thead><tr><th>PoI</th><th>pri</th><th>status</th><th>report</th><th>hops</th></tr></thead>
      <tbody id="pois"></tbody></table>
    </section>
  </div>
  <div style="display:flex;flex-direction:column;gap:12px">
    <section><h2>Communication</h2>
      <div class="row"><label for="ber">Bit error rate</label>
        <input type="range" id="ber" min="0" max="1" step="0.000001" value="0">
        <span class="val" id="berv">0</span></div>
      <div class="row"><label></label><span class="muted" style="font-size:11px" id="berhint">
        a clean channel</span></div>
      <div class="row"><label for="bertgt">applies to</label>
        <select id="bertgt"></select><button class="go" id="bergo">Set BER</button></div>
      <div class="row"><label for="loss">Packet loss</label>
        <input type="range" id="loss" min="0" max="1" step="0.01" value="0">
        <span class="val" id="lossv">0 %</span></div>
      <div class="row"><label for="losstgt">applies to</label>
        <select id="losstgt"></select><button class="go" id="lossgo">Set loss</button></div>
      <div class="row"><label for="outtgt">Radio outage</label>
        <select id="outtgt"></select>
        <input type="number" id="outdur" value="10" min="1" max="600" title="seconds">
        <button class="go" id="outgo">Cut radio</button></div>
      <div class="row"><label>Jamming</label>
        <input type="number" id="jx" value="0" title="centre east (m)">
        <input type="number" id="jy" value="0" title="centre north (m)">
        <input type="number" id="jr" value="60" title="radius (m)">
        <input type="number" id="jdb" value="25" title="dB">
        <button class="go" id="jamgo">Jam</button></div>
      <div class="row"><label></label><button class="go" id="clear">Clear all impairments</button></div>
    </section>
    <section><h2>Fleet and mission</h2>
      <div class="row"><label for="batttgt">Set battery</label>
        <select id="batttgt"></select>
        <input type="number" id="battpct" value="20" min="0" max="100" title="%">
        <button class="go" id="battgo">Apply</button></div>
      <div class="row"><label for="endtgt">Endurance</label>
        <select id="endtgt"></select>
        <input type="number" id="endmin" value="15" min="1" max="60" title="minutes">
        <button class="go" id="endgo">Apply</button></div>
      <div class="row"><label>New PoI</label>
        <input type="number" id="px" value="120" title="east (m)">
        <input type="number" id="py" value="120" title="north (m)">
        <input type="number" id="ppri" value="5" min="1" max="5" title="priority">
        <button class="go" id="poigo">Drop</button></div>
      <div class="row"><span class="muted" style="font-size:11px">
        Click the map to place a jamming zone (shift-click: a new PoI).</span></div>
    </section>
    <section><h2>Event log</h2><div id="log"></div></section>
  </div>
</main>
<script>
const $ = (id) => document.getElementById(id);
let S = null, seen = 0;

function fmtPdr(v){ return v == null ? "-" : (100*v).toFixed(0) + "%"; }
function post(body){
  return fetch("/api/inject", {method:"POST", headers:{"Content-Type":"application/json"},
    body: JSON.stringify(body)}).then(r => r.json()).then(j => {
      if (j.error) alert(j.error); return j; });
}
function fillTargets(){
  const ids = (S.uavs || []).map(u => u.id);
  for (const sel of ["bertgt","losstgt","outtgt","batttgt","endtgt"]) {
    const el = $(sel), keep = el.value;
    const opts = (sel === "outtgt") ? ids : ["*"].concat(ids);
    if (el.options.length !== opts.length) {
      el.innerHTML = opts.map(v => `<option value="${v}">${v === "*" ? "every link" : v}</option>`).join("");
      if (keep) el.value = keep;
    }
  }
}
function berHint(b){
  if (b <= 0) return "a clean channel";
  const bits = (S && S.radio && S.radio.backend && S.radio.backend.frame_bits) || 2048;
  const keep = Math.pow(1 - b, bits);
  return `${(100*keep).toFixed(keep < 0.1 ? 2 : 0)} % of frames survive (${bits}-bit frames)`;
}
$("ber").oninput = e => { const v = +e.target.value;
  $("berv").textContent = v.toExponential(2); $("berhint").textContent = berHint(v); };
$("loss").oninput = e => $("lossv").textContent = (100*e.target.value).toFixed(0) + " %";
$("bergo").onclick = () => post({type:"ber", target:$("bertgt").value, ber:+$("ber").value});
$("lossgo").onclick = () => post({type:"packet_loss", target:$("losstgt").value, loss:+$("loss").value});
$("outgo").onclick = () => post({type:"comm_outage", target:$("outtgt").value, duration_s:+$("outdur").value});
$("jamgo").onclick = () => post({type:"jamming", center_m:[+$("jx").value, +$("jy").value],
  radius_m:+$("jr").value, attenuation_db:+$("jdb").value, duration_s:60});
$("clear").onclick = () => post({type:"clear_impairments"});
$("battgo").onclick = () => post({type:"battery", target:$("batttgt").value, pct:+$("battpct").value});
$("endgo").onclick = () => post({type:"endurance", target:$("endtgt").value, minutes:+$("endmin").value});
$("poigo").onclick = () => post({type:"new_poi", pos_m:[+$("px").value, +$("py").value],
  priority:+$("ppri").value});
function kill(id){
  if (confirm(`Switch off ${id} in flight? Its motors stop and it falls.`))
    post({type:"uav_failure", target:id});
}

// ------------------------------------------------------------------ map
const cv = $("map"), cx = cv.getContext("2d");
let VIEW = null;
function draw(){
  if (!S) return;
  const fence = S.geofence || [-420,300,-300,300];
  const pad = 20, W = cv.width, H = cv.height;
  const sx = (W - 2*pad) / (fence[1]-fence[0]), sy = (H - 2*pad) / (fence[3]-fence[2]);
  const s = Math.min(sx, sy);
  const X = e => pad + (e - fence[0]) * s, Y = n => H - pad - (n - fence[2]) * s;
  VIEW = {X, Y, inv: (px,py) => [fence[0] + (px-pad)/s, fence[2] + (H-pad-py)/s]};
  cx.clearRect(0,0,W,H);
  if (S.area){ const a = S.area;
    cx.fillStyle = "#101c13"; cx.fillRect(X(a[0]), Y(a[3]), (a[1]-a[0])*s, (a[3]-a[2])*s);
    cx.strokeStyle = "#f2cc3c"; cx.lineWidth = 1.5;
    cx.strokeRect(X(a[0]), Y(a[3]), (a[1]-a[0])*s, (a[3]-a[2])*s);
    cx.fillStyle = "#6e7681"; cx.font = "11px sans-serif";
    cx.fillText(`${(a[1]-a[0]).toFixed(0)} x ${(a[3]-a[2]).toFixed(0)} m search area`, X(a[0])+6, Y(a[3])+14);
  }
  const pos = {}; (S.uavs||[]).forEach(u => { if (u.pos) pos[u.id] = u.pos; });
  if (S.gcs) pos[S.gcs.id] = S.gcs.pos;
  (S.links||[]).forEach(l => {
    if (!pos[l.a] || !pos[l.b] || !l.up) return;
    cx.strokeStyle = l.pdr > 0.8 ? "rgba(63,185,80,.75)" : l.pdr > 0.45
      ? "rgba(210,153,34,.75)" : "rgba(248,81,73,.6)";
    cx.lineWidth = l.pdr > 0.8 ? 2 : 1;
    cx.beginPath(); cx.moveTo(X(pos[l.a][0]), Y(pos[l.a][1]));
    cx.lineTo(X(pos[l.b][0]), Y(pos[l.b][1])); cx.stroke();
  });
  (S.radio && S.radio.impairments && S.radio.impairments.jamming || []).forEach(j => {
    cx.strokeStyle = "#bc8cff"; cx.setLineDash([5,4]); cx.lineWidth = 1.2;
    cx.beginPath(); cx.arc(X(j.center_m[0]), Y(j.center_m[1]), j.radius_m*s, 0, 7); cx.stroke();
    cx.setLineDash([]);
    cx.fillStyle = "#bc8cff"; cx.font = "10px sans-serif";
    cx.fillText("jamming", X(j.center_m[0])+4, Y(j.center_m[1])-4);
  });
  const ch = S.chain;
  if (ch && ch.points) ch.points.forEach(p => { if (!p) return;
    cx.strokeStyle = "rgba(88,166,255,.6)"; cx.setLineDash([3,3]);
    cx.beginPath(); cx.arc(X(p[0]), Y(p[1]), 5, 0, 7); cx.stroke(); cx.setLineDash([]); });
  (S.pois||[]).forEach(p => {
    const done = p.status === "DONE", rep = p.status === "REPORTED";
    cx.fillStyle = done ? "#3fb950" : rep ? "#d29922" : "#f85149";
    cx.beginPath(); cx.arc(X(p.pos[0]), Y(p.pos[1]), 4.5, 0, 7); cx.fill();
    cx.fillStyle = "#8b949e"; cx.font = "10px sans-serif";
    cx.fillText(p.id, X(p.pos[0])+7, Y(p.pos[1])+3);
  });
  if (S.gcs){ const g = S.gcs.pos;
    cx.fillStyle = "#e6edf3";
    cx.beginPath(); cx.moveTo(X(g[0]), Y(g[1])-7); cx.lineTo(X(g[0])-6, Y(g[1])+4);
    cx.lineTo(X(g[0])+6, Y(g[1])+4); cx.closePath(); cx.fill();
    cx.strokeStyle = "rgba(139,148,158,.45)"; cx.setLineDash([4,4]);
    cx.beginPath(); cx.arc(X(g[0]), Y(g[1]), 100*s, 0, 7); cx.stroke(); cx.setLineDash([]);
    cx.fillStyle = "#8b949e"; cx.font = "10px sans-serif"; cx.fillText("GCS", X(g[0])+9, Y(g[1])+10);
  }
  (S.uavs||[]).forEach(u => {
    if (!u.pos) return;
    const c = u.color ? `rgb(${u.color.map(v=>Math.round(255*v)).join(",")})` : "#58a6ff";
    const dead = u.state === "FAILED";
    cx.fillStyle = dead ? "#6e7681" : c;
    cx.beginPath(); cx.arc(X(u.pos[0]), Y(u.pos[1]), 5, 0, 7); cx.fill();
    if (u.relay_at){ cx.strokeStyle = c; cx.lineWidth = 1;
      cx.beginPath(); cx.arc(X(u.pos[0]), Y(u.pos[1]), 9, 0, 7); cx.stroke(); }
    cx.fillStyle = dead ? "#6e7681" : "#e6edf3"; cx.font = "10px sans-serif";
    cx.fillText(`${u.id}${dead ? " X" : ""} ${u.state||""}`, X(u.pos[0])+8, Y(u.pos[1])-6);
  });
}
cv.onclick = e => {
  if (!VIEW) return;
  const r = cv.getBoundingClientRect();
  const [ex, nn] = VIEW.inv((e.clientX-r.left) * cv.width/r.width,
                            (e.clientY-r.top) * cv.height/r.height);
  if (e.shiftKey){ $("px").value = ex.toFixed(0); $("py").value = nn.toFixed(0); }
  else { $("jx").value = ex.toFixed(0); $("jy").value = nn.toFixed(0); }
};

// ------------------------------------------------------------------ tables
function fleet(){
  $("fleet").innerHTML = (S.uavs||[]).map(u => {
    const c = u.color ? `rgb(${u.color.map(v=>Math.round(255*v)).join(",")})` : "#58a6ff";
    const r = u.route || {};
    const route = r.cost == null ? '<span class="muted">no route</span>'
      : `${r.next_hop||"-"} &middot; ${r.hops||0}h`;
    const batt = u.batt == null ? "-" : u.batt.toFixed(0) + "%";
    const bc = u.batt == null ? "" : u.batt < 25 ? ' class="badrow"' : u.batt < 40 ? ' class="warnrow"' : "";
    const sw = u.sweep ? `${(100*u.sweep.progress).toFixed(0)}% ${u.sweep.sector}` : "-";
    const final = ["FAILED","LANDED","DONE"].includes(u.state);
    return `<tr><td><span class="dot" style="background:${c}"></span>${u.id}</td>
      <td>${u.state||"-"}</td><td>${u.role||"-"}</td><td${bc}>${batt}</td><td>${route}</td>
      <td>${u.pending||0}</td><td>${sw}</td>
      <td><button class="kill" ${final?"disabled":""} onclick="kill('${u.id}')">kill</button></td></tr>`;
  }).join("");
}
function pois(){
  $("pois").innerHTML = (S.pois||[]).map(p => {
    const lat = p.latency_s == null ? "-" : p.latency_s.toFixed(1) + " s";
    const cls = p.latency_s == null ? "" : p.latency_s <= 10 ? ' class="dot" ' : ' class="warnrow" ';
    const col = p.status === "DONE" ? "var(--ok)" : p.status === "REPORTED" ? "var(--warn)" : "var(--bad)";
    return `<tr><td><span class="dot" style="background:${col}"></span>${p.id}</td>
      <td>${p.priority}</td><td>${p.status}</td><td${p.latency_s!=null&&p.latency_s>10?' class="warnrow"':''}>${lat}</td>
      <td>${p.hops==null?"-":p.hops}</td></tr>`;
  }).join("") || '<tr><td colspan="5" class="muted">none announced yet</td></tr>';
}
function logs(){
  const el = $("log"), near = el.scrollTop + el.clientHeight > el.scrollHeight - 30;
  el.innerHTML = (S.events||[]).slice(-70).map(e => {
    const bad = ["UAV_FAILED","UAV_DECLARED_LOST","COLLISION","GEOFENCE_VIOLATION","ROUTE_DOWN"].includes(e.type);
    const warn = ["DISTURBANCE","PANEL_INJECT","LINK_LOST","BATTERY_RTH","FERRY_START","RADIO_IMPAIRMENT"].includes(e.type);
    return `<div class="${bad?"badrow":warn?"warnrow":""}"><span class="t">${(e.t||0).toFixed(1)}</span>
      <span class="ev">${e.type}</span> ${e.src||""}</div>`;
  }).join("");
  if (near) el.scrollTop = el.scrollHeight;
}
function head(){
  const sc = S.scenario || {};
  $("scn").textContent = `${sc.name||"-"} | map ${sc.map||"-"} | ${S.trees||0} trees`;
  $("clk").textContent = S.mission_t == null ? `sim t = ${S.sim_t}` : `mission t = ${S.mission_t} s`;
  const r = S.radio || {}, h = r.hops || {}, l = r.latency_ms || {};
  $("net").textContent = `hops max ${h.max??"-"} | fwd ${h.forwarded_frames??0} | lat ${l.mean??"-"} ms`;
  const ch = S.chain;
  $("chain").textContent = ch ? `chain ${ch.mode} ${ch.points?ch.points.length:0}/${ch.needed} relays` : "chain -";
  $("conn").textContent = `${S.injections||0} injections`;
}
async function poll(){
  try {
    const r = await fetch("/api/state"); S = await r.json();
    head(); fleet(); pois(); logs(); fillTargets(); draw();
  } catch (e) { $("conn").textContent = "simulation not responding"; }
}
poll(); setInterval(poll, 1000);
</script></body></html>
"""


def main(args=None):
    rclpy.init(args=args)
    spin(ControlPanel())


if __name__ == "__main__":
    main()
