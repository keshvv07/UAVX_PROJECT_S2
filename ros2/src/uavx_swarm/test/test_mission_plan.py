"""Level 5's connected-chain policy (mission_plan.py), rule by rule, without a simulator.

Two layers. The unit tests below drive ConnectedPlanner with hand-made fleet snapshots and
check each operating rule directly. `MiniSim` at the bottom then flies the whole Level 5
mission kinematically - UAVs at 5 m/s, the real radio model and quake_town shadowing for
who can hear whom, reports delivered only over a multi-hop route - and checks the rules
on every tick, clean and under the disturbances an operator can inject.
"""
import math
import os

import pytest

from uavx_swarm import common
from uavx_swarm.common import load_scenario, set_origin
from uavx_swarm.mission_plan import ConnectedParams, ConnectedPlanner, PoiView, UavView
from uavx_swarm.radio_model import load_radio_params, predict_pdr
from uavx_swarm.relay_plan import FERRY, LIVE, RelayParams, RelayPlanner
from uavx_swarm.world_map import WorldMap

HERE = os.path.dirname(os.path.abspath(__file__))
LEVEL5 = os.path.join(HERE, "..", "config", "level5.yaml")


@pytest.fixture(autouse=True)
def _restore_origin():
    lat, lon = common.ORIGIN_LAT_DEG, common.ORIGIN_LON_DEG
    yield
    set_origin(lat, lon)


@pytest.fixture()
def sc():
    return load_scenario(LEVEL5)


def make_planner(sc):
    m = WorldMap.for_scenario(sc)
    radio = load_radio_params(sc)
    gcs = tuple(sc["gcs"]["position_m"])
    rp = RelayPlanner(radio, RelayParams(**sc["relay"]), m)
    rp.set_gcs(gcs)
    params = ConnectedParams.from_cfg(sc["gcs_logic"], (sc.get("companion") or {}).get("battery"))
    return ConnectedPlanner(rp, gcs, params), rp, radio, m, gcs


def fleet_on_pads(sc):
    return [UavView(u["id"], (u["pad_m"][0], u["pad_m"][1], 0.0), float(u["cruise_alt_m"]),
                    tuple(u["pad_m"][:2]), state="READY", link="OK") for u in sc["uavs"]]


def pois_of(sc, ids=None, t=0.0):
    out = []
    for p in sc["pois"]:
        if ids is None or p["id"] in ids:
            out.append(PoiView(p["id"], tuple(p["reported_m"]), int(p["priority"]), float(p["survey_s"]),
                               known_t=t))
    return out


def surveyors(plan):
    return [u for u, tasks in plan.tasks.items() if tasks and tasks[0]["kind"] in ("survey", "stage")]


def on_station(u: UavView, anchor) -> UavView:
    return UavView(u.id, tuple(anchor), u.alt, u.pad, batt=u.batt, state="RELAY", link="OK",
                   on_station=True, relay_at=tuple(anchor))


# ---------------------------------------------------------------------- the level itself
def test_level5_runs_the_connected_policy_and_its_geometry_needs_a_ferry(sc):
    """Rule 3 needs a PoI the chain cannot reach, or the ferry is never exercised."""
    assert sc["policy"] == "connected"
    planner, rp, *_ = make_planner(sc)
    gcs = planner.gcs
    need = {p.id: planner.needed(p, 25.0) for p in pois_of(sc)}
    budget = len(sc["uavs"]) - 1
    assert need["P1"] == 0 and need["P2"] <= budget and need["P3"] <= budget
    assert need["P4"] > budget, "P4 must be beyond the chain the two-UAV fleet can build"
    assert not sc.get("disturbances") and not sc.get("hidden_disturbances"), \
        "Level 5 runs clean; the operator injects disturbances from the console"
    assert math.dist(gcs[:2], (231.0, -41.0)) > 200.0


# ---------------------------------------------------------------------- rules 1 and 2
def test_one_surveyor_and_it_waits_for_the_relay_before_leaving_coverage(sc):
    planner, rp, radio, m, gcs = make_planner(sc)
    fleet = fleet_on_pads(sc)
    plan = planner.plan(1.0, fleet, pois_of(sc, {"P1", "P2", "P3"}))
    assert len(surveyors(plan)) == 1, "never more than one surveyor"
    sv = surveyors(plan)[0]
    assert sv == "uav1", "the surveyor whose height is clear of the relay layer"
    task = plan.tasks[sv][0]
    assert task["kind"] == "stage" and task["serves"] == "P3"        # P3 first: priority 3
    stage = (task["pos"][0], task["pos"][1], task["alt"])
    assert predict_pdr(radio, stage, gcs, gcs_link=True, world_map=m) >= 0.85, "it waits inside coverage"
    relay = plan.tasks["uav2"][0]
    assert relay["kind"] == "relay"
    anchor = (relay["pos"][0], relay["pos"][1], relay["alt"])

    # the relay reaches its anchor: now, and only now, the surveyor goes
    fleet[1] = on_station(fleet[1], anchor)
    plan = planner.plan(30.0, fleet, pois_of(sc, {"P1", "P2", "P3"}))
    assert plan.tasks["uav1"][0]["kind"] == "survey" and plan.tasks["uav1"][0]["id"] == "P3"
    assert not plan.tasks["uav1"][0].get("ferry"), "P3 is reported live, not ferried"
    poi = (146.0, -4.0, 25.0)
    assert predict_pdr(radio, poi, anchor, world_map=m) >= 0.85, "the surveyor at P3 is connected"
    assert plan.owners["P3"] == "uav1" and plan.owners.get("P1") is None


def test_a_relay_not_needed_now_but_needed_next_holds_instead_of_flying_home(sc):
    planner, *_ = make_planner(sc)
    fleet = fleet_on_pads(sc)
    pois = pois_of(sc, {"P1", "P2", "P3"})
    plan = planner.plan(1.0, fleet, pois)
    relay = plan.tasks["uav2"][0]
    fleet[1] = on_station(fleet[1], (relay["pos"][0], relay["pos"][1], relay["alt"]))
    planner.plan(30.0, fleet, pois)
    pois = [p for p in pois if p.id != "P3"]                          # P3 done
    plan = planner.plan(90.0, fleet, pois)
    assert plan.tasks["uav1"][0]["id"] == "P1"                        # direct: no relay needed ...
    assert plan.tasks["uav2"] == [], "... but P2 still needs one: it holds where it is, connected"


# ---------------------------------------------------------------------- rules 3 and 4
def ferry_setup(sc):
    planner, rp, radio, m, gcs = make_planner(sc)
    fleet = fleet_on_pads(sc)
    pois = pois_of(sc, {"P4"})
    plan = planner.plan(1.0, fleet, pois)
    relay = plan.tasks["uav2"][0]
    anchor = (relay["pos"][0], relay["pos"][1], relay["alt"])
    fleet[1] = on_station(fleet[1], anchor)
    fleet[0] = UavView("uav1", (80.0, -10.0, 25.0), 25.0, fleet[0].pad, state="STAGE", link="OK")
    plan = planner.plan(40.0, fleet, pois)
    return planner, plan, fleet, pois, anchor, radio, m


def test_a_far_poi_is_a_ferry_sortie_and_the_relay_holds_as_far_out_as_it_can(sc):
    planner, plan, fleet, pois, anchor, radio, m = ferry_setup(sc)
    assert plan.chain.mode == FERRY and plan.chain.relays == 1 and plan.chain.needed >= 2
    task = plan.tasks["uav1"][0]
    assert task["kind"] == "survey" and task["id"] == "P4" and task["ferry"] is True
    handoff = tuple(task["handoff"])
    # the handoff is where the last connected UAV can hear it: free air, a real link
    assert predict_pdr(radio, handoff, anchor, world_map=m) >= 0.85
    assert m.free_at(handoff[0], handoff[1], handoff[2], 8.0)
    assert math.dist(handoff[:2], (231.0, -41.0)) < math.dist(anchor[:2], (231.0, -41.0))
    assert plan.tasks["uav2"][0]["kind"] == "relay"
    assert planner.ferry_uav() == "uav1"


def test_while_the_sortie_is_out_the_far_poi_has_one_owner_and_the_chain_waits(sc):
    planner, plan, fleet, pois, anchor, radio, m = ferry_setup(sc)
    relay_task = plan.tasks["uav2"]
    sortie = plan.tasks["uav1"]
    # the surveyor is out of contact; a new near PoI appears
    fleet[0] = UavView("uav1", (200.0, -35.0, 25.0), 25.0, fleet[0].pad, state="NAVIGATE", link="OUT_OF_RANGE")
    more = pois + [PoiView("N1", (60.0, 0.0), 2, 15.0, known_t=60.0)]
    for t in (60.0, 80.0, 100.0):
        plan = planner.plan(t, fleet, more)
        assert plan.tasks["uav2"] == relay_task, "the relay stays on its coordinates and waits"
        assert plan.tasks["uav1"] == sortie
        assert plan.owners.get("P4") == "uav1" and plan.owners.get("N1") is None, \
            "nobody else is allotted the far PoI, and no second job starts"
        assert len(surveyors(plan)) == 1
        assert planner.ferry_expected("uav1", t)


def test_an_overdue_sortie_is_no_longer_expected(sc):
    planner, plan, *_ = ferry_setup(sc)
    due = planner.work.due_t
    assert due is not None and due > 40.0
    assert planner.ferry_expected("uav1", due - 1.0)
    assert not planner.ferry_expected("uav1", due + 1.0)
    assert not planner.ferry_expected("uav2", due - 1.0)


def test_a_sortie_the_battery_cannot_afford_is_deferred_and_live_work_goes_first(sc):
    planner, *_ = make_planner(sc)
    fleet = fleet_on_pads(sc)
    fleet = [UavView(u.id, u.pos, u.alt, u.pad, batt=31.0, state="READY", link="OK") for u in fleet]
    plan = planner.plan(1.0, fleet, pois_of(sc, {"P4", "P1"}))
    kinds = {t[0] for t in plan.events}
    # P1 is live and first anyway; then P4 alone at 31 % battery must be deferred
    assert plan.tasks["uav1"][0]["id"] in ("P1", "STAGE:P1")
    plan = planner.plan(5.0, fleet, pois_of(sc, {"P4"}))
    assert "FERRY_DEFERRED" in {t[0] for t in plan.events} | kinds
    assert not any(tasks and tasks[0].get("ferry") for tasks in plan.tasks.values())


# ---------------------------------------------------------------------- disturbances
def test_a_cut_off_surveyor_keeps_its_poi_until_it_is_declared_lost(sc):
    """With a spare UAV on the pad, a surveyor out of contact must not be duplicated."""
    sc = dict(sc, uavs=list(sc["uavs"]) + [{"id": "uav3", "pad_m": [-25.0, 0.0], "cruise_alt_m": 25.0}])
    planner, *_ = make_planner(sc)
    fleet = fleet_on_pads(sc)
    pois = pois_of(sc, {"P1"})
    plan = planner.plan(1.0, fleet, pois)
    sv = surveyors(plan)[0]
    i = next(k for k, u in enumerate(fleet) if u.id == sv)
    fleet[i] = UavView(sv, (60.0, -20.0, 25.0), fleet[i].alt, fleet[i].pad, state="NAVIGATE", link="OUT_OF_RANGE")
    plan = planner.plan(30.0, fleet, pois)
    assert plan.owners.get("P1") == sv
    assert surveyors(plan) == [sv], "nobody else is sent after P1 while its surveyor may be alive"
    fleet[i] = UavView(sv, (60.0, -20.0, 25.0), fleet[i].alt, fleet[i].pad, state="NAVIGATE", link="LOST")
    plan = planner.plan(120.0, fleet, pois)
    assert plan.owners.get("P1") not in (None, sv), "once it is declared lost, P1 moves on"


def test_losing_the_relay_turns_the_live_job_into_a_ferry_for_the_same_surveyor(sc):
    planner, *_ = make_planner(sc)
    fleet = fleet_on_pads(sc)
    pois = pois_of(sc, {"P3"})
    plan = planner.plan(1.0, fleet, pois)
    relay = plan.tasks["uav2"][0]
    fleet[1] = on_station(fleet[1], (relay["pos"][0], relay["pos"][1], relay["alt"]))
    plan = planner.plan(30.0, fleet, pois)
    assert plan.tasks["uav1"][0]["id"] == "P3"
    fleet[1] = UavView("uav2", fleet[1].pos, 45.0, fleet[1].pad, state="FAILED", link="LOST")
    plan = planner.plan(60.0, fleet, pois)
    assert plan.chain.relays == 0
    assert plan.tasks["uav1"][0]["id"] == "P3"
    assert plan.owners["P3"] == "uav1"
    assert plan.tasks["uav2"] == []


def test_an_emergency_pre_empts_live_work_but_never_a_sortie_in_flight(sc):
    planner, *_ = make_planner(sc)
    fleet = fleet_on_pads(sc)
    pois = pois_of(sc, {"P1"})
    plan = planner.plan(1.0, fleet, pois)
    assert plan.tasks["uav1"][0]["id"] == "P1"
    pois.append(PoiView("E1", (30.0, 40.0), 5, 15.0, known_t=10.0))
    plan = planner.plan(10.0, fleet, pois)
    assert plan.tasks["uav1"][0]["id"] in ("E1", "STAGE:E1")
    assert ("PREEMPTED", {"dropped": "P1", "now": "E1", "priority": 5}) in plan.events

    planner2, plan2, fleet2, pois2, *_ = ferry_setup(sc)
    pois2 = pois2 + [PoiView("E2", (30.0, 40.0), 5, 15.0, known_t=50.0)]
    plan2 = planner2.plan(50.0, fleet2, pois2)
    assert plan2.tasks["uav1"][0]["id"] == "P4", "a sortie in flight cannot be recalled anyway"


# ======================================================================== MiniSim
class SimUav:
    def __init__(self, u):
        self.id, self.alt = u["id"], float(u["cruise_alt_m"])
        self.pad = (float(u["pad_m"][0]), float(u["pad_m"][1]))
        self.pos = [self.pad[0], self.pad[1], 0.0]
        self.state, self.tasks, self.cur = "READY", [], None
        self.t_state, self.on_station, self.relay_at = 0.0, False, None
        self.failed, self.outage_until = False, -1.0
        self.buffer, self.done_ids, self.down_since = [], [], None
        self.handoff = None

    @property
    def airborne(self):
        return self.pos[2] > 1.0


class MiniSim:
    """Kinematic Level 5: the GCS planner as built, simplified companions, the real channel."""
    DT, SPEED, CLIMB = 0.5, 5.0, 2.5

    def __init__(self, sc):
        self.sc = sc
        self.planner, self.rp, self.radio, self.map, self.gcs = make_planner(sc)
        self.uavs = {u["id"]: SimUav(u) for u in sc["uavs"]}
        self.view = {u: {"link": "OK", "last": 0.0, "state": "READY", "pos": tuple(s.pos),
                         "on_station": False, "relay_at": None} for u, s in self.uavs.items()}
        self.pois = {p["id"]: dict(p, status="PENDING", known_t=0.0) for p in sc["pois"] if p["appear_s"] <= 0}
        self.later = [p for p in sc["pois"] if p["appear_s"] > 0]
        self.t = 0.0
        self.fail_timeout = float(sc["gcs_logic"]["fail_timeout_s"])
        self.log = {"off_ferry_down": {u: 0.0 for u in self.uavs}, "max_ferries": 0, "conflicts": [],
                    "reports": {}, "ferried": set(), "lost": [], "relayed": set()}

    # ---------------------------------------------------------------- radio truth
    def _alive(self, uid):
        s = self.uavs[uid]
        return not s.failed and self.t >= s.outage_until

    def reach(self):
        nodes = {"gcs": self.gcs}
        nodes.update({u: tuple(s.pos) for u, s in self.uavs.items() if self._alive(u)})
        adj = {n: set() for n in nodes}
        names = list(nodes)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if predict_pdr(self.radio, nodes[a], nodes[b], gcs_link="gcs" in (a, b),
                               world_map=self.map) >= self.radio.link_up_pdr:
                    adj[a].add(b)
                    adj[b].add(a)
        seen, todo = {"gcs"}, ["gcs"]
        while todo:
            for n in adj[todo.pop()]:
                if n not in seen:
                    seen.add(n)
                    todo.append(n)
        return seen, adj

    # ---------------------------------------------------------------- one tick
    def step(self, plan_now: bool):
        t = self.t
        while self.later and self.later[0]["appear_s"] <= t:
            p = self.later.pop(0)
            self.pois[p["id"]] = dict(p, status="PENDING", known_t=t)
        reach, adj = self.reach()
        for uid, s in self.uavs.items():
            v = self.view[uid]
            if uid in reach:
                if v["link"] == "LOST":
                    v["link"] = "OK"
                v.update(link="OK", last=t, state=s.state, pos=tuple(s.pos), on_station=s.on_station,
                         relay_at=s.relay_at, batt=100.0)
                for pid, det_t in s.buffer:
                    self.log["reports"].setdefault(pid, (uid, t - det_t))
                    if "gcs" not in adj[uid]:
                        self.log["relayed"].add(pid)
                s.buffer = []
                for pid in s.done_ids:
                    if pid in self.pois:
                        self.pois[pid]["status"] = "DONE"
            elif v["link"] == "OK" and t - v["last"] > self.fail_timeout:
                if self.planner.ferry_uav() == uid:
                    v["link"] = "OUT_OF_RANGE"
                else:
                    heard = [w for w in self.uavs if w != uid and self.view[w]["link"] == "OK"
                             and t - self.view[w]["last"] <= self.fail_timeout and self.view[w]["on_station"]]
                    exp = predict_pdr(self.radio, v["pos"], self.gcs, gcs_link=True, world_map=self.map)
                    for w in heard:
                        exp = max(exp, predict_pdr(self.radio, v["pos"], self.view[w]["pos"], world_map=self.map))
                    v["link"] = "LOST" if exp >= 0.5 else "OUT_OF_RANGE"
                    if v["link"] == "LOST":
                        self.log["lost"].append((uid, round(t, 1)))
            elif v["link"] == "OUT_OF_RANGE" and self.planner.ferry_uav() == uid \
                    and not self.planner.ferry_expected(uid, t):
                v["link"] = "LOST"
                self.log["lost"].append((uid, round(t, 1)))
        if plan_now:
            fleet = [UavView(u, v["pos"], self.uavs[u].alt, self.uavs[u].pad, state=v["state"], link=v["link"],
                             on_station=v["on_station"], relay_at=v["relay_at"]) for u, v in self.view.items()]
            pois = [PoiView(p["id"], tuple(p["reported_m"]), p["priority"], p["survey_s"], p["status"],
                            p["known_t"]) for p in self.pois.values()]
            plan = self.planner.plan(t, fleet, pois)
            for uid, tasks in plan.tasks.items():
                if uid in reach:
                    self._apply(self.uavs[uid], [x for x in tasks if x["id"] not in self.uavs[uid].done_ids])
        ferrying = 0
        holders = {}
        for s in self.uavs.values():
            if s.failed:
                continue
            sortie = s.state == "FERRY" or bool(s.cur and s.cur.get("ferry"))
            ferrying += sortie
            if s.state in ("NAVIGATE", "SURVEY") and s.cur:
                holders.setdefault(s.cur["id"], []).append(s.id)
            if s.airborne and not sortie and s.id not in reach and t >= s.outage_until:
                self.log["off_ferry_down"][s.id] += self.DT
            self._fly(s, reach)
        self.log["max_ferries"] = max(self.log["max_ferries"], ferrying)
        for pid, who in holders.items():
            if len(who) > 1:
                self.log["conflicts"].append((pid, who, t))
        self.t += self.DT

    # ---------------------------------------------------------------- simplified companion
    def _apply(self, s, tasks):
        if s.failed:
            return
        if s.state in ("READY", "TAKEOFF"):
            s.tasks = [x for x in tasks if x["kind"] != "park"]
            return
        if s.state == "FERRY":
            s.tasks = tasks
            return
        if s.cur and tasks and tasks[0]["id"] == s.cur["id"]:
            s.cur["pos"] = tasks[0].get("pos", s.cur.get("pos"))
            s.tasks = tasks[1:]
            return
        if s.state == "RETURN" and not [x for x in tasks if x["kind"] != "park"]:
            return
        s.tasks = tasks
        self._next(s)

    def _next(self, s):
        s.cur, s.on_station, s.relay_at = None, False, None
        while s.tasks:
            task = s.tasks.pop(0)
            if task["kind"] == "park":
                s.state = "RETURN"
                return
            s.cur = task
            s.state = {"relay": "RELAY", "stage": "STAGE"}.get(task["kind"], "NAVIGATE")
            if task.get("ferry"):
                s.handoff = tuple(task["handoff"])
            if task["kind"] == "relay":
                s.relay_at = (task["pos"][0], task["pos"][1], task["alt"])
            return
        s.state = "HOLD"

    def _move(self, s, goal, alt):
        dx, dy = goal[0] - s.pos[0], goal[1] - s.pos[1]
        d = math.hypot(dx, dy)
        step = min(d, self.SPEED * self.DT)
        if d > 1e-6:
            s.pos[0] += dx / d * step
            s.pos[1] += dy / d * step
        dz = alt - s.pos[2]
        s.pos[2] += max(-self.CLIMB * self.DT, min(self.CLIMB * self.DT, dz))
        return d < 2.0 and abs(alt - s.pos[2]) < 1.0

    def _fly(self, s, reach):
        t = self.t
        if s.failed:
            s.pos[2] = max(0.0, s.pos[2] - 5.0 * self.DT)
            return
        if s.state == "READY":
            if s.tasks:
                s.state = "TAKEOFF"
            return
        if s.state == "TAKEOFF":
            if self._move(s, s.pos[:2], s.alt):
                self._next(s)
            return
        sortie = s.state == "FERRY" or bool(s.cur and s.cur.get("ferry"))
        connected = s.id in reach
        s.down_since = None if connected or sortie else (s.down_since if s.down_since is not None else t)
        if s.state in ("NAVIGATE", "STAGE", "HOLD") and s.down_since is not None:       # the leash
            down = t - s.down_since
            if down >= 20.0:
                p = self.rp.handoff_point((s.pos[0], s.pos[1], s.alt), self.gcs)
                self._move(s, p, s.alt)
                return
            if down >= 3.0:
                return
        if s.state == "RELAY":
            s.on_station = self._move(s, s.relay_at[:2], s.relay_at[2]) or s.on_station
        elif s.state == "STAGE":
            self._move(s, s.cur["pos"], s.alt)
        elif s.state == "NAVIGATE":
            if self._move(s, s.cur["pos"], s.alt):
                s.state, s.t_state = "SURVEY", t
        elif s.state == "SURVEY":
            if t - s.t_state >= 3.0 and not any(p == s.cur["id"] for p, _ in s.buffer) \
                    and s.cur["id"] not in self.log["reports"]:
                s.buffer.append((s.cur["id"], t))
            if t - s.t_state >= s.cur["survey_s"]:
                s.done_ids.append(s.cur["id"])
                if s.cur.get("ferry"):
                    self.log["ferried"].add(s.cur["id"])
                    s.cur, s.state = None, "FERRY"
                else:
                    self._next(s)
        elif s.state == "FERRY":
            if connected:
                self._next(s)
            else:
                self._move(s, s.handoff[:2], s.alt)
        elif s.state == "RETURN":
            if self._move(s, s.pad, s.alt if math.dist(s.pos[:2], s.pad) > 2.0 else 0.0) or \
                    (math.dist(s.pos[:2], s.pad) < 2.0 and s.pos[2] < 0.5):
                s.pos[2], s.state = 0.0, "READY"

    def run(self, until_s: float, events=(), settle_s: float = 0.0):
        """Fly until every PoI is done (plus `settle_s` more, to see the swarm park)."""
        events = sorted(events, key=lambda e: e[0])
        tick, done_t = 0, None
        while self.t < until_s:
            while events and events[0][0] <= self.t:
                _, fn = events.pop(0)
                fn(self)
            self.step(plan_now=(tick % 2 == 0))
            tick += 1
            if done_t is None and self.pois and not self.later \
                    and all(p["status"] == "DONE" for p in self.pois.values()):
                done_t = self.t
            if done_t is not None and self.t - done_t >= settle_s:
                break
        return self.log


def assert_rules(sim: MiniSim, log: dict, allow_down=()):
    assert all(p["status"] == "DONE" for p in sim.pois.values()), \
        {k: p["status"] for k, p in sim.pois.items()}
    assert set(log["reports"]) == set(sim.pois), "every report reached the GCS"
    assert log["max_ferries"] <= 1, "rule 4: one sortie at a time"
    assert not log["conflicts"], f"rule 4: a PoI flown by two UAVs at once: {log['conflicts'][:3]}"
    for uid, down in log["off_ferry_down"].items():
        if uid not in allow_down:
            assert down <= 1.0, f"rule 1: {uid} spent {down} s out of the network off a sortie"


def test_minisim_clean_run_obeys_every_rule(sc):
    sim = MiniSim(sc)
    log = sim.run(900.0, settle_s=140.0)          # and on, until the idle swarm has landed
    assert_rules(sim, log)
    for uid, down in log["off_ferry_down"].items():
        assert down == 0.0, f"rule 1: {uid} lost the network for {down} s off the sortie"
    for s in sim.uavs.values():
        assert s.state == "READY" and s.pos[2] == 0.0 and math.dist(s.pos[:2], s.pad) < 2.5, \
            f"{s.id} should be parked on its pad: {s.state} at {s.pos}"
    assert log["ferried"] == {"P4"}, "rule 3: the far PoI and only the far PoI is ferried"
    assert {"P2", "P3"} <= log["relayed"], "rule 2: reports came home over the relay"
    for pid, (_, latency) in log["reports"].items():
        if pid != "P4":
            assert latency <= 10.0, f"{pid} took {latency} s"
    assert not log["lost"]


def test_minisim_relay_killed_mid_survey(sc):
    """The operator's kill switch on the relay: nobody doubles up, everything still done."""
    sim = MiniSim(sc)

    def kill(s):
        s.uavs["uav2"].failed = True
    log = sim.run(900.0, events=[(70.0, kill)])
    assert_rules(sim, log, allow_down=("uav1",))
    assert ("uav2" in {u for u, _ in log["lost"]}), "the dead relay is written off"
    assert "uav1" not in {u for u, _ in log["lost"]}, "the surveyor cut off behind it is not"


def test_minisim_relay_radio_outage_is_ridden_through(sc):
    sim = MiniSim(sc)

    def outage(s):
        s.uavs["uav2"].outage_until = s.t + 15.0
    log = sim.run(900.0, events=[(70.0, outage)])
    assert_rules(sim, log, allow_down=("uav1",))
    assert not log["lost"], "a 15 s outage is shorter than fail_timeout_s: nobody is lost"


def test_minisim_operator_drops_an_emergency_poi(sc):
    sim = MiniSim(sc)

    def drop(s):
        s.pois["E1"] = {"id": "E1", "reported_m": [40.0, -45.0], "priority": 5, "survey_s": 15.0,
                        "appear_s": 0.0, "status": "PENDING", "known_t": s.t}
    log = sim.run(900.0, events=[(40.0, drop)])
    assert_rules(sim, log)
    assert "E1" in log["reports"]
