"""A node's attachment to the emulated mesh radio (radio_net), with multi-hop forwarding.

    send(dst, kind, payload)                  best effort (heartbeats, status)
    send(dst, kind, payload, reliable=True)   ARQ: retransmitted every retry_s until the
                                              destination ACKs it - store-and-forward for
                                              free while there is no route at all

Routing. The endpoint asks a callback (`next_hop`, wired to this node's RouteTable) where
to send a frame addressed to a node that is not a neighbour. The frame then carries
`final_dst` and a `ttl`, and every node it passes through forwards it on the same way, so
a report from a surveyor four hops out reaches the GCS without anybody holding a map of
the network. Frames are forwarded on behalf of other nodes too - that is what being a
relay means at this layer - and each forward is counted so the referee can show how much
traffic a relay actually carried.

With no route at all, a reliable frame stays in the outbox and is retried; that is the
store-and-forward fallback that gets a report home when the UAV flies back into contact,
and `flush` sends the backlog the moment a route appears.

Every frame carries a per-sender sequence number (PDR accounting and neighbour ETX) and
the sim time the ORIGINAL sender created it, so end-to-end latency measured at the
destination is the organisers' detection-to-report time (Q23, Q29) however many hops it
took.
"""
from __future__ import annotations

import json
from collections import OrderedDict
from typing import Callable, Optional

from uavx_interfaces.msg import RadioPacket

from .common import RADIO_QOS, dumps, now_s

DEFAULT_TTL = 8


class RadioEndpoint:
    def __init__(self, node, node_id: str,
                 on_receive: Callable[[RadioPacket, dict], None],
                 on_acked: Optional[Callable[[dict, float], None]] = None,
                 retry_s: float = 1.0, max_retry_s: float = 4.0, callback_group=None,
                 next_hop: Optional[Callable[[str], str]] = None,
                 on_forward: Optional[Callable[[RadioPacket, str], None]] = None,
                 on_any: Optional[Callable[[RadioPacket], None]] = None,
                 ttl: int = DEFAULT_TTL):
        self.node = node
        self.id = node_id
        self.on_receive = on_receive
        self.on_acked = on_acked
        # Called for EVERY frame that arrives, including ones meant for somebody else: this
        # is how a node measures its neighbours (routing.RouteTable.on_frame).
        self.on_any = on_any
        self.retry_s = retry_s
        self.max_retry_s = max_retry_s
        self.enabled = True
        self.ttl = int(ttl)
        # Where to send a frame for `final_dst`. The default treats every destination as a
        # direct neighbour, which is exactly the single-hop behaviour of Levels 3-4.
        self.next_hop = next_hop or (lambda final_dst: final_dst)
        self.on_forward = on_forward
        self._seq = 0
        self._msg_counter = 0
        self.outbox: "OrderedDict[str, dict]" = OrderedDict()
        self._seen: "OrderedDict[tuple[str, str], float]" = OrderedDict()
        # Reverse-path learning: a frame that arrived from node X came through neighbour N,
        # so a reply to X goes back through N. Without this, an ACK for a report that came
        # four hops in would be routed towards the GCS - the wrong way - and the sender
        # would retransmit until it flew home. Standard, cheap, and it expires on its own.
        self.reverse: dict[str, tuple[str, float]] = {}
        self.reverse_ttl_s = 20.0
        self.stats = {"tx": 0, "rx": 0, "retx": 0, "dup": 0, "acked": 0,
                      "forwarded": 0, "undeliverable": 0, "ttl_dropped": 0}
        self._pub = node.create_publisher(RadioPacket, "/radio/tx", RADIO_QOS)
        node.create_subscription(RadioPacket, f"/{node_id}/radio/rx", self._on_rx, RADIO_QOS,
                                 callback_group=callback_group)

    # ------------------------------------------------------------- sending
    def send(self, dst: str, kind: str, payload: dict, reliable: bool = False) -> Optional[str]:
        if not self.enabled:
            return None
        msg_id = ""
        stamp = self.node.get_clock().now().to_msg()
        if reliable:
            self._msg_counter += 1
            msg_id = f"{self.id}-{self._msg_counter}"
            self.outbox[msg_id] = {"dst": dst, "kind": kind, "payload": payload, "stamp": stamp,
                                   "first_t": now_s(self.node), "attempt": 1,
                                   "next_t": now_s(self.node) + self.retry_s}
        self._emit(dst, kind, payload, msg_id, stamp, 1)
        return msg_id or None

    def _emit(self, final_dst: str, kind, payload, msg_id, stamp, attempt) -> bool:
        """Put a frame of our own on the air, routed towards final_dst. False: no route."""
        hop = final_dst if final_dst == "*" else self.next_hop(final_dst)
        if not hop:
            self.stats["undeliverable"] += 1
            return False
        self._seq += 1
        pkt = RadioPacket()
        pkt.src = self.id
        pkt.dst = hop
        pkt.final_dst = final_dst
        pkt.seq = self._seq
        pkt.kind = kind
        pkt.msg_id = msg_id
        pkt.payload = payload if isinstance(payload, str) else dumps(payload)
        pkt.stamp = stamp
        pkt.attempt = min(int(attempt), 255)
        pkt.hops = 0
        pkt.ttl = self.ttl
        pkt.route = [self.id]
        self._pub.publish(pkt)
        self.stats["tx"] += 1
        return True

    def tick(self) -> None:
        """Retransmit reliable frames whose ACK is overdue (call from a timer)."""
        if not self.enabled:
            return
        t = now_s(self.node)
        for msg_id, e in list(self.outbox.items()):
            if t >= e["next_t"]:
                e["attempt"] += 1
                backoff = min(self.retry_s * (1.5 ** (e["attempt"] - 2)), self.max_retry_s)
                e["next_t"] = t + backoff
                if self._emit(e["dst"], e["kind"], e["payload"], msg_id, e["stamp"], e["attempt"]):
                    self.stats["retx"] += 1

    def flush(self) -> int:
        """Retransmit everything still unacknowledged on the next tick (a route just appeared)."""
        t = now_s(self.node)
        for e in self.outbox.values():
            e["next_t"] = t
        return len(self.outbox)

    def cancel(self, predicate: Callable[[dict], bool]) -> int:
        dropped = [k for k, e in self.outbox.items() if predicate(e)]
        for k in dropped:
            del self.outbox[k]
        return len(dropped)

    @property
    def pending(self) -> int:
        return len(self.outbox)

    def oldest_pending_age_s(self) -> float:
        """How long the oldest unacknowledged frame has been waiting (0 with none)."""
        if not self.outbox:
            return 0.0
        return now_s(self.node) - min(e["first_t"] for e in self.outbox.values())

    # ------------------------------------------------------------- receiving
    def reverse_hop(self, node_id: str, t: float) -> str:
        """The neighbour a frame from `node_id` last came through, while that is still fresh."""
        hop, seen = self.reverse.get(node_id, ("", -1e9))
        return hop if t - seen <= self.reverse_ttl_s else ""

    def _learn_reverse(self, pkt: RadioPacket) -> None:
        if pkt.src == self.id:
            return
        came_from = (list(pkt.route) or [pkt.src])[-1]
        if came_from and came_from != self.id:
            self.reverse[pkt.src] = (came_from, now_s(self.node))

    def _on_rx(self, pkt: RadioPacket) -> None:
        if not self.enabled:
            return
        if self.on_any is not None:
            self.on_any(pkt)
        self._learn_reverse(pkt)
        final = pkt.final_dst or pkt.dst
        if final not in ("*", self.id):
            self._forward(pkt, final)
            return
        self.stats["rx"] += 1
        try:
            payload = json.loads(pkt.payload) if pkt.payload else {}
        except ValueError:
            payload = {}
        if pkt.kind == "ACK":
            entry = self.outbox.pop(payload.get("msg_id", ""), None)
            if entry is not None:
                self.stats["acked"] += 1
                if self.on_acked:
                    self.on_acked(entry, now_s(self.node) - entry["first_t"])
            return
        if pkt.msg_id:
            # ACK the ORIGINAL sender, routed the same way the frame came.
            self._emit(pkt.src, "ACK", {"msg_id": pkt.msg_id}, "",
                       self.node.get_clock().now().to_msg(), 1)
            key = (pkt.src, pkt.msg_id)
            if key in self._seen:
                self.stats["dup"] += 1
                return
            self._seen[key] = now_s(self.node)
            while len(self._seen) > 2000:
                self._seen.popitem(last=False)
        self.on_receive(pkt, payload)

    def _forward(self, pkt: RadioPacket, final: str) -> None:
        """Pass somebody else's frame along towards `final` - the relay's real job."""
        if pkt.ttl <= 1:
            self.stats["ttl_dropped"] += 1
            return
        if self.id in list(pkt.route)[:-1]:        # we have already carried this frame
            self.stats["dup"] += 1
            return
        hop = self.next_hop(final)
        if not hop or hop == pkt.src:              # no route on, or straight back where it came from
            self.stats["undeliverable"] += 1
            return
        out = RadioPacket()
        out.src = pkt.src
        out.dst = hop
        out.final_dst = final
        out.seq = pkt.seq
        out.kind = pkt.kind
        out.msg_id = pkt.msg_id
        out.payload = pkt.payload
        out.stamp = pkt.stamp                      # end-to-end latency keeps the original time
        out.attempt = pkt.attempt
        out.hops = min(int(pkt.hops) + 1, 255)
        out.ttl = int(pkt.ttl) - 1
        out.route = list(pkt.route) + [self.id]
        self._pub.publish(out)
        # A forwarded frame is a frame this radio really transmitted, so it counts in `tx`
        # as well: otherwise a relay's own airtime is invisible in the packet log.
        self.stats["forwarded"] += 1
        self.stats["tx"] += 1
        if self.on_forward:
            self.on_forward(pkt, hop)


def packet_age_s(node, pkt: RadioPacket) -> float:
    """Sim seconds since the ORIGINAL sender created this frame (end-to-end latency)."""
    sent = pkt.stamp.sec + pkt.stamp.nanosec * 1e-9
    return now_s(node) - sent
