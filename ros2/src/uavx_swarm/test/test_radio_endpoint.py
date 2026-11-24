"""Multi-hop forwarding, ARQ and the reverse path, on a three-node chain with no simulator.

The nodes here are a few lines of test harness that pass RadioPacket objects to each other
directly; radio_net's channel is not involved, so what is being checked is purely the
endpoint logic - routing a frame beyond its next hop, forwarding on somebody else's behalf,
dropping duplicates and loops, acknowledging back along the path the frame came in on, and
holding a report until a route exists.

If uavx_interfaces is not built (a machine with no ROS 2), a stand-in message class is
registered first. In a ROS 2 environment the real generated message is used.
"""
import sys
import types

import pytest

if "uavx_interfaces.msg" not in sys.modules:
    try:
        import uavx_interfaces.msg                              # noqa: F401
    except Exception:
        pkg = types.ModuleType("uavx_interfaces")
        msg = types.ModuleType("uavx_interfaces.msg")

        class RadioPacket:                                      # mirrors msg/RadioPacket.msg
            def __init__(self):
                self.src = self.dst = self.final_dst = ""
                self.kind = self.msg_id = self.payload = ""
                self.seq = 0
                self.stamp = None
                self.attempt = self.hops = self.ttl = 0
                self.route = []

        msg.RadioPacket = RadioPacket
        pkg.msg = msg
        sys.modules["uavx_interfaces"] = pkg
        sys.modules["uavx_interfaces.msg"] = msg

from uavx_swarm.radio_endpoint import RadioEndpoint     # noqa: E402


class Clock:
    def __init__(self, bus):
        self.bus = bus

    def now(self):
        bus = self.bus

        class T:
            nanoseconds = int(bus.t * 1e9)

            def to_msg(self):
                class S:
                    sec = int(bus.t)
                    nanosec = int((bus.t % 1) * 1e9)
                return S()
        return T()


class Bus:
    """A perfect radio: every frame published reaches the node it is addressed to."""

    def __init__(self):
        self.t = 0.0
        self.inboxes: dict[str, list] = {}
        self.nodes: dict[str, "FakeNode"] = {}
        self.sent: list = []
        self.links: set = set()       # (a, b) pairs allowed to carry frames; empty = all

    def allow(self, a: str, b: str) -> None:
        self.links.add(frozenset((a, b)))

    def _reaches(self, a: str, b: str) -> bool:
        return not self.links or frozenset((a, b)) in self.links

    def publish(self, pkt) -> None:
        self.sent.append(pkt)
        hop_src = (list(pkt.route) or [pkt.src])[-1]
        targets = [n for n in self.nodes if n != hop_src] if pkt.dst == "*" else [pkt.dst]
        for dst in targets:
            if dst in self.nodes and self._reaches(hop_src, dst):
                self.inboxes.setdefault(dst, []).append(pkt)

    def deliver(self, rounds: int = 6) -> None:
        for _ in range(rounds):
            pending = {k: v for k, v in self.inboxes.items() if v}
            if not pending:
                return
            self.inboxes = {}
            for dst, frames in pending.items():
                for pkt in frames:
                    self.nodes[dst].endpoint._on_rx(pkt)


class FakeNode:
    def __init__(self, bus: Bus, node_id: str):
        self.bus = bus
        self.id = node_id
        self._clock = Clock(bus)
        bus.nodes[node_id] = self
        self.received: list = []
        self.acked: list = []
        self.endpoint = None

    def get_clock(self):
        return self._clock

    def create_publisher(self, *a, **k):
        node = self

        class Pub:
            def publish(self, pkt):
                node.bus.publish(pkt)
        return Pub()

    def create_subscription(self, *a, **k):
        return None


def make(bus: Bus, node_id: str, next_hop=None) -> FakeNode:
    node = FakeNode(bus, node_id)
    node.endpoint = RadioEndpoint(
        node, node_id,
        on_receive=lambda pkt, payload: node.received.append((pkt.kind, pkt.src, payload, pkt.hops)),
        on_acked=lambda entry, latency: node.acked.append((entry["kind"], latency)),
        next_hop=next_hop)
    return node


@pytest.fixture()
def chain():
    """gcs <-> relay <-> surveyor: the end nodes cannot hear each other."""
    bus = Bus()
    gcs = make(bus, "gcs", next_hop=lambda d: "relay" if d == "uav1" else d)
    relay = make(bus, "relay", next_hop=lambda d: d)
    uav = make(bus, "uav1", next_hop=lambda d: "relay" if d == "gcs" else d)
    bus.allow("gcs", "relay")
    bus.allow("relay", "uav1")
    return bus, gcs, relay, uav


def test_a_report_reaches_the_gcs_through_the_relay(chain):
    bus, gcs, relay, uav = chain
    uav.endpoint.send("gcs", "POI_REPORT", {"poi_id": "T1"}, reliable=True)
    bus.deliver()
    kinds = [r[0] for r in gcs.received]
    assert "POI_REPORT" in kinds
    kind, src, payload, hops = next(r for r in gcs.received if r[0] == "POI_REPORT")
    assert src == "uav1" and payload["poi_id"] == "T1"
    assert hops == 1                                   # it was forwarded once
    # two frames carried: the report outbound and the GCS's ACK back
    assert relay.endpoint.stats["forwarded"] == 2
    assert relay.received == []                        # the relay did not consume either


def test_the_ack_comes_back_along_the_path_the_frame_took(chain):
    """Without reverse-path learning the ACK would be routed the wrong way and the
    surveyor would retransmit until it flew home."""
    bus, gcs, relay, uav = chain
    uav.endpoint.send("gcs", "POI_REPORT", {"poi_id": "T1"}, reliable=True)
    bus.deliver()
    assert uav.endpoint.pending == 0                   # acknowledged end to end
    assert uav.acked and uav.acked[0][0] == "POI_REPORT"
    assert relay.endpoint.reverse_hop("uav1", bus.t) == "uav1"
    assert gcs.endpoint.reverse_hop("uav1", bus.t) == "relay"


def test_a_report_with_no_route_waits_and_is_flushed(chain):
    bus, gcs, relay, uav = chain
    uav.endpoint.next_hop = lambda d: ""               # the relay has gone
    uav.endpoint.send("gcs", "POI_REPORT", {"poi_id": "T2"}, reliable=True)
    bus.deliver()
    assert uav.endpoint.pending == 1
    assert uav.endpoint.stats["undeliverable"] >= 1
    assert not any(r[0] == "POI_REPORT" for r in gcs.received)
    assert uav.endpoint.oldest_pending_age_s() >= 0.0
    # the relay comes back: the backlog goes out on the next tick
    uav.endpoint.next_hop = lambda d: "relay" if d == "gcs" else d
    bus.t += 1.0
    uav.endpoint.flush()
    uav.endpoint.tick()
    bus.deliver()
    assert uav.endpoint.pending == 0
    assert any(r[0] == "POI_REPORT" for r in gcs.received)


def test_a_duplicate_is_delivered_once(chain):
    bus, gcs, relay, uav = chain
    uav.endpoint.send("gcs", "POI_REPORT", {"poi_id": "T3"}, reliable=True)
    bus.deliver()
    bus.t += 2.0
    uav.endpoint.outbox["uav1-1"] = {"dst": "gcs", "kind": "POI_REPORT",
                                     "payload": {"poi_id": "T3"},
                                     "stamp": uav.get_clock().now().to_msg(),
                                     "first_t": 0.0, "attempt": 1, "next_t": 0.0}
    uav.endpoint.tick()
    bus.deliver()
    assert sum(1 for r in gcs.received if r[0] == "POI_REPORT") == 1
    assert gcs.endpoint.stats["dup"] >= 1


def test_a_frame_sent_straight_back_is_refused_not_looped():
    """The cheapest loop of all - two nodes pointing at each other - costs one frame."""
    bus = Bus()
    a = make(bus, "a", next_hop=lambda d: "b")
    b = make(bus, "b", next_hop=lambda d: "a")
    a.endpoint.send("nowhere", "STATUS", {})
    bus.deliver(rounds=12)
    assert b.endpoint.stats["undeliverable"] >= 1       # b will not hand it back to a
    assert b.endpoint.stats["forwarded"] == 0


def test_ttl_stops_a_frame_that_keeps_moving_but_never_arrives():
    """A chain of nodes that each believe the next one is closer: the hop budget ends it."""
    bus = Bus()
    ring = ["n0", "n1", "n2", "n3", "n4"]
    nodes = {}
    for i, name in enumerate(ring):
        nxt = ring[(i + 1) % len(ring)]
        nodes[name] = make(bus, name, next_hop=(lambda d, nxt=nxt: nxt))
    nodes["n0"].endpoint.ttl = 3
    nodes["n0"].endpoint.send("nowhere", "STATUS", {})
    bus.deliver(rounds=15)
    dropped = sum(n.endpoint.stats["ttl_dropped"] for n in nodes.values())
    forwarded = sum(n.endpoint.stats["forwarded"] for n in nodes.values())
    assert dropped >= 1, "the frame was never stopped by its hop budget"
    assert forwarded <= 3, f"a frame outlived its ttl ({forwarded} forwards)"


def test_a_node_does_not_carry_the_same_frame_twice():
    bus = Bus()
    a = make(bus, "a", next_hop=lambda d: "b")
    b = make(bus, "b", next_hop=lambda d: "c")
    c = make(bus, "c", next_hop=lambda d: "b")         # c sends it back to b
    a.endpoint.send("d", "STATUS", {})
    bus.deliver(rounds=10)
    assert b.endpoint.stats["forwarded"] <= 1
    assert b.endpoint.stats["dup"] >= 1


def test_a_broadcast_is_not_forwarded(chain):
    """STATUS broadcasts are how neighbours measure each other - flooding them would
    make every hop look like a neighbour."""
    bus, gcs, relay, uav = chain
    uav.endpoint.send("*", "STATUS", {"batt": 90})
    bus.deliver()
    assert any(r[0] == "STATUS" for r in relay.received)
    assert not any(r[0] == "STATUS" for r in gcs.received)      # two hops away, not relayed
    assert relay.endpoint.stats["forwarded"] == 0


def test_end_to_end_latency_keeps_the_original_timestamp(chain):
    bus, gcs, relay, uav = chain
    uav.endpoint.send("gcs", "POI_REPORT", {"poi_id": "T4"}, reliable=True)
    first = bus.sent[0]
    bus.t += 3.0
    bus.deliver()
    delivered = next(p for p in bus.sent if p.final_dst == "gcs" and p.kind == "POI_REPORT"
                     and p.hops == 1)
    assert delivered.stamp.sec == first.stamp.sec           # not restamped by the relay
    assert delivered.src == "uav1"


def test_stats_account_for_what_a_relay_carried(chain):
    bus, gcs, relay, uav = chain
    for i in range(4):
        uav.endpoint.send("gcs", "TASK_DONE", {"poi_id": f"T{i}"}, reliable=True)
        bus.deliver()
    assert relay.endpoint.stats["forwarded"] >= 4            # reports out
    assert relay.endpoint.stats["tx"] >= 4
    assert uav.endpoint.stats["acked"] == 4
