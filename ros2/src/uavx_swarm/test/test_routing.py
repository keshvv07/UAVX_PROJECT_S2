"""Decentralised ETX routing: neighbour measurement, next-hop choice, loops, staleness."""
import math

import pytest

from uavx_swarm.routing import GCS_COST, RouteParams, RouteTable, link_cost


def beacons(table: RouteTable, src: str, t0: float, count: int, drop_every: int = 0,
            step: float = 1.0) -> float:
    """Feed `count` 1 Hz beacons from `src`, dropping every `drop_every`-th one."""
    t = t0
    for i in range(1, count + 1):
        t = t0 + i * step
        if drop_every and i % drop_every == 0:
            continue                      # lost frame: the sequence number gaps
        table.on_frame(src, i, t)
    return t


def test_link_cost_penalises_a_weak_hop():
    p = RouteParams()
    assert link_cost(1.0, p) == pytest.approx(1.0)
    assert link_cost(0.8, p) == pytest.approx(1.25)                 # healthy: plain ETX
    assert link_cost(0.5, p) == pytest.approx(2.0 + p.weak_link_penalty)   # weak: penalised
    assert link_cost(0.6, p) == pytest.approx(1 / 0.6 + p.weak_link_penalty)
    assert link_cost(p.min_link_pdr, p) == pytest.approx(1 / p.min_link_pdr)
    assert link_cost(0.0, p) > 100.0


def test_ratio_comes_from_the_gaps_in_the_sequence():
    r = RouteTable("uav1")
    t = beacons(r, "gcs", 0.0, 10)
    assert r.neighbours["gcs"].pdr == pytest.approx(1.0)
    r2 = RouteTable("uav1")
    t2 = beacons(r2, "gcs", 0.0, 10, drop_every=2)
    assert 0.3 < r2.neighbours["gcs"].pdr < 0.8
    assert t > 0 and t2 > 0


def test_direct_link_to_the_gcs_wins_when_it_is_good():
    r = RouteTable("uav1")
    t = beacons(r, "gcs", 0.0, 6)
    r.on_advert("gcs", GCS_COST, t=t)
    r.update(t)
    assert r.connected and r.direct_to_gcs
    assert r.hops == 1
    assert r.cost == pytest.approx(1.0, abs=0.3)


def test_a_relay_is_used_when_the_gcs_is_out_of_reach():
    r = RouteTable("uav1")
    t = beacons(r, "uav2", 0.0, 6)
    r.on_advert("uav2", 1.0, next_hop="gcs", hops=1, t=t)
    r.update(t)
    assert r.connected and r.next_hop == "uav2" and r.hops == 2
    assert r.cost > 1.0


def test_a_good_two_hop_route_beats_a_marginal_direct_one():
    """Plain ETX would take the single weak hop; the penalty is what stops it."""
    r = RouteTable("uav1")
    t = beacons(r, "gcs", 0.0, 10, drop_every=2)       # ~0.5 PDR: below min_link_pdr
    beacons(r, "uav2", 0.0, 10)                        # a clean neighbour
    r.on_advert("gcs", GCS_COST, t=t)
    r.on_advert("uav2", 1.0, next_hop="gcs", hops=1, t=t)
    r.update(t)
    assert r.next_hop == "uav2"


def test_hysteresis_keeps_a_route_that_is_only_slightly_worse():
    r = RouteTable("uav1")
    t = beacons(r, "uav2", 0.0, 8)
    beacons(r, "uav3", 0.0, 8)
    r.on_advert("uav2", 1.0, next_hop="gcs", hops=1, t=t)
    r.on_advert("uav3", 1.0, next_hop="gcs", hops=1, t=t)
    r.update(t)
    first = r.next_hop
    other = "uav3" if first == "uav2" else "uav2"
    # the alternative becomes marginally cheaper: within the 15 % band, so nothing moves
    r.on_advert(other, 0.95, next_hop="gcs", hops=1, t=t)
    changed = r.update(t)
    assert not changed and r.next_hop == first
    # clearly cheaper: now it switches
    r.on_advert(other, 0.2, next_hop="gcs", hops=1, t=t)
    assert r.update(t)
    assert r.next_hop == other


def test_a_neighbour_that_routes_through_us_is_never_a_next_hop():
    r = RouteTable("uav1")
    t = beacons(r, "uav2", 0.0, 6)
    r.on_advert("uav2", 0.5, next_hop="uav1", hops=1, t=t)     # it points back at us
    r.update(t)
    assert not r.connected


def test_a_silent_neighbour_is_dropped_within_a_couple_of_seconds():
    r = RouteTable("uav1", params=RouteParams(stale_s=3.0))
    t = beacons(r, "gcs", 0.0, 6)
    r.on_advert("gcs", GCS_COST, t=t)
    r.update(t)
    assert r.connected
    r.update(t + 4.0)                                          # nothing heard since
    assert not r.connected and r.next_hop == ""


def test_hop_limit_is_respected():
    r = RouteTable("uav1", params=RouteParams(max_hops=3))
    t = beacons(r, "uav2", 0.0, 6)
    r.on_advert("uav2", 3.0, next_hop="uav3", hops=3, t=t)     # would make us the 4th hop
    r.update(t)
    assert not r.connected


def test_losing_the_relay_disconnects_and_regaining_it_reconnects():
    r = RouteTable("uav1")
    t = beacons(r, "uav2", 0.0, 6)
    r.on_advert("uav2", 1.0, next_hop="gcs", hops=1, t=t)
    r.update(t)
    assert r.connected
    r.drop("uav2")
    assert not r.connected
    t2 = beacons(r, "uav2", t + 1.0, 6)
    r.on_advert("uav2", 1.0, next_hop="gcs", hops=1, t=t2)
    r.update(t2)
    assert r.connected and r.next_hop == "uav2"


def test_best_relay_ignores_the_gcs_and_picks_the_cheapest_peer():
    r = RouteTable("uav1")
    t = beacons(r, "gcs", 0.0, 6)
    beacons(r, "uav2", 0.0, 6)
    beacons(r, "uav3", 0.0, 6)
    r.on_advert("gcs", GCS_COST, t=t)
    r.on_advert("uav2", 4.0, next_hop="gcs", hops=1, t=t)
    r.on_advert("uav3", 2.0, next_hop="gcs", hops=1, t=t)
    who, cost = r.best_relay(t)
    assert who == "uav3" and cost == pytest.approx(2.0)


def test_advert_is_what_a_status_broadcast_carries():
    r = RouteTable("uav1")
    assert r.advert() == {"cost": None, "next_hop": "", "hops": 0}
    t = beacons(r, "gcs", 0.0, 6)
    r.on_advert("gcs", GCS_COST, t=t)
    r.update(t)
    ad = r.advert()
    assert ad["next_hop"] == "gcs" and ad["hops"] == 1 and ad["cost"] is not None
    assert not math.isinf(r.cost)
