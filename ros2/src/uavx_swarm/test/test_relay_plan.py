"""Relay chains and the partial data ferry - the decision the whole Stage 2 strategy rests on."""
import math
import os

import pytest

from uavx_swarm.radio_model import RadioParams, predict_pdr
from uavx_swarm.relay_plan import BLIND, FERRY, LIVE, RelayParams, RelayPlanner
from uavx_swarm.world_map import WorldMap

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "uavx_gazebo", "maps", "forest_rescue.yaml")
SURVEY_ALT = 35.0


@pytest.fixture(scope="module")
def forest():
    return WorldMap.load(MAP)


@pytest.fixture(scope="module")
def gcs(forest):
    return tuple(forest.gcs_antenna_m)


@pytest.fixture()
def planner(forest, gcs):
    p = RelayPlanner(RadioParams(), RelayParams(), forest)
    p.set_gcs(gcs)
    return p


def test_hop_spacing_comes_from_the_channel_not_a_guess(planner):
    """A hop is 0.9 of the range at which a healthy link still predicts 85 % PDR."""
    assert planner.hop_m == pytest.approx(90.0, abs=1.0)
    # the first hop is shorter because it also has to climb to relay height
    assert planner.gcs_hop_m < planner.hop_m
    assert planner.gcs_hop_m == pytest.approx(80.4, abs=1.5)


def test_every_planned_hop_really_predicts_the_target_pdr(planner, gcs):
    radio = RadioParams()
    for target in [(-250.0, 0.0, SURVEY_ALT), (0.0, 0.0, SURVEY_ALT), (120.0, -90.0, SURVEY_ALT)]:
        points = planner.chain_points(gcs, target)
        chain = [gcs, *points, target]
        for i in range(len(chain) - 1):
            pdr = predict_pdr(radio, chain[i], chain[i + 1], gcs_link=(i == 0))
            assert pdr >= planner.p.min_planned_pdr - 1e-6, (target, i, pdr)


def test_the_relay_counts_that_decide_the_strategy(planner, gcs):
    """These three numbers are why a 4-5 UAV fleet ferries instead of chaining."""
    assert planner.relays_needed(gcs, (-250.0, 0.0, SURVEY_ALT)) == 1      # near tree line
    assert planner.relays_needed(gcs, (0.0, 0.0, SURVEY_ALT)) == 4         # area centre
    assert planner.relays_needed(gcs, (250.0, 250.0, SURVEY_ALT)) == 7     # far corner


def test_relay_anchors_sit_inside_the_fence_and_clear_of_obstacles(planner, gcs, forest):
    for target in [(0.0, 0.0, SURVEY_ALT), (200.0, 150.0, SURVEY_ALT)]:
        for p in planner.chain_points(gcs, target):
            assert forest.inside_fence(p[0], p[1], p[2])
            assert forest.free_at(p[0], p[1], p[2], 8.0)
            assert p[2] == planner.p.relay_alt_m


def test_relay_height_keeps_a_separation_margin_from_the_surveyors(planner):
    """A relay holding station must never be the reason a separation minimum is broken."""
    assert planner.p.relay_alt_m - SURVEY_ALT >= 20.0


def test_a_chain_that_fits_is_flown_live(planner, gcs):
    chain = planner.decide("T1", gcs, (0.0, 0.0, SURVEY_ALT), budget=4)
    assert chain.mode == LIVE
    assert chain.relays == chain.needed == 4
    assert chain.handoff is None
    assert chain.tip is not None


def test_a_chain_that_does_not_fit_becomes_a_partial_ferry(planner, gcs):
    chain = planner.decide("T9", gcs, (250.0, 250.0, SURVEY_ALT), budget=4)
    assert chain.mode == FERRY
    assert chain.needed == 7 and chain.relays == 4            # as far out as the fleet allows
    assert chain.handoff is not None
    # the handoff is between the target and the chain tip, and nearer the target
    d_target = math.dist(chain.handoff[:2], chain.target[:2])
    d_tip = math.dist(chain.handoff[:2], chain.tip[:2])
    assert d_target < math.dist(chain.target[:2], chain.tip[:2])
    assert d_tip < 100.0


def test_with_no_relay_available_it_falls_back_to_store_and_forward(planner, gcs):
    chain = planner.decide("T9", gcs, (250.0, 250.0, SURVEY_ALT), budget=0)
    assert chain.mode == BLIND
    assert chain.relays == 0
    assert chain.handoff is not None                          # still worth flying towards home


def test_the_handoff_point_is_inside_the_relays_radio_range(planner):
    """"Come back into communication range of the nearest UAV", in numbers."""
    radio = RadioParams()
    node = (-60.0, 100.0, 55.0)
    far = (240.0, 240.0, SURVEY_ALT)
    h = planner.handoff_point(far, node)
    assert predict_pdr(radio, h, node) >= planner.p.min_planned_pdr * 0.9
    # it is the NEAREST such point: a metre further out and the link is worse
    beyond = (h[0] + (far[0] - h[0]) * 0.08, h[1] + (far[1] - h[1]) * 0.08, h[2])
    assert math.dist(beyond[:2], node[:2]) > math.dist(h[:2], node[:2])
    assert h[2] == pytest.approx(far[2])                      # at the ferrying UAV's own level


def test_a_node_already_in_range_needs_no_trip(planner):
    node = (0.0, 0.0, 55.0)
    here = (20.0, 10.0, SURVEY_ALT)
    assert planner.handoff_point(here, node)[:2] == pytest.approx(here[:2], abs=0.1)


def test_assignment_fills_the_whole_chain_or_nothing(planner, gcs):
    chain = planner.decide("T1", gcs, (0.0, 0.0, SURVEY_ALT), budget=4)
    fleet = [{"id": f"uav{i}", "pos": (-356.0, -15.0 + 6 * i, 30.0), "batt": 100.0,
              "relaying_at": None} for i in range(1, 5)]
    picks = planner.assign(chain, fleet)
    assert len(picks) == len(chain.points)
    assert set(picks) <= {f["id"] for f in fleet}
    assert len({tuple(p) for p in picks.values()}) == len(chain.points)   # no shared anchor
    # one UAV short, or all of them flat: a partial chain connects nothing, so refuse
    assert planner.assign(chain, fleet[:3]) == {}
    flat = [dict(f, batt=10.0) for f in fleet]
    assert planner.assign(chain, flat) == {}


def test_a_relay_already_on_station_keeps_its_anchor(planner, gcs):
    chain = planner.decide("T1", gcs, (0.0, 0.0, SURVEY_ALT), budget=4)
    anchor = chain.points[-1]
    sticky = {"id": "uav9", "pos": (anchor[0] + 5, anchor[1], 55.0), "batt": 60.0,
              "relaying_at": anchor}
    others = [{"id": f"uav{i}", "pos": (-356.0, 0.0, 30.0), "batt": 100.0, "relaying_at": None}
              for i in range(1, 4)]
    picks = planner.assign(chain, [sticky] + others)
    assert tuple(picks["uav9"]) == tuple(anchor)


def test_chain_serialises_for_the_task_message(planner, gcs):
    chain = planner.decide("T9", gcs, (250.0, 250.0, SURVEY_ALT), budget=2)
    d = chain.to_dict()
    assert d["mode"] == FERRY and d["needed"] == 7 and len(d["points"]) == 2
    assert all(isinstance(v, list) and len(v) == 3 for v in d["points"])
    assert isinstance(d["handoff"], list)
