"""Every scenario is self-consistent, and the organisers' constraints are enforced somewhere.

This is the test that would have caught the two mistakes that cost a run rather than a
criterion: a scenario pointing at a world whose geographic origin disagrees with the frame
the autopilot is commanded in, and a `pass:` block asking the referee for a check it does
not implement. Runs without a simulator or a ROS 2 install.
"""
import math
import os

import pytest
import yaml

from uavx_swarm import common          # the ENU origin is module state: read it through here
from uavx_swarm.common import load_scenario, set_origin
from uavx_swarm.coverage import partition
from uavx_swarm.radio_model import load_radio_params
from uavx_swarm.world_map import WorldMap

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", ".."))
CONFIG = os.path.join(SRC, "uavx_swarm", "config")
GAZEBO = os.path.join(SRC, "uavx_gazebo")
SCENARIOS = sorted(f[:-5] for f in os.listdir(CONFIG)
                   if f.endswith(".yaml") and f != "radio.yaml")

# Everything the referee (mission_monitor._criteria) knows how to check.
PASS_KEYS = {
    "all_landed", "allow_failure_of", "pad_error_m", "max_speed_mps", "max_alt_m",
    "min_separation_m", "waypoints_reached", "pois_detected", "geolocation_error_m",
    "images_saved", "reports_at_gcs", "disturbances_handled", "report_latency_s",
    "live_report_latency_s", "live_pois", "ferry_pois", "pois_completed",
    "reallocations_min", "max_lost_declarations", "failure_detection_max_s",
    "status_pdr_min", "emergency_poi", "coverage_fraction_min", "sectors_covered",
    "multi_hop_reports_min", "relays_on_station_min", "ferry_delivered",
    "max_report_hops", "comm_range_limit", "endurance_respected",
    # connected-chain rules (policy: connected, Level 5)
    "connected_report_latency_s", "max_concurrent_ferries", "single_owner",
    "network_connectivity_min", "relayed_reports_min",
}
POLICIES = {"legacy", "connected"}
COMPONENTS = {"bench_check", "companion", "poi_detector", "radio_net", "gcs",
              "scenario_manager", "control_panel", "pov_viewer", "mission_monitor"}


@pytest.fixture(autouse=True)
def _restore_origin():
    """load_scenario pins the global ENU origin, so put it back between tests."""
    lat, lon = common.ORIGIN_LAT_DEG, common.ORIGIN_LON_DEG
    yield
    set_origin(lat, lon)


@pytest.mark.parametrize("name", SCENARIOS)
def test_scenario_loads_and_is_consistent(name):
    sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
    assert sc["name"], f"{name} has no name"
    assert sc["duration_s"] > 0
    assert set(sc["components"]) <= COMPONENTS, set(sc["components"]) - COMPONENTS
    assert set(sc["pass"]) <= PASS_KEYS, f"{name}: referee cannot check {set(sc['pass']) - PASS_KEYS}"
    ids = [u["id"] for u in sc["uavs"]]
    assert len(ids) == len(set(ids)), f"{name} has duplicate UAV ids"
    sysids = [u["sysid"] for u in sc["uavs"]]
    assert len(sysids) == len(set(sysids))
    pads = [tuple(u["pad_m"][:2]) for u in sc["uavs"]]
    assert len(pads) == len(set(pads)), f"{name}: two UAVs share a pad"
    for a in range(len(pads)):
        for b in range(a + 1, len(pads)):
            assert math.dist(pads[a], pads[b]) >= 5.0, f"{name}: pads {a} and {b} are too close"
    assert os.path.isfile(os.path.join(GAZEBO, "worlds", f"{sc['world']}.sdf")), \
        f"{name}: no world file for {sc['world']}"
    assert sc.get("policy", "legacy") in POLICIES, f"{name}: unknown policy {sc.get('policy')!r}"


@pytest.mark.parametrize("name", SCENARIOS)
def test_map_and_world_agree_on_the_frame(name):
    """A map's geographic origin must match its world's <spherical_coordinates>.

    If they disagree, every position goal is converted to a latitude and longitude in the
    wrong place and the swarm flies off the map - a failure no criterion would explain.
    """
    sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
    if not sc.get("map"):
        return
    m = WorldMap.load(os.path.join(GAZEBO, "maps", f"{sc['map']}.yaml"))
    frame = m.data.get("frame")
    world = open(os.path.join(GAZEBO, "worlds", f"{sc['world']}.sdf"), encoding="utf-8").read()
    import re
    lat = float(re.search(r"<latitude_deg>([^<]+)</latitude_deg>", world).group(1))
    lon = float(re.search(r"<longitude_deg>([^<]+)</longitude_deg>", world).group(1))
    if frame:
        assert frame["latitude_deg"] == pytest.approx(lat, abs=1e-6), f"{name}: map/world latitude"
        assert frame["longitude_deg"] == pytest.approx(lon, abs=1e-6), f"{name}: map/world longitude"
        # and load_scenario must actually have applied it
        assert common.ORIGIN_LAT_DEG == pytest.approx(lat, abs=1e-6), \
            f"{name}: load_scenario did not pin the ENU frame to the world's origin"
        assert common.ORIGIN_LON_DEG == pytest.approx(lon, abs=1e-6)


@pytest.mark.parametrize("name", SCENARIOS)
def test_pads_and_pois_are_inside_the_fence(name):
    sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
    if not sc.get("map"):
        return
    m = WorldMap.load(os.path.join(GAZEBO, "maps", f"{sc['map']}.yaml"))
    for u in sc["uavs"]:
        assert m.inside_fence(u["pad_m"][0], u["pad_m"][1], 0.0), f"{name}: {u['id']} pad outside the fence"
        assert u["cruise_alt_m"] <= m.fence_alt_m, f"{name}: {u['id']} cruises above the ceiling"
    for p in sc["pois"]:
        assert m.inside_fence(p["pos_m"][0], p["pos_m"][1], 0.0), f"{name}: {p['id']} outside the fence"


def test_radio_is_the_organisers_radio():
    """Q13: no scenario may quietly widen the 100 m range."""
    for name in SCENARIOS:
        sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
        p = load_radio_params(sc)
        assert p.max_range_m <= 100.0, f"{name}: radio range {p.max_range_m} m exceeds the limit"


# ------------------------------------------------------------------ the GC1 mission
@pytest.fixture()
def gc1():
    return load_scenario(os.path.join(CONFIG, "gc1.yaml"))


def test_gc1_flies_the_organisers_map_with_four_to_five_uavs(gc1):
    assert gc1["map"] == "forest_rescue" and gc1["world"] == "uavx_forest"
    assert 4 <= len(gc1["uavs"]) <= 5                     # Q10/Q44: a justified small fleet
    m = WorldMap.load(os.path.join(GAZEBO, "maps", "forest_rescue.yaml"))
    assert tuple(gc1["gcs"]["position_m"]) == tuple(m.gcs_antenna_m)
    pads = {tuple(p) for p in m.pads}
    for u in gc1["uavs"]:
        assert tuple(u["pad_m"][:2]) in pads, f"{u['id']} does not start on a real pad"


def test_gc1_honours_every_numeric_constraint(gc1):
    """The quick-reference table from the technical session, checked against the scenario."""
    assert gc1["ap_params"]["WP_SPD"] <= 10            # Q12: 10 m/s recommended
    assert gc1["pass"]["max_alt_m"] <= 100             # 100 m ceiling
    assert gc1["companion"]["battery"]["endurance_min"] == 20.0        # Q7
    assert gc1["pass"]["live_report_latency_s"] == 10.0               # Q23
    assert load_radio_params(gc1).max_range_m == 100.0                # Q13
    assert gc1["companion"]["cruise_speed_mps"] <= gc1["ap_params"]["WP_SPD"]
    assert gc1["gcs_logic"]["speed_mps"] <= gc1["ap_params"]["WP_SPD"]
    # the fence has to reach the far corner of the search area from the pads
    far = math.dist((gc1["uavs"][0]["pad_m"][0], gc1["uavs"][0]["pad_m"][1]), (250.0, 250.0))
    assert gc1["ap_params"]["FENCE_RADIUS"] > far


def test_gc1_sectors_fit_inside_one_battery(gc1):
    """Four quadrants at 10 m/s, with room left for a ferry trip and the flight home."""
    from uavx_swarm.coverage import lane_spacing_m, lanes, sweep_time_s
    m = WorldMap.load(os.path.join(GAZEBO, "maps", "forest_rescue.yaml"))
    gcs = tuple(gc1["gcs"]["position_m"])
    sectors = partition(m.mission_area, gc1["gcs_logic"]["sectors"], gcs)
    spacing = lane_spacing_m(gc1["gcs_logic"]["survey_alt_m"], gc1["gcs_logic"]["sweep_overlap"])
    endurance_s = gc1["companion"]["battery"]["endurance_min"] * 60.0
    for s in sectors:
        pts = lanes(s, spacing, inset_m=spacing / 2, start_near=gcs)
        out_and_back = sweep_time_s(pts, gc1["gcs_logic"]["speed_mps"], gcs) \
            + math.dist(pts[-1], gcs[:2]) / gc1["gcs_logic"]["speed_mps"]
        assert out_and_back < 0.7 * endurance_s, f"{s.id} leaves no margin: {out_and_back:.0f} s"


def test_gc1_relay_heights_keep_the_separation_minimum(gc1):
    """A relay holding station must not break the rule the referee scores."""
    gap = gc1["relay"]["relay_alt_m"] - gc1["gcs_logic"]["survey_alt_m"]
    assert gap >= gc1["pass"]["min_separation_m"]
    assert gc1["relay"]["relay_alt_m"] <= gc1["pass"]["max_alt_m"]
    # and every UAV transits at its own height, so five can leave one apron
    alts = sorted(u["cruise_alt_m"] for u in gc1["uavs"])
    assert len(set(alts)) == len(alts)
    assert all(b - a >= 3.0 for a, b in zip(alts, alts[1:]))


def test_gc1_target_field_matches_how_the_organisers_place_theirs(gc1):
    t = gc1["targets"]
    assert t["count"] == 10                            # five site types, two of each
    assert t["min_separation_m"] >= 15.0               # their ROI_MIN_SEPARATION_M
    assert t["at_start"] < t["count"]                  # Q20: not all available at the start
    assert t["appear_window_s"][0] >= 30.0             # their ROI_APPEAR_WINDOW_S lower bound
    assert t["appear_window_s"][1] <= gc1["gcs_logic"]["poi_window_s"]


def test_no_scenario_scripts_a_disturbance():
    """Disturbances (UAV kill, comms degradation, new PoIs) come only from the control console."""
    for name in SCENARIOS:
        sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
        assert not sc.get("disturbances"), f"{name} scripts disturbances"
        assert not sc.get("hidden_disturbances"), f"{name} draws hidden disturbances"


def test_gc1_seed_is_random_unless_given(gc1):
    assert gc1["seed"] == "random"                     # a fresh PoI field unless --seed is given


def test_gc1_asks_the_referee_for_the_organisers_metrics(gc1):
    """Q42: the evaluation parameters have to be scored, not just logged."""
    p = gc1["pass"]
    for key in ("coverage_fraction_min", "pois_detected", "pois_completed", "reports_at_gcs",
                "live_report_latency_s", "comm_range_limit", "endurance_respected",
                "multi_hop_reports_min", "ferry_delivered", "disturbances_handled",
                "min_separation_m", "max_speed_mps"):
        assert key in p, f"gc1 does not ask the referee to check {key}"


def test_control_panel_runs_in_the_mission_scenarios():
    """Q32-Q35: the evaluator needs the panel, so it has to be in `components:`."""
    for name in ("level3", "level4", "level5", "gc1", "demo3d"):
        sc = load_scenario(os.path.join(CONFIG, f"{name}.yaml"))
        assert "control_panel" in sc["components"], f"{name} has no control panel"


def test_raw_yaml_has_no_duplicate_keys():
    """A duplicated key in YAML silently wins, which is how a constraint gets lost."""
    class NoDuplicates(yaml.SafeLoader):
        pass

    def construct(loader, node, deep=False):
        seen = set()
        for key_node, _ in node.value:
            key = loader.construct_object(key_node, deep=deep)
            assert key not in seen, f"duplicate key {key!r}"
            seen.add(key)
        return yaml.SafeLoader.construct_mapping(loader, node, deep)

    NoDuplicates.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct)
    for name in SCENARIOS:
        with open(os.path.join(CONFIG, f"{name}.yaml"), encoding="utf-8") as fh:
            yaml.load(fh, NoDuplicates)
