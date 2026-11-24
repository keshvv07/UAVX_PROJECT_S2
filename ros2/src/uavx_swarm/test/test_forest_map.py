"""The organisers' forest site, checked against the numbers they published.

Every assertion here is a figure from the Grand Challenge 1 technical session or from the
simulation environment the organisers shared, so if a regeneration of the map ever drifts
away from their geometry these tests fail rather than the mission quietly flying the wrong
area. Runs without a simulator.
"""
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from uavx_swarm.radio_model import RadioParams, predict_pdr
from uavx_swarm.world_map import WorldMap

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO = os.path.join(HERE, "..", "..", "uavx_gazebo")
MAP = os.path.join(GAZEBO, "maps", "forest_rescue.yaml")
WORLD = os.path.join(GAZEBO, "worlds", "uavx_forest.sdf")


@pytest.fixture(scope="module")
def forest():
    return WorldMap.load(MAP)


def test_search_area_is_500_by_500_on_the_origin(forest):
    """Q6: the area to cover is 500 m x 500 m, not the 1 km x 1 km first circulated."""
    x0, x1, y0, y1 = forest.mission_area
    assert (x1 - x0, y1 - y0) == (500.0, 500.0)
    assert (x0, x1, y0, y1) == (-250.0, 250.0, -250.0, 250.0)
    assert forest.inside_area(0.0, 0.0) and forest.inside_area(249.0, -249.0)
    assert not forest.inside_area(260.0, 0.0)


def test_ground_station_sits_365_m_west_with_100_m_of_open_ground(forest):
    """uavsim site.py: the apron's front edge is 100 m short of the tree line."""
    assert forest.gcs_antenna_m == [-359.0, -26.0, 10.0]      # the mast, not the apron centre
    assert len(forest.pads) == 24                             # 4 rows x 6 columns
    xs = sorted({p[0] for p in forest.pads})
    ys = sorted({p[1] for p in forest.pads})
    assert xs == [-374.0, -368.0, -362.0, -356.0]             # pad rows
    assert ys == [-15.0, -9.0, -3.0, 3.0, 9.0, 15.0]          # pad columns, 6 m apart
    apron_front = max(xs) + 6.0                               # half a pad pitch + margin
    assert forest.mission_area[0] - apron_front == pytest.approx(100.0, abs=1.0)


def test_geofence_covers_the_area_and_the_transit_corridor(forest):
    assert forest.inside_fence(-365.0, 0.0, 30.0)             # over the apron
    assert forest.inside_fence(249.0, 249.0, 99.0)            # the far corner
    assert not forest.inside_fence(0.0, 0.0, 120.0)           # Q: 100 m ceiling
    assert not forest.inside_fence(500.0, 0.0, 30.0)


def test_the_forest_is_the_organisers_forest(forest):
    assert forest.canopy.count == 726                         # roi_forest: 436 oak + 290 pine
    oaks = sum(1 for t in forest.canopy.trees if t[2] == 0)
    assert (oaks, forest.canopy.count - oaks) == (436, 290)
    assert forest.canopy.top_m == 6.5                         # the oak collision cylinder
    names = {s.name for s in forest.canopy.species}
    assert names == {"oak", "pine"}
    oak = next(s for s in forest.canopy.species if s.name == "oak")
    assert (oak.radius_m, oak.height_m) == (5.0, 6.5)
    assert all(-250.0 <= x <= 250.0 and -250.0 <= y <= 250.0 for x, y, _ in forest.canopy.trees)


def test_trees_are_not_obstacles_at_survey_height(forest):
    """A 6.5 m canopy is nothing to fly round at 35 m, and the planner must not pretend
    otherwise - 726 phantom obstacles would make every A* call pointless and slow."""
    x, y, _ = forest.canopy.tree(0)
    assert forest.free_at(x, y, 35.0, clearance=8.0)
    assert not forest.free_at(x, y, 8.0, clearance=4.0)       # but at treetop height it is real
    path = forest.plan((-200.0, 0.0), (200.0, 0.0), 35.0, clearance=8.0)
    assert path == [(200.0, 0.0)]                             # straight across, no detour


def test_canopy_attenuates_a_low_radio_path(forest):
    """A link that dips into the crowns loses signal; one above them does not."""
    x, y, sp = forest.canopy.tree(0)
    through = forest.canopy.loss_db((x - 40.0, y, 3.0), (x + 40.0, y, 3.0))
    over = forest.canopy.loss_db((x - 40.0, y, 40.0), (x + 40.0, y, 40.0))
    assert through > 0.0
    assert over == 0.0
    assert forest.canopy.loss_db((x - 40, y, 3), (x + 40, y, 3)) <= forest.canopy.max_loss_db


def test_a_hundred_metre_link_is_the_hard_limit(forest):
    """Q13: teams cannot assume a range greater than 100 m, with or without the forest."""
    p = RadioParams()
    a = (0.0, 0.0, 35.0)
    assert predict_pdr(p, a, (99.0, 0.0, 35.0), world_map=forest) > 0.8
    assert predict_pdr(p, a, (101.0, 0.0, 35.0), world_map=forest) == 0.0


def test_relay_count_to_the_far_corner_is_why_the_ferry_exists(forest):
    """The arithmetic the whole communication strategy rests on."""
    gcs = tuple(forest.gcs_antenna_m)
    assert math.dist(gcs[:2], (-250.0, 0.0)) == pytest.approx(112.0, abs=2.0)
    assert math.dist(gcs[:2], (0.0, 0.0)) == pytest.approx(360.0, abs=2.0)
    assert math.dist(gcs[:2], (250.0, 250.0)) == pytest.approx(669.0, abs=3.0)


def test_world_generation_writes_every_tree(forest, tmp_path):
    out = forest.write_world(WORLD, str(tmp_path / "world.sdf"))
    world = ET.parse(out).getroot().find("world")
    assert world.get("name") == "uavx_forest"
    assert not any(m.get("name").startswith("map_") for m in world.findall("model"))
    assert len(world.findall(".//collision/geometry/plane")) == 1
    gcs = world.find("model[@name='gcs_site']")
    gx, gy = map(float, gcs.findtext("pose").split()[:2])
    for i, pad in enumerate(forest.pads):
        pose = gcs.findtext(f"link/visual[@name='pad_{i}']/pose")
        x, y = map(float, pose.split()[:2])
        assert (gx + x, gy + y) == tuple(pad)
    scenery = world.find("model[@name='forest_scenery']/link")
    trees = []
    for collision in scenery.findall("collision"):
        species = collision.get("name").split("_")[0]
        if species in ("Oak", "Pine"):
            x, y = map(float, collision.findtext("pose").split()[:2])
            trees.append((round(x, 2), round(y, 2), int(species == "Pine")))
    assert trees == [tuple(tree) for tree in forest.canopy.trees]
    assert len(scenery.findall("visual/geometry/mesh")) == 2 * len(trees)
    assert sum(v.get("name").startswith("grass_") for v in scenery.findall("visual")) == 1156
    resources = {n.text for tag in ("uri", "albedo_map") for n in world.iter(tag)}
    for uri in resources:
        assert uri.startswith("model://")
        assert (Path(GAZEBO) / "models" / uri.removeprefix("model://")).is_file()


def test_target_placement_keeps_clear_of_trunks(forest):
    """How the organisers place their search-and-rescue sites: clear of the canopy so the
    marker can be seen from above, and 15 m apart."""
    import random
    rng = random.Random(11)
    placed = []
    for _ in range(10):
        p = forest.ground_free_point(rng, (-235, -235, 235, 235), min_tree_m=6.0,
                                     avoid=placed, avoid_r=15.0)
        assert p is not None
        assert forest.inside_area(p[0], p[1])
        near = min(math.dist(p, forest.canopy.tree(i)[:2])
                   for i in range(forest.canopy.count))
        assert near > 6.0
        for q in placed:
            assert math.dist(p, q) >= 15.0
        placed.append(p)
