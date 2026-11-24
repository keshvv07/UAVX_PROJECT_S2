"""Area coverage: camera swath, sector split, lane paths and resuming after a ferry trip."""
import math

import pytest

from uavx_swarm import coverage as cov

AREA = (-250.0, 250.0, -250.0, 250.0)       # the organisers' 500 m x 500 m
GCS = (-359.0, -26.0, 10.0)


def test_swath_matches_the_uavx_quad_camera():
    """640 x 480 at a 1.2 rad horizontal field of view, nadir: 1.37 h across track."""
    cross, along = cov.swath_m(35.0)
    assert cross == pytest.approx(2 * 35.0 * math.tan(0.6), rel=1e-9)
    assert cross == pytest.approx(47.9, abs=0.1)
    assert along == pytest.approx(cross * 480 / 640, rel=1e-9)
    assert cov.swath_m(70.0)[0] == pytest.approx(2 * cov.swath_m(35.0)[0])


def test_lane_spacing_leaves_the_asked_for_overlap():
    cross, _ = cov.swath_m(35.0)
    assert cov.lane_spacing_m(35.0, overlap=0.0) == pytest.approx(cross)
    assert cov.lane_spacing_m(35.0, overlap=0.3) == pytest.approx(cross * 0.7)
    assert cov.lane_spacing_m(35.0, overlap=0.3) == pytest.approx(33.5, abs=0.1)


@pytest.mark.parametrize("n", [1, 2, 3, 4, 5, 6, 8])
def test_partition_returns_n_sectors_that_tile_the_area(n):
    sectors = cov.partition(AREA, n, GCS)
    assert len(sectors) == n
    assert sum(s.area_m2 for s in sectors) == pytest.approx(500.0 * 500.0, rel=1e-6)
    # every point of the area falls in some sector
    for x in range(-249, 250, 37):
        for y in range(-249, 250, 37):
            assert any(s.contains(x, y) for s in sectors), (x, y)


def test_partition_puts_the_nearest_sector_first():
    sectors = cov.partition(AREA, 4, GCS)
    dists = [math.dist(s.centre, GCS[:2]) for s in sectors]
    assert dists == sorted(dists)
    assert sectors[0].centre[0] < sectors[-1].centre[0]      # nearest is the western one


def test_quadrants_are_square_for_four_uavs():
    sectors = cov.partition(AREA, 4, GCS)
    for s in sectors:
        assert (s.x1 - s.x0, s.y1 - s.y0) == (250.0, 250.0)


def test_lanes_cover_their_sector_within_half_a_swath():
    sector = cov.partition(AREA, 4, GCS)[0]
    spacing = cov.lane_spacing_m(35.0)
    pts = cov.lanes(sector, spacing, inset_m=spacing / 2)
    assert len(pts) % 2 == 0                                  # one pair of ends per lane
    cross, _ = cov.swath_m(35.0)
    # any point in the sector is within half the camera swath of some lane leg
    for x in range(int(sector.x0) + 2, int(sector.x1), 23):
        for y in range(int(sector.y0) + 2, int(sector.y1), 23):
            best = min(_point_to_segment((x, y), pts[i], pts[i + 1])
                       for i in range(0, len(pts) - 1))
            assert best <= cross / 2, (x, y, best)


def _point_to_segment(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(ax + t * dx - p[0], ay + t * dy - p[1])


def test_lanes_start_at_the_corner_nearest_the_uav():
    sector = cov.partition(AREA, 4, GCS)[0]
    spacing = cov.lane_spacing_m(35.0)
    from_gcs = cov.lanes(sector, spacing, start_near=GCS)
    from_far = cov.lanes(sector, spacing, start_near=(sector.x1, sector.y1))
    assert math.dist(from_gcs[0], GCS[:2]) < math.dist(from_far[0], GCS[:2])


def test_sweep_time_is_what_the_mission_budget_assumes():
    """One quadrant at 10 m/s has to fit inside a 20 minute battery with room for a ferry."""
    sector = cov.partition(AREA, 4, GCS)[0]
    pts = cov.lanes(sector, cov.lane_spacing_m(35.0), inset_m=cov.lane_spacing_m(35.0) / 2)
    assert cov.path_length_m(pts) < 2500.0
    assert cov.sweep_time_s(pts, 10.0, GCS) < 400.0


def test_sweep_plan_resumes_where_it_broke_off():
    """The point of the partial ferry: a round trip costs progress, not the sweep."""
    sector = cov.partition(AREA, 4, GCS)[0]
    plan = cov.SweepPlan(sector.id, cov.lanes(sector, 40.0))
    total = len(plan.points)
    for _ in range(3):
        plan.advance()
    broke_off_at = plan.resume_point
    assert plan.index == 3
    assert not plan.done
    # ... a ferry trip happens here, which does not touch the plan ...
    assert plan.resume_point == broke_off_at
    assert plan.remaining == total - 3
    assert 0.0 < plan.progress < 1.0
    while not plan.done:
        plan.advance()
    assert plan.progress == 1.0
    assert plan.covered_m > 0.0
    plan.restart()
    assert (plan.index, plan.laps, plan.done) == (0, 1, False)


def test_sweep_plan_handles_an_empty_path():
    plan = cov.SweepPlan("S0", [])
    assert plan.done and plan.next_point is None and plan.progress == 1.0
    plan.advance()                                            # must not raise
