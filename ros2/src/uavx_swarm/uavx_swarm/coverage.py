"""Area coverage: how a 500 m x 500 m search area is split up and swept.

The organisers give a search area, not a list of waypoints (Q6, Q19-Q20: the Points of
Interest appear at times nobody tells the swarm in advance), so a surveyor's real job is
to *cover ground* and let its camera find what is there. This module turns the mission
area from the map into flyable work:

    swath_m(alt, camera)   how wide a strip the nadir camera actually sees from `alt`
    partition(area, n)     the area split into n compact sectors, nearest the GCS first
    lanes(sector, spacing) a boustrophedon (lawnmower) path covering one sector
    SweepPlan              that path plus how far along it a UAV has got, so a UAV that
                           breaks off to deliver its reports can pick up where it stopped

Everything is pure geometry and has no ROS dependency, so it is unit-tested without a
simulator.

Why the lane spacing matters. The uavx_quad's camera is 640 x 480 with a 1.2 rad
horizontal field of view, mounted nadir with image "up" along the nose: the cross-track
width on the ground is 2 h tan(0.6) = 1.37 h, so 48 m from 35 m up. Lanes are spaced at
(1 - overlap) of that, which both guarantees a gap-free sweep and leaves margin for the
position error the aircraft actually flies with.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# uavx_quad survey camera (uavx_gazebo/models/uavx_quad/model.sdf)
DEFAULT_HFOV_RAD = 1.2
DEFAULT_WIDTH_PX = 640
DEFAULT_HEIGHT_PX = 480


def swath_m(alt_m: float, hfov_rad: float = DEFAULT_HFOV_RAD,
            width_px: int = DEFAULT_WIDTH_PX, height_px: int = DEFAULT_HEIGHT_PX) -> tuple[float, float]:
    """(cross-track, along-track) ground footprint of the nadir camera from `alt_m`."""
    half = math.tan(hfov_rad / 2.0)
    cross = 2.0 * alt_m * half
    along = 2.0 * alt_m * half * (height_px / width_px)
    return cross, along


def lane_spacing_m(alt_m: float, overlap: float = 0.3, **camera) -> float:
    """Distance between neighbouring lanes for a given overlap fraction (0..0.9)."""
    cross, _ = swath_m(alt_m, **camera)
    return max(5.0, cross * (1.0 - min(max(overlap, 0.0), 0.9)))


@dataclass(frozen=True)
class Sector:
    """A rectangle of the search area, in world ENU metres."""
    id: str
    x0: float
    x1: float
    y0: float
    y1: float

    @property
    def centre(self) -> tuple[float, float]:
        return (0.5 * (self.x0 + self.x1), 0.5 * (self.y0 + self.y1))

    @property
    def area_m2(self) -> float:
        return (self.x1 - self.x0) * (self.y1 - self.y0)

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return self.x0 - margin <= x <= self.x1 + margin and self.y0 - margin <= y <= self.y1 + margin

    def to_list(self) -> list[float]:
        return [round(self.x0, 1), round(self.x1, 1), round(self.y0, 1), round(self.y1, 1)]


def partition(area, n: int, origin=(0.0, 0.0)) -> list[Sector]:
    """Split `area` (x0, x1, y0, y1) into `n` sectors, nearest `origin` first.

    The split is the most square grid that gives at least n cells, which keeps each
    sector compact - a surveyor spends its battery on lanes, not on crossing its own
    sector. Any surplus cells are merged into their neighbour so exactly n come back.
    Ordering by distance from the ground station means sector 0 is the one a UAV can
    reach soonest, and the farthest sector is the one the relay chain has to work for.
    """
    x0, x1, y0, y1 = (float(v) for v in area)
    n = max(1, int(n))
    span_x, span_y = x1 - x0, y1 - y0
    best = None
    for cols in range(1, n + 1):
        rows = math.ceil(n / cols)
        w, h = span_x / cols, span_y / rows
        score = (max(w, h) / max(min(w, h), 1e-9), cols * rows - n)
        if best is None or score < best[0]:
            best = (score, cols, rows)
    _, cols, rows = best

    cells: list[Sector] = []
    for r in range(rows):
        for c in range(cols):
            cells.append(Sector(f"S{len(cells)}", x0 + c * span_x / cols, x0 + (c + 1) * span_x / cols,
                                y0 + r * span_y / rows, y0 + (r + 1) * span_y / rows))
    # merge surplus cells into the row neighbour on their left, so coverage stays complete
    while len(cells) > n:
        victim = cells.pop()
        for i, s in enumerate(cells):
            if abs(s.y0 - victim.y0) < 1e-6 and abs(s.x1 - victim.x0) < 1e-6:
                cells[i] = Sector(s.id, s.x0, victim.x1, s.y0, s.y1)
                break
        else:
            s = cells[-1]
            cells[-1] = Sector(s.id, min(s.x0, victim.x0), max(s.x1, victim.x1),
                               min(s.y0, victim.y0), max(s.y1, victim.y1))
    cells.sort(key=lambda s: math.dist(s.centre, origin[:2]))
    return [Sector(f"S{i + 1}", s.x0, s.x1, s.y0, s.y1) for i, s in enumerate(cells)]


def lanes(sector: Sector, spacing_m: float, inset_m: float = 0.0, start_near=None) -> list[tuple[float, float]]:
    """Boustrophedon waypoints covering `sector`, lanes along its longer side.

    `inset_m` pulls the outermost lanes in from the edge (half a swath already covers it).
    `start_near` picks which corner to begin at, so a UAV does not cross its own sector to
    reach the first lane.
    """
    x0, x1 = sector.x0 + inset_m, sector.x1 - inset_m
    y0, y1 = sector.y0 + inset_m, sector.y1 - inset_m
    if x1 < x0:
        x0 = x1 = 0.5 * (sector.x0 + sector.x1)
    if y1 < y0:
        y0 = y1 = 0.5 * (sector.y0 + sector.y1)
    along_y = (y1 - y0) >= (x1 - x0)
    cross0, cross1 = (x0, x1) if along_y else (y0, y1)
    run0, run1 = (y0, y1) if along_y else (x0, x1)

    span = cross1 - cross0
    count = max(1, int(math.ceil(span / max(spacing_m, 1e-6))) + 1)
    step = span / (count - 1) if count > 1 else 0.0
    offsets = [cross0 + i * step for i in range(count)]
    if start_near is not None:
        near_cross = start_near[0] if along_y else start_near[1]
        near_run = start_near[1] if along_y else start_near[0]
        if abs(near_cross - cross1) < abs(near_cross - cross0):
            offsets.reverse()
        if abs(near_run - run1) < abs(near_run - run0):
            run0, run1 = run1, run0

    out: list[tuple[float, float]] = []
    for i, c in enumerate(offsets):
        a, b = (run0, run1) if i % 2 == 0 else (run1, run0)
        for r in (a, b):
            out.append((round(c, 1), round(r, 1)) if along_y else (round(r, 1), round(c, 1)))
    return out


def path_length_m(points) -> float:
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


def sweep_time_s(points, speed_mps: float, start=None) -> float:
    """Rough time to fly a lane path at `speed_mps`, including reaching its first point."""
    if not points:
        return 0.0
    lead = math.dist(start[:2], points[0]) if start is not None else 0.0
    return (lead + path_length_m(points)) / max(speed_mps, 0.1)


@dataclass
class SweepPlan:
    """A sector's lane path plus the index of the next waypoint still to fly.

    `advance` is called as the UAV reaches each waypoint. Breaking off (to deliver reports
    through a relay, or to recharge) does not touch the index, so `resume_point` is still
    the place to come back to - this is what makes a partial data ferry cheap: the surveyor
    gives up a round trip, not its progress.
    """
    sector_id: str
    points: list[tuple[float, float]]
    index: int = 0
    covered_m: float = 0.0
    laps: int = field(default=0)

    @property
    def done(self) -> bool:
        return self.index >= len(self.points)

    @property
    def remaining(self) -> int:
        return max(0, len(self.points) - self.index)

    @property
    def next_point(self):
        return None if self.done else self.points[self.index]

    @property
    def resume_point(self):
        """Where to pick the sweep up again (the waypoint that was being flown to)."""
        return self.next_point

    @property
    def progress(self) -> float:
        return 1.0 if not self.points else round(min(1.0, self.index / len(self.points)), 3)

    def advance(self) -> None:
        if self.done:
            return
        if self.index > 0:
            self.covered_m += math.dist(self.points[self.index - 1], self.points[self.index])
        self.index += 1

    def restart(self) -> None:
        """Sweep the sector again (nothing found and there is battery left for another pass)."""
        self.index = 0
        self.laps += 1

    def to_dict(self) -> dict:
        return {"sector": self.sector_id, "waypoints": len(self.points), "index": self.index,
                "progress": self.progress, "covered_m": round(self.covered_m, 1), "laps": self.laps}
