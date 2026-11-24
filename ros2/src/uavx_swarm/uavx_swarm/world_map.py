"""3-D map of the mission site - one description used by every part of the stack.

A map file (uavx_gazebo/maps/<name>.yaml) describes the site: the mission area, the
ground station, the launch pads, 3-D obstacles (buildings, towers, ridges) and - for the
organisers' forest site - a tree canopy. The same description is

  * written into the Gazebo world (sdf_models)             -> physics, camera, rangefinder
  * avoided by each UAV's companion (plan)                  -> A* on a grid at the UAV's
                                                              flight level, obstacles
                                                              inflated by the clearance,
                                                              then line-of-sight shortcut
  * attenuating radio links that cross it (los_loss_db)     -> radio_net and the GCS
  * swept by the coverage planner (mission_area)            -> survey lanes
  * checked by the referee (contains / distance / geofence) -> collisions, clearance, fence

Frame: world ENU metres, heights above ground. Which point is the origin is the map's
own business: `quake_town` puts it at the GCS, the organisers' `forest_rescue` puts it at
the centre of the 500 m x 500 m search area with the GCS 365 m to the west. Nothing in
the stack assumes one or the other - positions come from the map.

Tilted (collapsed) slabs are planned and attenuated as their upright bounding box of the
given height, which is conservative.
"""
from __future__ import annotations

import heapq
import math
import os
from dataclasses import dataclass, field

import yaml

# Fallback geofence for a scenario with no map at all (the early levels): the Stage 1
# operational area. A map overrides it with its own `geofence:` block.
GEOFENCE = (-200.0, 1100.0, -550.0, 550.0)


def _gazebo_share() -> str:
    """uavx_gazebo's share directory, or its source tree when ROS 2 is absent (tests)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("uavx_gazebo")
    except Exception:
        src = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        path = os.path.join(src, "uavx_gazebo")
        if os.path.isdir(path):
            return path
        raise


@dataclass
class Obstacle:
    id: str
    shape: str                      # "box" | "cylinder"
    cx: float
    cy: float
    height: float
    length: float = 0.0             # box: along its yaw axis
    width: float = 0.0
    radius: float = 0.0             # cylinder
    yaw: float = 0.0                # rad
    roll: float = 0.0               # visual tilt of collapsed slabs (rad)
    pitch: float = 0.0
    kind: str = "building"
    attenuation_db: float = 20.0
    color: tuple = (0.55, 0.53, 0.5)
    bound_r: float = field(init=False)

    def __post_init__(self):
        self.bound_r = self.radius if self.shape == "cylinder" else 0.5 * math.hypot(self.length, self.width)

    def _local(self, x: float, y: float):
        dx, dy = x - self.cx, y - self.cy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def horizontal_gap(self, x: float, y: float) -> float:
        """Horizontal distance from (x, y) to the footprint (0 inside)."""
        if self.shape == "cylinder":
            return max(0.0, math.hypot(x - self.cx, y - self.cy) - self.radius)
        u, v = self._local(x, y)
        du = max(0.0, abs(u) - self.length / 2)
        dv = max(0.0, abs(v) - self.width / 2)
        return math.hypot(du, dv)

    def contains(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        if z > self.height + margin or z < -margin:
            return False
        return self.horizontal_gap(x, y) <= margin

    def distance(self, x: float, y: float, z: float) -> float:
        return math.hypot(self.horizontal_gap(x, y), max(0.0, z - self.height))

    def near_segment(self, a, b, margin: float) -> bool:
        """Cheap reject: can the 2-D segment a-b come within bound_r + margin of the centre?"""
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((self.cx - ax) * dx + (self.cy - ay) * dy) / L2))
        return math.hypot(ax + t * dx - self.cx, ay + t * dy - self.cy) <= self.bound_r + margin

    def sdf(self) -> str:
        r, g, b = self.color
        mat = f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
        if self.shape == "cylinder":
            geo = f"<cylinder><radius>{self.radius}</radius><length>{self.height}</length></cylinder>"
        else:
            geo = f"<box><size>{self.length} {self.width} {self.height}</size></box>"
        pose = f"{self.cx} {self.cy} {self.height / 2:.3f} {self.roll:.4f} {self.pitch:.4f} {self.yaw:.4f}"
        return (f'    <model name="map_{self.id}"><static>true</static><pose>{pose}</pose>\n'
                f'      <link name="link"><collision name="c"><geometry>{geo}</geometry></collision>\n'
                f'      <visual name="v"><geometry>{geo}</geometry>{mat}</visual></link></model>\n')


# --------------------------------------------------------------------------- canopy
@dataclass(frozen=True)
class Species:
    name: str
    radius_m: float
    height_m: float
    trunk_r_m: float = 0.35
    color: tuple = (0.18, 0.38, 0.16)


class Canopy:
    """A forest: many small tree cylinders, handled as a layer rather than as obstacles.

    Hundreds of 5-7 m trees are not obstacles for a UAV at survey height, but they do
    attenuate a radio path that dips into the canopy (a low UAV, or the slant to a ground
    antenna), and the referee still has to notice a UAV that flies into one. Keeping them
    out of `obstacles` keeps the A* planner and the per-link loss loop fast; everything
    that needs them asks the canopy directly, through a 20 m bucket grid.

    Foliage loss follows the usual vegetation model: a fixed dB per metre of path inside a
    crown, capped so a long grazing path cannot produce an absurd number.
    """

    CELL_M = 20.0

    def __init__(self, data: dict):
        self.top_m = float(data.get("top_m", 0.0))
        self.loss_db_per_m = float(data.get("loss_db_per_m", 0.4))
        self.max_loss_db = float(data.get("max_loss_db", 25.0))
        self.species: list[Species] = []
        for name, s in (data.get("species") or {}).items():
            self.species.append(Species(name=name, radius_m=float(s["radius_m"]),
                                        height_m=float(s["height_m"]),
                                        trunk_r_m=float(s.get("trunk_r_m", 0.35)),
                                        color=tuple(s.get("color", (0.18, 0.38, 0.16)))))
        self.trees: list[tuple[float, float, int]] = [
            (float(t[0]), float(t[1]), int(t[2]) if len(t) > 2 else 0) for t in (data.get("trees") or [])]
        self._grid: dict[tuple[int, int], list[int]] = {}
        self._max_r = max((s.radius_m for s in self.species), default=0.0)
        for i, (x, y, _) in enumerate(self.trees):
            self._grid.setdefault((int(x // self.CELL_M), int(y // self.CELL_M)), []).append(i)

    def __bool__(self) -> bool:
        return bool(self.trees)

    @property
    def count(self) -> int:
        return len(self.trees)

    def _near(self, x: float, y: float, reach: float):
        """Indices of trees whose cell is within `reach` of (x, y)."""
        r = int((reach + self._max_r) // self.CELL_M) + 1
        cx, cy = int(x // self.CELL_M), int(y // self.CELL_M)
        for i in range(cx - r, cx + r + 1):
            for j in range(cy - r, cy + r + 1):
                yield from self._grid.get((i, j), ())

    def tree(self, index: int) -> tuple[float, float, Species]:
        x, y, s = self.trees[index]
        return x, y, self.species[s % len(self.species)] if self.species else Species("tree", 2.0, 5.0)

    def hit(self, x: float, y: float, z: float, margin: float = 0.0):
        """The first tree whose crown contains (x, y, z), or None."""
        if z > self.top_m + margin:
            return None
        for i in self._near(x, y, margin + 1.0):
            tx, ty, sp = self.tree(i)
            if z <= sp.height_m + margin and math.hypot(x - tx, y - ty) <= sp.radius_m + margin:
                return i
        return None

    def clearance(self, x: float, y: float, z: float) -> float:
        """Distance to the nearest crown surface (0 inside one); inf with no tree nearby."""
        if not self.trees:
            return math.inf
        best = math.inf
        for i in self._near(x, y, 30.0):
            tx, ty, sp = self.tree(i)
            gap = max(0.0, math.hypot(x - tx, y - ty) - sp.radius_m)
            best = min(best, math.hypot(gap, max(0.0, z - sp.height_m)))
        return best

    def loss_db(self, a, b, step: float = 2.0) -> float:
        """Foliage loss of the 3-D path a-b: dB per metre spent inside a crown."""
        if not self.trees or (a[2] > self.top_m and b[2] > self.top_m):
            return 0.0
        d = math.dist(a, b)
        if d < 1e-6:
            return 0.0
        n = max(2, int(d / step))
        inside = 0
        for i in range(n + 1):
            t = i / n
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            z = a[2] + t * (b[2] - a[2])
            if z <= self.top_m and self.hit(x, y, z) is not None:
                inside += 1
        return min(self.max_loss_db, self.loss_db_per_m * inside * d / n)

    def sdf_models(self) -> str:
        """One static link holding every tree: far cheaper to load than a model each."""
        if not self.trees:
            return ""
        parts = ['    <model name="map_forest"><static>true</static><link name="link">\n']
        for i, (x, y, _) in enumerate(self.trees):
            tx, ty, sp = self.tree(i)
            r, g, b = sp.color
            crown_z = sp.height_m - sp.radius_m * 0.55
            parts.append(
                f'      <collision name="t{i}c"><pose>{tx} {ty} {sp.height_m / 2:.2f} 0 0 0</pose>'
                f'<geometry><cylinder><radius>{sp.radius_m}</radius><length>{sp.height_m}</length>'
                f'</cylinder></geometry></collision>\n'
                f'      <visual name="t{i}t"><pose>{tx} {ty} {crown_z / 2:.2f} 0 0 0</pose>'
                f'<geometry><cylinder><radius>{sp.trunk_r_m}</radius><length>{crown_z:.2f}</length>'
                f'</cylinder></geometry>'
                f'<material><ambient>0.29 0.21 0.13 1</ambient><diffuse>0.33 0.24 0.15 1</diffuse>'
                f'</material></visual>\n'
                f'      <visual name="t{i}c"><pose>{tx} {ty} {crown_z:.2f} 0 0 0</pose>'
                f'<geometry><sphere><radius>{sp.radius_m}</radius></sphere></geometry>'
                f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material></visual>\n')
        parts.append("    </link></model>\n")
        return "".join(parts)


class WorldMap:
    def __init__(self, data: dict, source: str = ""):
        self.name = data.get("name", os.path.splitext(os.path.basename(source))[0])
        self.description = data.get("description", "")
        self.source = source
        self.data = data
        self.cell = float(data.get("planner_cell_m", 4.0))
        self.obstacles: list[Obstacle] = []
        for i, o in enumerate(data.get("obstacles", [])):
            shape = o.get("shape", "box")
            size = o.get("size", [10, 10, 10])
            self.obstacles.append(Obstacle(
                id=str(o.get("id", f"O{i + 1}")), shape=shape, cx=float(o["center"][0]), cy=float(o["center"][1]),
                height=float(o.get("height", size[2] if shape == "box" else 10.0)),
                length=float(size[0]) if shape == "box" else 0.0, width=float(size[1]) if shape == "box" else 0.0,
                radius=float(o.get("radius", 0.0)), yaw=math.radians(float(o.get("yaw_deg", 0.0))),
                roll=math.radians(float(o.get("roll_deg", 0.0))), pitch=math.radians(float(o.get("pitch_deg", 0.0))),
                kind=o.get("kind", "building"), attenuation_db=float(o.get("attenuation_db", 20.0)),
                color=tuple(o.get("color", (0.55, 0.53, 0.5)))))
        self.canopy = Canopy(data.get("canopy") or {})
        area = data.get("mission_area") or {}
        self.mission_area = (float(area["x"][0]), float(area["x"][1]),
                             float(area["y"][0]), float(area["y"][1])) if area else None
        fence = data.get("geofence") or {}
        if fence:
            self.geofence = (float(fence["x"][0]), float(fence["x"][1]),
                             float(fence["y"][0]), float(fence["y"][1]))
            self.fence_alt_m = float(fence.get("z_max", 100.0))
        else:
            self.geofence = GEOFENCE
            self.fence_alt_m = 100.0
        gcs = data.get("gcs") or {}
        self.gcs_antenna_m = [float(v) for v in gcs["antenna_m"]] if gcs.get("antenna_m") else None
        self.pads: list[list[float]] = [[float(p[0]), float(p[1])] for p in (gcs.get("pads") or [])]

    # ------------------------------------------------------------------ loading
    @classmethod
    def load(cls, name_or_path: str) -> "WorldMap":
        path = name_or_path
        if not os.path.isfile(path):
            name = name_or_path if name_or_path.endswith(".yaml") else f"{name_or_path}.yaml"
            path = os.path.join(_gazebo_share(), "maps", name)
        with open(path, encoding="utf-8") as fh:
            return cls(yaml.safe_load(fh) or {}, path)

    @staticmethod
    def for_scenario(scenario: dict):
        return WorldMap.load(scenario["map"]) if scenario.get("map") else None

    # ------------------------------------------------------------------ queries
    def blocked(self, x: float, y: float, z: float, margin: float = 0.0, obstacles=None) -> bool:
        if any(o.contains(x, y, z, margin) for o in (obstacles if obstacles is not None else self.obstacles)):
            return True
        return obstacles is None and self.canopy.hit(x, y, z, margin) is not None

    def clearance(self, x: float, y: float, z: float) -> float:
        return min(min((o.distance(x, y, z) for o in self.obstacles), default=math.inf),
                   self.canopy.clearance(x, y, z))

    def inside_area(self, x: float, y: float, margin: float = 0.0) -> bool:
        """Inside the mission (survey) area? True everywhere when the map defines none."""
        if self.mission_area is None:
            return True
        x0, x1, y0, y1 = self.mission_area
        return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin

    def inside_fence(self, x: float, y: float, z: float = 0.0) -> bool:
        x0, x1, y0, y1 = self.geofence
        return x0 <= x <= x1 and y0 <= y <= y1 and z <= self.fence_alt_m

    def los_loss_db(self, a, b, step: float = 2.0) -> float:
        """Extra path loss (dB) of the 3-D radio path a-b: every obstacle it crosses, plus foliage."""
        total = self.canopy.loss_db(a, b, step)
        d = math.dist(a, b)
        n = max(2, int(d / step))
        for o in self.obstacles:
            if not o.near_segment(a, b, 0.0) or (a[2] > o.height and b[2] > o.height):
                continue
            for i in range(1, n):
                t = i / n
                if o.contains(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])):
                    total += o.attenuation_db
                    break
        return total

    def _relevant(self, alt: float, clearance: float):
        return [o for o in self.obstacles if o.height + clearance >= alt]

    def _canopy_relevant(self, alt: float, clearance: float) -> bool:
        return bool(self.canopy) and self.canopy.top_m + clearance >= alt

    def segment_clear(self, a, b, alt: float, clearance: float, obstacles=None) -> bool:
        obs = [o for o in (obstacles if obstacles is not None else self._relevant(alt, clearance))
               if o.near_segment(a, b, clearance)]
        canopy = self._canopy_relevant(alt, clearance)
        if not obs and not canopy:
            return True
        d = math.dist(a, b)
        n = max(2, int(d / 1.0))
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            if any(o.horizontal_gap(x, y) <= clearance for o in obs):
                return False
            if canopy and self.canopy.hit(x, y, alt, clearance) is not None:
                return False
        return True

    def free_at(self, x: float, y: float, alt: float, clearance: float, obstacles=None) -> bool:
        """Is a UAV at (x, y) and height alt clear of everything by `clearance`?"""
        obs = obstacles if obstacles is not None else self._relevant(alt, clearance)
        if any(o.horizontal_gap(x, y) <= clearance and o.height + clearance >= alt for o in obs):
            return False
        return not (self._canopy_relevant(alt, clearance) and self.canopy.hit(x, y, alt, clearance) is not None)

    def plan(self, start, goal, alt: float, clearance: float = 10.0) -> list[tuple[float, float]]:
        """Obstacle-free waypoints from start to goal at height alt (start excluded).

        The last waypoint is the goal, or the nearest free point if the goal itself is too
        close to an obstacle. Falls back to the straight line if no path exists.
        """
        start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
        obs = self._relevant(alt, clearance)
        if not obs and not self._canopy_relevant(alt, clearance):
            return [goal]
        if self.segment_clear(start, goal, alt, clearance, obs):
            return [goal]
        c = self.cell
        fx0, fx1, fy0, fy1 = self.geofence
        x0 = max(fx0, min(start[0], goal[0]) - 120.0)
        x1 = min(fx1, max(start[0], goal[0]) + 120.0)
        y0 = max(fy0, min(start[1], goal[1]) - 120.0)
        y1 = min(fy1, max(start[1], goal[1]) + 120.0)
        nx, ny = int((x1 - x0) / c) + 1, int((y1 - y0) / c) + 1

        def centre(i, j):
            return (x0 + i * c, y0 + j * c)

        def cell_of(p):
            return (min(nx - 1, max(0, int(round((p[0] - x0) / c)))), min(ny - 1, max(0, int(round((p[1] - y0) / c)))))

        free_cache: dict = {}

        def free(i, j):
            k = (i, j)
            if k not in free_cache:
                x, y = centre(i, j)
                free_cache[k] = self.free_at(x, y, alt, clearance, obs)
            return free_cache[k]

        s, g = cell_of(start), cell_of(goal)
        if not free(*g):          # goal too close to an obstacle: nearest free cell
            best, best_d = None, math.inf
            for r in range(1, 12):
                for i in range(g[0] - r, g[0] + r + 1):
                    for j in range(g[1] - r, g[1] + r + 1):
                        if 0 <= i < nx and 0 <= j < ny and free(i, j):
                            d = math.dist(centre(i, j), goal)
                            if d < best_d:
                                best, best_d = (i, j), d
                if best:
                    break
            if best is None:
                return [goal]
            g = best
            goal = centre(*g)
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        openq = [(0.0, s)]
        g_cost = {s: 0.0}
        parent = {}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for di, dj, w in moves:
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < nx and 0 <= nb[1] < ny) or not (free(*nb) or nb == s):
                    continue
                cost = g_cost[cur] + w * c
                if cost < g_cost.get(nb, math.inf):
                    g_cost[nb] = cost
                    parent[nb] = cur
                    heapq.heappush(openq, (cost + math.dist(centre(*nb), centre(*g)), nb))
        if g not in parent and g != s:
            return [goal]
        cells = [g]
        while cells[-1] != s:
            cells.append(parent[cells[-1]])
        pts = [start] + [centre(*k) for k in reversed(cells[:-1])]
        pts[-1] = goal
        # line-of-sight shortcutting
        out, i = [], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.segment_clear(pts[i], pts[j], alt, clearance, obs):
                j -= 1
            out.append(pts[j])
            i = j
        if not out:
            out = [goal]
        return [(round(x, 2), round(y, 2)) for x, y in out]

    def random_free_point(self, rng, region, margin: float = 20.0, avoid=(), avoid_r: float = 30.0, tries: int = 400):
        x0, y0, x1, y1 = region
        for _ in range(tries):
            p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
            if any(o.horizontal_gap(*p) <= margin for o in self.obstacles):
                continue
            if all(math.dist(p, q[:2]) > avoid_r for q in avoid):
                return [round(p[0], 1), round(p[1], 1)]
        return None

    def ground_free_point(self, rng, region, min_tree_m: float = 6.0, avoid=(), avoid_r: float = 15.0,
                          tries: int = 2000):
        """A spot on the ground clear of trunks and of other targets - how the organisers
        place their search-and-rescue sites (prop centre at least `min_tree_m` from a trunk,
        targets at least `avoid_r` apart)."""
        x0, y0, x1, y1 = region
        for _ in range(tries):
            p = (round(rng.uniform(x0, x1), 1), round(rng.uniform(y0, y1), 1))
            if any(o.horizontal_gap(*p) <= min_tree_m for o in self.obstacles):
                continue
            if any(math.dist(p, q[:2]) < avoid_r for q in avoid):
                continue
            if self.canopy.trees:
                near = min((math.hypot(p[0] - self.canopy.tree(i)[0], p[1] - self.canopy.tree(i)[1])
                            for i in self.canopy._near(p[0], p[1], min_tree_m + 2.0)), default=math.inf)
                if near <= min_tree_m:
                    continue
            return [p[0], p[1]]
        return None

    # ------------------------------------------------------------------ Gazebo
    def sdf_models(self) -> str:
        return "".join(o.sdf() for o in self.obstacles) + self.canopy.sdf_models()

    def write_world(self, base_world_path: str, out_path: str) -> str:
        """Base world (ground, GCS, pads, plugins) + this map's obstacles -> out_path."""
        import re
        with open(base_world_path, encoding="utf-8") as fh:
            text = fh.read()
        # The organiser world already contains the geometry described by its planner map.
        if os.path.basename(base_world_path) != self.data.get("scenery_world"):
            text = re.sub(r'<model name="rubble">.*?</model>\s*', "", text, flags=re.S)
            text = text.replace("</world>", f"    <!-- 3-D map: {self.name} ({len(self.obstacles)} obstacles, "
                                f"{self.canopy.count} trees) -->\n" + self.sdf_models() + "  </world>")
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(text)
        return out_path
