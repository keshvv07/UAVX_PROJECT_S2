"""Where to put relays, and when to ferry instead - the geometry behind the GCS's decision.

The organisers fix a 100 m radio range (Q13) and a 500 m x 500 m search area 365 m from
the ground station (Q6). Those two numbers decide the whole communication strategy, and
this module is where that arithmetic lives:

    GCS mast -> near tree line      112 m    1 relay for a live link
    GCS mast -> centre of the area  360 m    4 relays
    GCS mast -> far corner          664 m    7 relays

With a 4-5 UAV fleet (and a fleet that large is a design choice, not a requirement -
Q10/Q44 discourage throwing UAVs at the problem) a live chain to the near half of the area
is affordable and a live chain to the far corner is not. So the policy is a *partial*
ferry, which is what `decide` returns:

  LIVE    the chain to the PoI fits in the relays the fleet can spare. Relays take their
          anchor points before the surveyor leaves coverage, and the report is delivered
          within the organisers' 10 s (Q23).

  FERRY   it does not fit. The relays still go as far out as the fleet allows - that is
          the "partial" part - the surveyor flies on alone and buffers its reports, and
          when it is done it flies to the `handoff` point: the nearest point to it that is
          inside the chain tip's radio range. From there the report multi-hops home. The
          surveyor gives up a short round trip, not its place in the sweep.

  BLIND   not even one relay is available: classic store-and-forward, the report arrives
          when the surveyor is back in the GCS's own range.

Anchor spacing comes from the channel model rather than from a guess: a hop is
`range_for_pdr(min_planned_pdr) * spacing_safety` long, so every planned hop predicts at
least `min_planned_pdr`. The first hop is shortened for the climb from the ground antenna
to the relay height, because that slant eats into the same budget. Relays fly
`relay_alt_m` above ground, one flight level clear of the surveyors, so a relay holding
station can never be the reason a separation minimum is broken.

Ported from the Stage 1 simulator's swarm/relay_selector.py: the straight-chain spacing,
the dog-leg detour when a straight chain is blocked, and the cost function that picks who
flies each anchor (travel time, battery, and stickiness so a relay already on station is
not swapped out for a marginal gain).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .radio_model import RadioParams, predict_pdr, range_for_pdr

LIVE, FERRY, BLIND = "LIVE", "FERRY", "BLIND"


@dataclass(frozen=True)
class RelayParams:
    relay_alt_m: float = 55.0             # one flight level above the surveyors
    min_planned_pdr: float = 0.85         # every planned hop must predict at least this
    spacing_safety: float = 0.9           # hop length = range_for_pdr(...) * this
    reposition_threshold_m: float = 25.0  # closer than this counts as "already there"
    min_battery_pct: float = 35.0         # a UAV below this is not asked to relay
    stickiness_s: float = 30.0            # bonus for keeping the relay already on station
    handoff_margin: float = 0.85          # rendezvous at this fraction of the hop range
    max_chain: int = 8                    # refuse to plan an absurd chain

    def __post_init__(self) -> None:
        if not 0.5 <= self.min_planned_pdr < 1.0 or not 0.0 < self.spacing_safety <= 1.0:
            raise ValueError("invalid relay parameters")
        if not 0.0 < self.handoff_margin <= 1.0 or self.max_chain < 1:
            raise ValueError("invalid relay parameters")


@dataclass
class Chain:
    """A planned relay chain from the GCS towards one target."""
    target_id: str
    target: tuple                     # (x, y, z) of the surveyor's work
    points: list[tuple]              # relay anchor points, GCS end first
    mode: str                        # LIVE | FERRY | BLIND
    needed: int                      # relays a full live chain would take
    budget: int                      # relays the fleet could spare
    tip: tuple | None = None         # outermost anchor actually planned
    handoff: tuple | None = None     # where a ferrying surveyor delivers (None if LIVE)

    @property
    def relays(self) -> int:
        return len(self.points)

    def to_dict(self) -> dict:
        def r(p):
            return None if p is None else [round(float(v), 1) for v in p]
        return {"target": self.target_id, "mode": self.mode, "needed": self.needed,
                "budget": self.budget, "points": [r(p) for p in self.points],
                "tip": r(self.tip), "handoff": r(self.handoff)}


class RelayPlanner:
    def __init__(self, radio: RadioParams, params: RelayParams | None = None, world_map=None):
        self.radio = radio
        self.p = params or RelayParams()
        self.map = world_map
        reach = range_for_pdr(radio, self.p.min_planned_pdr)
        gcs_reach = range_for_pdr(radio, self.p.min_planned_pdr, gcs_link=True)
        self.hop_m = reach * self.p.spacing_safety
        self.gcs_hop_m = gcs_reach * self.p.spacing_safety

    def set_gcs(self, gcs_pos) -> None:
        """Shorten the first hop by the climb from the ground antenna to relay height."""
        climb = abs(self.p.relay_alt_m - float(gcs_pos[2]))
        gcs_reach = range_for_pdr(self.radio, self.p.min_planned_pdr, gcs_link=True)
        flat = math.sqrt(max(gcs_reach ** 2 - climb ** 2, 1.0))
        self.gcs_hop_m = flat * self.p.spacing_safety

    # ------------------------------------------------------------------ geometry
    def _straight(self, a, b) -> list[tuple]:
        """Anchors spaced evenly along a -> b so no hop is longer than the planning spacing."""
        d = math.dist(a[:2], b[:2])
        first = self.gcs_hop_m
        if d <= first:
            return []
        n = math.ceil((d - first) / self.hop_m - 1e-9)
        if n > self.p.max_chain:
            n = self.p.max_chain
        scale = d / (first + n * self.hop_m)
        ux, uy = (b[0] - a[0]) / d, (b[1] - a[1]) / d
        alt = self.p.relay_alt_m
        return [(round(a[0] + ux * scale * (first + i * self.hop_m), 1),
                 round(a[1] + uy * scale * (first + i * self.hop_m), 1), alt) for i in range(n)]

    def _hop_ok(self, a, b, gcs_link: bool) -> bool:
        return predict_pdr(self.radio, a, b, gcs_link=gcs_link, world_map=self.map) >= self.p.min_planned_pdr

    def _point_ok(self, p) -> bool:
        if self.map is None:
            return True
        return self.map.inside_fence(p[0], p[1], p[2]) and self.map.free_at(p[0], p[1], p[2], 8.0)

    def _validate(self, gcs, points, target) -> bool:
        chain = [tuple(gcs), *points, tuple(target)]
        if not all(self._point_ok(p) for p in points):
            return False
        return all(self._hop_ok(chain[i], chain[i + 1], i == 0) for i in range(len(chain) - 1))

    def chain_points(self, gcs, target) -> list[tuple]:
        """Relay anchors for a live link GCS -> target; a dog-leg if the straight line fails."""
        points = self._straight(gcs, target)
        if self._validate(gcs, points, target):
            return points
        d = math.dist(gcs[:2], target[:2])
        if d < 1.0:
            return []
        base = math.atan2(target[1] - gcs[1], target[0] - gcs[0])
        best = None
        for deg in (20, 35, 50):
            for sign in (1, -1):
                ang = math.radians(deg)
                leg = (d / 2) / math.cos(ang)
                w = (round(gcs[0] + leg * math.cos(base + sign * ang), 1),
                     round(gcs[1] + leg * math.sin(base + sign * ang), 1), self.p.relay_alt_m)
                path = self._straight(gcs, w) + [w] + self._straight(w, target)
                if self._validate(gcs, path, target) and (best is None or len(path) < len(best)):
                    best = path
        return best if best is not None else points

    def relays_needed(self, gcs, target) -> int:
        return len(self.chain_points(gcs, target))

    def handoff_point(self, from_pos, node_pos) -> tuple:
        """The nearest point to `from_pos` that is inside `node_pos`'s radio range.

        This is what "come back into communication range of the nearest UAV" means
        numerically: stay on the line towards that node and stop as soon as the hop is
        short enough to carry the data, at the ferrying UAV's own flight level.
        """
        reach = range_for_pdr(self.radio, self.p.min_planned_pdr) * self.p.handoff_margin
        alt = float(from_pos[2]) if len(from_pos) > 2 else self.p.relay_alt_m
        dz = abs(alt - float(node_pos[2]))
        flat = math.sqrt(max(reach ** 2 - dz ** 2, 1.0))
        d = math.dist(from_pos[:2], node_pos[:2])
        if d <= flat:
            return (round(float(from_pos[0]), 1), round(float(from_pos[1]), 1), round(alt, 1))
        t = (d - flat) / d
        return (round(from_pos[0] + t * (node_pos[0] - from_pos[0]), 1),
                round(from_pos[1] + t * (node_pos[1] - from_pos[1]), 1), round(alt, 1))

    def safe_handoff_point(self, from_pos, node_pos, gcs_link: bool = False,
                           clearance_m: float = 8.0, step_m: float = 4.0) -> tuple:
        """`handoff_point`, checked against the map: free airspace AND a predicted link.

        Walking from `from_pos` towards `node_pos`, the first point that is clear of every
        obstacle at the flying UAV's height and whose hop to the node predicts at least
        `min_planned_pdr` with the map's shadowing. A point inside a building, or behind one,
        is not a place a report can be handed over from. Ends at the node itself.
        """
        alt = float(from_pos[2]) if len(from_pos) > 2 else self.p.relay_alt_m
        start = self.handoff_point(from_pos, node_pos)
        d = math.dist(start[:2], node_pos[:2])
        n = max(1, int(d / step_m))
        for i in range(n + 1):
            f = i / n
            p = (round(start[0] + f * (node_pos[0] - start[0]), 1),
                 round(start[1] + f * (node_pos[1] - start[1]), 1), round(alt, 1))
            if self.map is not None and not (self.map.inside_fence(p[0], p[1], p[2])
                                             and self.map.free_at(p[0], p[1], p[2], clearance_m)):
                continue
            if predict_pdr(self.radio, p, node_pos, gcs_link=gcs_link, world_map=self.map) >= self.p.min_planned_pdr:
                return p
        return (round(float(node_pos[0]), 1), round(float(node_pos[1]), 1), round(alt, 1))

    # ------------------------------------------------------------------ policy
    def decide(self, target_id: str, gcs, target, budget: int) -> Chain:
        """Live chain, partial ferry or blind store-and-forward for one target."""
        full = self.chain_points(gcs, target)
        needed = len(full)
        budget = max(0, int(budget))
        if needed <= budget:
            return Chain(target_id, tuple(target), full, LIVE, needed, budget,
                         tip=full[-1] if full else tuple(gcs))
        points = full[:budget]
        tip = points[-1] if points else tuple(gcs)
        mode = FERRY if points else BLIND
        return Chain(target_id, tuple(target), points, mode, needed, budget, tip=tip,
                     handoff=self.handoff_point(target, tip))

    def assign(self, chain: Chain, candidates: list[dict], speed_mps: float = 10.0,
               keep_low_battery: bool = False) -> dict[str, tuple]:
        """Who flies each anchor of `chain`: {uav_id: point}.

        The anchors farthest from the GCS choose first, because they cost the most battery
        to reach and hold. A UAV already relaying within `reposition_threshold_m` of a point
        keeps it; otherwise the cheapest by travel time and battery wins. A chain is only
        filled if every one of its anchors can be - a half-built chain connects nothing.

        `candidates` are dicts with id, pos, batt and relaying_at (its current anchor, or
        None), which is all the GCS knows about a UAV from its status broadcasts.

        `keep_low_battery` lets a relay already on one of the anchors keep it below
        `min_battery_pct` (the threshold is for taking a NEW anchor): the companion goes home
        on its own when its battery says so, and swapping it out early only moves the chain.
        """
        def eligible(c) -> bool:
            if float(c.get("batt", 100.0)) >= self.p.min_battery_pct:
                return True
            at = c.get("relaying_at")
            return keep_low_battery and at is not None and any(
                math.dist(at[:2], p[:2]) <= self.p.reposition_threshold_m for p in chain.points)

        free = {c["id"]: c for c in candidates if eligible(c)}
        if len(free) < len(chain.points):
            return {}
        out: dict[str, tuple] = {}
        for point in sorted(chain.points, key=lambda p: -math.dist(p[:2], chain.target[:2])):
            best, best_cost = None, math.inf
            for c in free.values():
                if c["id"] in out:
                    continue
                cost = math.dist(c["pos"][:2], point[:2]) / max(speed_mps, 0.1) \
                    + (100.0 - float(c.get("batt", 100.0))) * 0.5
                at = c.get("relaying_at")
                if at is not None and math.dist(at[:2], point[:2]) <= self.p.reposition_threshold_m:
                    cost -= self.p.stickiness_s
                if cost < best_cost:
                    best, best_cost = c["id"], cost
            if best is None:
                return {}
            out[best] = point
        return out
