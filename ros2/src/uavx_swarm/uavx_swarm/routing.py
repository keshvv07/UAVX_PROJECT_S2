"""Decentralised multi-hop routing to the GCS - ETX distance-vector.

Each node runs its own copy of this: nobody is told the topology, and a UAV only ever
learns what its neighbours tell it over the emulated radio. That matters for the
organisers' Q25-Q30: the route has to survive links that fade, get jammed or go away with
a UAV, and it has to do so from local information.

How it works

  Neighbour quality   every frame carries its sender's per-sender sequence number, so a
                      receiver measures the delivery ratio of each neighbour over a sliding
                      `window_s` from the gaps in that sequence. ETX of a hop is 1 / ratio -
                      the expected number of transmissions to get one frame through.

  Cost to the GCS     the GCS advertises cost 0 in its heartbeat. Every node advertises
                      `cost = link ETX to its next hop + that neighbour's cost` in its 1 Hz
                      status broadcast, and picks the neighbour that minimises it.

  Weak links          a hop below `min_link_pdr` carries `weak_link_penalty` extra
                      transmissions, so the router prefers a longer chain of good links and
                      only uses a marginal one when nothing else reaches the GCS. (Ported
                      from the Stage 1 simulator's swarm/route_manager.py, same numbers.)

  Stability           the current next hop is kept while it is within `hysteresis` of the
                      best, which stops two near-equal routes flapping; a kept next hop
                      must still advertise a strictly lower cost than this node's own, so
                      routing loops cannot form.

  Staleness           an advertisement or a neighbour measurement older than `stale_s` is
                      dropped, so a UAV that goes quiet (failed, out of range, radio out)
                      stops being used as a hop within a second or two.

A node with no usable neighbour has `connected` False and route cost infinity: that is the
signal the companion uses to buffer reports and, if nothing improves, to fly back into
contact.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

GCS_COST = 0.0


@dataclass(frozen=True)
class RouteParams:
    window_s: float = 5.0             # sliding window for neighbour delivery ratio
    stale_s: float = 3.0              # drop a neighbour not heard from for this long
    hysteresis: float = 0.15          # keep the current next hop while within 15 % of the best
    min_link_pdr: float = 0.7         # links below this are a last resort
    weak_link_penalty: float = 10.0   # extra transmissions charged for using a weak link
    min_usable_pdr: float = 0.15      # below this a neighbour is not a hop at all
    max_hops: int = 6

    def __post_init__(self) -> None:
        if not 0.0 <= self.hysteresis < 1.0 or self.weak_link_penalty < 0:
            raise ValueError("invalid routing parameters")
        if not 0.0 <= self.min_link_pdr < 1.0 or not 0.0 < self.min_usable_pdr < 1.0:
            raise ValueError("invalid routing parameters")


def link_cost(pdr: float, params: RouteParams) -> float:
    """ETX of one hop, plus a penalty when the link is weak."""
    cost = 1.0 / max(pdr, 1e-3)
    return cost + params.weak_link_penalty if pdr < params.min_link_pdr else cost


@dataclass
class Neighbour:
    id: str
    pdr: float = 0.0
    cost: float = math.inf        # the neighbour's own advertised cost to the GCS
    next_hop: str = ""            # whom it routes through (loop detection / display)
    hops: int = 0                 # its own advertised hop count to the GCS
    last_t: float = -math.inf     # last frame received from it
    last_ad_t: float = -math.inf  # last cost advertisement from it

    def fresh(self, t: float, stale_s: float) -> bool:
        return (t - self.last_t) <= stale_s and (t - self.last_ad_t) <= stale_s


class RouteTable:
    """One node's view of how to reach the GCS."""

    def __init__(self, node_id: str, gcs_id: str = "gcs", params: RouteParams | None = None):
        self.id = node_id
        self.gcs_id = gcs_id
        self.p = params or RouteParams()
        self.neighbours: dict[str, Neighbour] = {}
        self._seen: dict[str, deque] = {}        # src -> deque of (t, seq)
        self.next_hop: str = ""
        self.cost: float = math.inf
        self.hops: int = 0
        self.changes = 0

    # ------------------------------------------------------------------ inputs
    def on_frame(self, src: str, seq: int, t: float) -> None:
        """A frame arrived from `src`: update that neighbour's measured delivery ratio."""
        if src == self.id:
            return
        q = self._seen.setdefault(src, deque())
        q.append((t, int(seq)))
        while q and t - q[0][0] > self.p.window_s:
            q.popleft()
        n = self.neighbours.setdefault(src, Neighbour(src))
        n.last_t = t
        n.pdr = self._ratio(q)

    @staticmethod
    def _ratio(q) -> float:
        """Delivered / expected over the window, from the gaps in the sender's sequence."""
        if len(q) < 2:
            return 0.5 if q else 0.0      # one frame: assume nothing either way
        seqs = [s for _, s in q]
        expected = max(seqs) - min(seqs) + 1
        return min(1.0, len(q) / max(expected, 1))

    def on_advert(self, src: str, cost: float, next_hop: str = "", hops: int = 0, t: float = 0.0) -> None:
        """`src` says it can reach the GCS at `cost` (the GCS itself advertises 0)."""
        if src == self.id:
            return
        n = self.neighbours.setdefault(src, Neighbour(src))
        n.cost = GCS_COST if src == self.gcs_id else float(cost)
        n.next_hop = next_hop or ""
        n.hops = 0 if src == self.gcs_id else int(hops)
        n.last_ad_t = t
        if n.last_t == -math.inf:
            n.last_t = t

    def drop(self, node_id: str) -> None:
        self.neighbours.pop(node_id, None)
        self._seen.pop(node_id, None)
        if self.next_hop == node_id:
            self.next_hop, self.cost, self.hops = "", math.inf, 0

    # ------------------------------------------------------------------ decision
    def update(self, t: float) -> bool:
        """Recompute the next hop. Returns True when it changed."""
        best, best_cost, best_hops = "", math.inf, 0
        for n in self.neighbours.values():
            if not n.fresh(t, self.p.stale_s) or n.pdr < self.p.min_usable_pdr:
                continue
            if n.cost is math.inf or n.next_hop == self.id:       # it routes through us: a loop
                continue
            if n.id != self.gcs_id and n.hops + 1 > self.p.max_hops:
                continue
            total = n.cost + link_cost(n.pdr, self.p)
            if total < best_cost:
                best, best_cost, best_hops = n.id, total, (0 if n.id == self.gcs_id else n.hops) + 1

        cur = self.neighbours.get(self.next_hop)
        if best and cur is not None and self.next_hop != best and cur.fresh(t, self.p.stale_s) \
                and cur.pdr >= self.p.min_usable_pdr and cur.cost is not math.inf \
                and cur.next_hop != self.id and cur.cost < best_cost:
            keep = cur.cost + link_cost(cur.pdr, self.p)
            if keep <= best_cost * (1.0 + self.p.hysteresis):
                best, best_cost, best_hops = cur.id, keep, (0 if cur.id == self.gcs_id else cur.hops) + 1

        changed = best != self.next_hop
        self.next_hop, self.cost, self.hops = best, best_cost, best_hops
        if changed:
            self.changes += 1
        return changed

    # ------------------------------------------------------------------ queries
    @property
    def connected(self) -> bool:
        return bool(self.next_hop) and self.cost < math.inf

    @property
    def direct_to_gcs(self) -> bool:
        return self.next_hop == self.gcs_id

    def best_relay(self, t: float) -> tuple[str, float]:
        """The neighbour (not the GCS) with the lowest cost - whom to ferry data to."""
        best, cost = "", math.inf
        for n in self.neighbours.values():
            if n.id == self.gcs_id or not n.fresh(t, self.p.stale_s) or n.cost is math.inf:
                continue
            if n.cost < cost:
                best, cost = n.id, n.cost
        return best, cost

    def advert(self) -> dict:
        """What this node puts in its status broadcast."""
        return {"cost": None if self.cost is math.inf else round(self.cost, 3),
                "next_hop": self.next_hop, "hops": self.hops}

    def to_dict(self, t: float = 0.0) -> dict:
        return {"id": self.id, "next_hop": self.next_hop, "hops": self.hops,
                "cost": None if self.cost is math.inf else round(self.cost, 3),
                "connected": self.connected,
                "neighbours": {n.id: {"pdr": round(n.pdr, 3),
                                      "cost": None if n.cost is math.inf else round(n.cost, 3),
                                      "fresh": n.fresh(t, self.p.stale_s)}
                               for n in sorted(self.neighbours.values(), key=lambda x: x.id)}}
