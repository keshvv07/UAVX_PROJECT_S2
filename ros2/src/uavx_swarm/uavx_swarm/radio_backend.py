"""Where the channel numbers come from - and the seam an external simulator plugs into.

`radio_net` never computes a link itself: it asks a backend for the instantaneous quality
of one link and keeps the bookkeeping (smoothing, up/down hysteresis, per-frame draws,
delivery, logging) to itself. That split is deliberate. The organisers allow an external
communication simulator such as ns-3 as an extra layer (Q26) and say they will vary
communication parameters themselves during evaluation (Q25, Q27), so the channel has to be
replaceable without touching the swarm.

    InternalBackend   the model this stack is validated against: log-distance path loss,
                      logistic PDR in SNR, obstacle and canopy attenuation from the map,
                      Rayleigh-ish fading, and the organisers' 100 m hard range. Shares its
                      equations and its numbers with the Stage 1 Python simulator, so a
                      result here and a result there rest on one set of assumptions.

    Ns3Backend        the seam. ns-3 is not a dependency of this stack and is not in the
                      submitted image; this class documents exactly what an ns-3 (or
                      OMNeT++, or measured-trace) backend has to provide and fails with a
                      clear message if selected without one, rather than silently falling
                      back to the internal model and reporting numbers that did not come
                      from where the run claims they did.

Select one from a scenario:

    radio_backend: internal          # the default
    radio_backend: ns3               # needs UAVX_NS3_ENDPOINT and a running ns-3 bridge

A backend sees node ids and ground-truth positions, which is all an external simulator
needs to build its own topology every update interval.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

from .radio_model import RadioParams, latency_ms, mean_snr_db, pdr_from_snr


@dataclass
class LinkQuality:
    """One link as the channel sees it, before radio_net's smoothing and hysteresis."""
    pdr: float                 # instantaneous delivery ratio of the channel itself
    snr_db: float
    latency_ms: float
    distance_m: float
    extra_loss_db: float = 0.0
    label: str = ""            # why it is degraded: "obstructed", "canopy", "jamming", ...


class ChannelBackend:
    name = "abstract"

    def __init__(self, params: RadioParams, world_map=None, rng=None):
        self.p = params
        self.map = world_map
        self.rng = rng

    def evaluate(self, a: str, b: str, pos_a, pos_b, gcs_link: bool,
                 extra_loss_db: float = 0.0, t: float = 0.0) -> LinkQuality:
        raise NotImplementedError

    def info(self) -> dict:
        return {"backend": self.name, "max_range_m": self.p.max_range_m,
                "frame_bits": self.p.frame_bits}


class InternalBackend(ChannelBackend):
    """The stack's own channel model (see radio_model.py)."""

    name = "internal"

    def evaluate(self, a: str, b: str, pos_a, pos_b, gcs_link: bool,
                 extra_loss_db: float = 0.0, t: float = 0.0) -> LinkQuality:
        label = ""
        extra = float(extra_loss_db)
        if self.map is not None:
            blocked = self.map.los_loss_db(pos_a, pos_b)
            if blocked > 0.0:
                extra += blocked
                label = "obstructed"
        snr, d = mean_snr_db(self.p, pos_a, pos_b, gcs_link, extra)
        if self.p.fading_sigma_db > 0 and self.rng is not None:
            snr += self.rng.gauss(0.0, self.p.fading_sigma_db)
        pdr = 0.0 if d > self.p.max_range_m else pdr_from_snr(self.p, snr)
        return LinkQuality(pdr=pdr, snr_db=snr, latency_ms=latency_ms(self.p, max(pdr, 0.05)),
                           distance_m=d, extra_loss_db=extra, label=label)


class Ns3Backend(ChannelBackend):
    """Seam for an external packet-level simulator. Not wired up, and deliberately so.

    What an implementation has to do, in this order:

      1. Build the topology once per `update_interval_s`: push every node's ground-truth
         position (this stack already has it, from Gazebo) into the external simulator's
         mobility model. Node ids here are strings ("gcs", "uav1"); map them to the
         simulator's own node container once, at construction.
      2. Ask the simulator for the per-link quality it predicts - PDR (or frame error
         rate), SNR and delay - and return it as a LinkQuality. radio_net keeps doing the
         per-frame draw, the ARQ and the logging, so nothing else in the stack changes.
      3. Keep the organisers' hard constraint: no link at all beyond `max_range_m` (Q13).
         An external model with a longer tail would quietly break the constraint the run is
         scored against.

    The practical route is a small ns-3 program with the WiFi or LrWpan model and a socket
    or shared-memory bridge, started beside the other nodes and addressed through
    UAVX_NS3_ENDPOINT. Keeping it out of the submitted image is a choice about what has to
    be reproducible on the evaluators' hardware; the seam is here so that choice can be
    revisited without rewriting the radio.
    """

    name = "ns3"

    def __init__(self, params: RadioParams, world_map=None, rng=None, endpoint: str | None = None):
        super().__init__(params, world_map, rng)
        self.endpoint = endpoint or os.environ.get("UAVX_NS3_ENDPOINT", "")
        raise RuntimeError(
            "radio_backend 'ns3' is a documented seam, not a working backend: this stack ships "
            "the validated internal channel model instead (see radio_backend.py for what an ns-3 "
            "implementation has to provide). Use radio_backend: internal, or implement "
            "Ns3Backend.evaluate against a running ns-3 bridge.")

    def evaluate(self, a: str, b: str, pos_a, pos_b, gcs_link: bool,
                 extra_loss_db: float = 0.0, t: float = 0.0) -> LinkQuality:   # pragma: no cover
        raise NotImplementedError


BACKENDS = {"internal": InternalBackend, "ns3": Ns3Backend}


def make_backend(name: str, params: RadioParams, world_map=None, rng=None) -> ChannelBackend:
    key = (name or "internal").strip().lower()
    if key not in BACKENDS:
        raise ValueError(f"unknown radio_backend {name!r}; available: {sorted(BACKENDS)}")
    return BACKENDS[key](params, world_map, rng)


def hops_to_pdr(link_pdrs) -> float:
    """End-to-end delivery ratio of a multi-hop route: the product of its hops."""
    out = 1.0
    for pdr in link_pdrs:
        out *= max(0.0, min(1.0, pdr))
    return out


def etx_of_route(link_pdrs) -> float:
    """Expected transmissions for one frame over a route (infinite if a hop is dead)."""
    total = 0.0
    for pdr in link_pdrs:
        if pdr <= 0.0:
            return math.inf
        total += 1.0 / pdr
    return total
