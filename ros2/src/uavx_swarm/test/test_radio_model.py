"""The channel model and the organisers' bit error rate control (Q13, Q33)."""
import math

import pytest

from uavx_swarm.radio_backend import InternalBackend, Ns3Backend, etx_of_route, hops_to_pdr, make_backend
from uavx_swarm.radio_model import (RadioParams, ber_for_pdr, frame_success, latency_ms,
                                    predict_pdr, range_for_pdr)


def test_eighty_five_percent_at_a_hundred_metres_and_nothing_beyond():
    """The organisers' 100 m range (Q13), and the figure the relay spacing is built on."""
    p = RadioParams()
    assert predict_pdr(p, (0, 0, 35), (99.5, 0, 35)) == pytest.approx(0.85, abs=0.05)
    assert predict_pdr(p, (0, 0, 35), (100.5, 0, 35)) == 0.0
    assert range_for_pdr(p, 0.85) == pytest.approx(100.0, abs=0.5)
    assert range_for_pdr(p, 0.99) < range_for_pdr(p, 0.85)


def test_pdr_falls_off_with_distance():
    p = RadioParams()
    near = predict_pdr(p, (0, 0, 35), (20, 0, 35))
    mid = predict_pdr(p, (0, 0, 35), (70, 0, 35))
    assert near > mid > 0.0


def test_frame_survival_under_a_bit_error_rate():
    """A 2048-bit frame: the slider's useful range is its first thousandth, by physics."""
    bits = 2048.0
    assert frame_success(0.0, bits) == 1.0
    assert frame_success(1.0, bits) == 0.0
    assert frame_success(1e-6, bits) == pytest.approx(0.998, abs=0.002)
    assert frame_success(1e-4, bits) == pytest.approx(0.815, abs=0.01)
    assert frame_success(1e-3, bits) == pytest.approx(0.129, abs=0.01)
    assert frame_success(1e-2, bits) < 1e-8
    # monotone, so a slider always makes things worse rather than oddly better
    values = [frame_success(b, bits) for b in (0.0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 0.5)]
    assert values == sorted(values, reverse=True)


def test_ber_for_pdr_inverts_frame_success():
    bits = 2048.0
    for target in (0.99, 0.9, 0.5, 0.1):
        ber = ber_for_pdr(target, bits)
        assert frame_success(ber, bits) == pytest.approx(target, rel=1e-6)
    assert 0.0 < ber_for_pdr(0.5, bits) < 1e-3


def test_baseline_ber_is_off_by_default():
    assert RadioParams().bit_error_rate == 0.0
    assert RadioParams().frame_bits == 2048.0


def test_latency_grows_as_delivery_falls():
    p = RadioParams()
    assert latency_ms(p, 1.0) == pytest.approx(p.hop_latency_ms)
    assert latency_ms(p, 0.5) > latency_ms(p, 1.0)


def test_unknown_radio_parameter_is_refused():
    with pytest.raises(ValueError):
        RadioParams.from_dict({"not_a_parameter": 1})


def test_multi_hop_arithmetic():
    assert hops_to_pdr([0.9, 0.9]) == pytest.approx(0.81)
    assert hops_to_pdr([]) == 1.0
    assert etx_of_route([0.5, 0.5]) == pytest.approx(4.0)
    assert math.isinf(etx_of_route([0.9, 0.0]))


def test_internal_backend_is_the_default_and_applies_the_hard_range():
    b = make_backend("internal", RadioParams())
    assert isinstance(b, InternalBackend)
    assert b.evaluate("a", "b", (0, 0, 35), (50, 0, 35), gcs_link=False).pdr > 0.9
    assert b.evaluate("a", "b", (0, 0, 35), (150, 0, 35), gcs_link=False).pdr == 0.0
    assert b.info()["backend"] == "internal"


def test_ns3_backend_refuses_rather_than_pretending():
    """A run must never report numbers from a channel it did not actually use."""
    with pytest.raises(RuntimeError, match="documented seam"):
        make_backend("ns3", RadioParams())
    assert Ns3Backend.name == "ns3"


def test_unknown_backend_is_refused():
    with pytest.raises(ValueError, match="unknown radio_backend"):
        make_backend("make_it_up", RadioParams())
