"""The drone point-of-view grid: layout, labels, missing feeds and image decoding.

Pure numpy + OpenCV, so it runs without a simulator or a ROS 2 install.
"""
import numpy as np

from uavx_swarm.pov_grid import BACKGROUND, compose_grid, grid_shape, to_bgr


def frame(value, w=640, h=480):
    f = np.zeros((h, w, 3), dtype=np.uint8)
    f[:, :] = value
    return f


def test_grid_is_as_square_as_possible():
    assert grid_shape(1) == (1, 1)
    assert grid_shape(2) == (1, 2)
    assert grid_shape(4) == (2, 2)      # the reference video: Drone 1-2 over Drone 3-4
    assert grid_shape(5) == (2, 3)      # gc1's five UAVs
    assert grid_shape(5, cols=5) == (1, 5)
    assert grid_shape(3, cols=9) == (1, 3)


def test_four_drones_fill_a_two_by_two_canvas_in_order():
    colours = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
    canvas = compose_grid([frame(c) for c in colours], [f"Drone {i}" for i in range(1, 5)], (480, 360))
    assert canvas.shape == (720, 960, 3)
    # sample the middle of each tile, clear of the label text in its top-left corner
    for i, colour in enumerate(colours):
        r, c = divmod(i, 2)
        assert tuple(canvas[r * 360 + 250, c * 480 + 300]) == colour


def test_label_is_drawn_on_every_tile():
    plain = compose_grid([frame((40, 40, 40))] * 2, ["", ""], (480, 360))
    labelled = compose_grid([frame((40, 40, 40))] * 2, ["Drone 1", "Drone 2"], (480, 360))
    for c in range(2):
        tile = (slice(0, 40), slice(c * 480, c * 480 + 160))
        assert (labelled[tile] != plain[tile]).any()          # text pixels appeared in the corner


def test_missing_feed_shows_no_signal_instead_of_crashing():
    canvas = compose_grid([frame((200, 200, 200)), None], ["Drone 1", "Drone 2"], (480, 360))
    assert canvas.shape == (360, 960, 3)
    assert tuple(canvas[300, 700]) == BACKGROUND              # the empty tile is dark
    assert (canvas[170:190, 480 + 200:480 + 300] != np.array(BACKGROUND)).any()   # "no signal" text


def test_unused_cells_stay_background():
    canvas = compose_grid([frame((9, 9, 9))] * 5, [f"Drone {i}" for i in range(1, 6)], (240, 180))
    assert canvas.shape == (360, 720, 3)
    assert tuple(canvas[300, 600]) == BACKGROUND              # the 6th cell of the 3x2 grid


def test_odd_aspect_frames_are_letterboxed_not_stretched():
    wide = frame((255, 255, 255), w=800, h=200)
    canvas = compose_grid([wide], ["Drone 1"], (400, 300))
    assert tuple(canvas[10, 200]) == BACKGROUND               # bar above the picture
    assert tuple(canvas[150, 200]) == (255, 255, 255)


def test_rgb8_is_decoded_to_bgr_and_padding_is_ignored():
    w, h, step = 4, 2, 16                                     # 12 bytes of pixels + 4 of padding per row
    row = bytes([255, 0, 0] * w) + bytes(4)                   # pure red in rgb8
    out = to_bgr(row * h, w, h, step, "rgb8")
    assert out.shape == (h, w, 3) and tuple(out[0, 0]) == (0, 0, 255)     # red in BGR
    assert to_bgr(row * h, w, h, step, "bgr8")[0, 0].tolist() == [255, 0, 0]


def test_mono_and_unknown_encodings():
    assert to_bgr(bytes([7] * 6), 3, 2, 3, "mono8").shape == (2, 3, 3)
    assert to_bgr(bytes(6), 3, 2, 3, "32FC1") is None
    assert to_bgr(bytes(5), 3, 2, 3, "mono8") is None         # truncated payload
