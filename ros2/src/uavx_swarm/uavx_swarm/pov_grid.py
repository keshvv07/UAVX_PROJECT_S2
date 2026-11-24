"""Tile every drone's camera frame into one labelled grid (the "drone POV" window).

Pure numpy + OpenCV, no ROS imports, so it is unit-tested without a simulator
(test/test_pov_grid.py). The viewer node (nodes/pov_viewer.py) feeds it the latest frame of
each UAV and shows the result:

    +-----------+-----------+
    | Drone 1   | Drone 2   |        each tile is that UAV's own downward camera,
    |           |           |        labelled top-left; a UAV with no frame yet (or one
    +-----------+-----------+        whose camera died with it) shows "no signal".
    | Drone 3   | Drone 4   |
    +-----------+-----------+
"""
from __future__ import annotations

import math

import cv2
import numpy as np

BACKGROUND = (24, 24, 24)       # BGR


def grid_shape(count: int, cols: int = 0) -> tuple[int, int]:
    """(rows, cols) for `count` tiles: as square as possible unless `cols` is given."""
    count = max(1, int(count))
    cols = int(cols) if cols and cols > 0 else math.ceil(math.sqrt(count))
    cols = max(1, min(cols, count))
    return math.ceil(count / cols), cols


def to_bgr(data, width: int, height: int, step: int, encoding: str):
    """sensor_msgs/Image payload -> HxWx3 BGR uint8 for OpenCV, or None if unsupported."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8UC1": 1, "8UC3": 3}.get(encoding)
    if channels is None or width <= 0 or height <= 0:
        return None
    step = int(step) if step else width * channels
    raw = np.frombuffer(bytes(data), dtype=np.uint8)
    if raw.size < step * height or step < width * channels:
        return None
    img = raw[: step * height].reshape(height, step)[:, : width * channels].reshape(height, width, channels)
    if encoding in ("rgb8",):
        return np.ascontiguousarray(img[:, :, ::-1])
    if encoding in ("bgr8", "8UC3"):
        return np.ascontiguousarray(img)
    if encoding == "rgba8":
        return np.ascontiguousarray(img[:, :, 2::-1])
    if encoding == "bgra8":
        return np.ascontiguousarray(img[:, :, :3])
    return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_GRAY2BGR)       # mono8 / 8UC1


def draw_label(tile: np.ndarray, text: str, origin: tuple[int, int] = (10, 24), scale: float = 0.65) -> None:
    """White text with a dark outline so it reads over sky, grass and white pads alike."""
    cv2.putText(tile, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(tile, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def _fit(frame: np.ndarray, tw: int, th: int) -> np.ndarray:
    """Scale a frame into a tw x th tile, keeping its aspect ratio (letterboxed)."""
    h, w = frame.shape[:2]
    scale = min(tw / w, th / h)
    nw, nh = max(1, round(w * scale)), max(1, round(h * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    tile = np.full((th, tw, 3), BACKGROUND, dtype=np.uint8)
    y0, x0 = (th - nh) // 2, (tw - nw) // 2
    tile[y0:y0 + nh, x0:x0 + nw] = resized
    return tile


def compose_grid(frames: list, labels: list[str], tile_size: tuple[int, int] = (480, 360),
                 cols: int = 0) -> np.ndarray:
    """One BGR image holding every UAV's latest frame. `frames[i]` is None until it arrives."""
    tw, th = tile_size
    rows, cols = grid_shape(len(frames), cols)
    canvas = np.full((rows * th, cols * tw, 3), BACKGROUND, dtype=np.uint8)
    for i, frame in enumerate(frames):
        r, c = divmod(i, cols)
        if frame is None:
            tile = np.full((th, tw, 3), BACKGROUND, dtype=np.uint8)
            cv2.putText(tile, "no signal", (tw // 2 - 55, th // 2 + 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (120, 120, 120), 1, cv2.LINE_AA)
        else:
            tile = _fit(frame, tw, th)
        draw_label(tile, labels[i] if i < len(labels) else f"Drone {i + 1}")
        canvas[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile
    return canvas
