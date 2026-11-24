"""Drone point-of-view window - every UAV's own camera, live, in one labelled grid.

    Drone 1 | Drone 2
    Drone 3 | Drone 4

Subscribes to /<uav>/camera/image for every UAV in the scenario that carries a camera (the same
nadir survey camera poi_detector reads) and shows them side by side in one OpenCV window. It
only watches: it publishes nothing, never touches the swarm and never ends the run.

Keys in the window:  f = fullscreen on/off,  q or Esc = close the viewer (the run carries on).

It needs a display. With none (SSH, Docker, CI) it says so once and idles, so a headless run is
not affected. Turn it off explicitly with `ros2 launch uavx_swarm uavx.launch.py ... pov:=false`.
"""
from __future__ import annotations

import os
import re
import sys

import cv2
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..common import SENSOR_QOS, load_scenario, spin
from ..pov_grid import compose_grid, to_bgr


def drone_label(uav_id: str, index: int) -> str:
    """'uav3' -> 'Drone 3'; ids without a number fall back to their position in the fleet."""
    m = re.search(r"(\d+)$", uav_id)
    return f"Drone {int(m.group(1)) if m else index + 1}"


def has_display() -> bool:
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


class PovViewer(Node):
    def __init__(self):
        super().__init__("pov_viewer")
        self.declare_parameter("scenario", "gc1")
        self.declare_parameter("tile_w", 480)
        self.declare_parameter("tile_h", 360)
        self.declare_parameter("cols", 0)                  # 0 = as square as possible
        self.declare_parameter("fps", 15.0)                # window refresh, wall clock
        sc = load_scenario(self.get_parameter("scenario").value)
        self.uavs = [u for u in sc["uavs"] if u.get("camera", True)]
        self.labels = [drone_label(u["id"], i) for i, u in enumerate(self.uavs)]
        self.tile = (int(self.get_parameter("tile_w").value), int(self.get_parameter("tile_h").value))
        self.cols = int(self.get_parameter("cols").value)
        self.window = "UAV-X drone POV"
        self.frames: list = [None] * len(self.uavs)
        self.dirty = True
        self.shown = False
        self.fullscreen = False
        self.enabled = bool(self.uavs)

        if not self.uavs:
            self.get_logger().warn("no UAV in this scenario carries a camera - nothing to show")
        elif not has_display():
            self.get_logger().warn("no display (DISPLAY / WAYLAND_DISPLAY unset) - drone POV window disabled")
            self.enabled = False
        if not self.enabled:
            return

        for i, u in enumerate(self.uavs):
            self.create_subscription(Image, f"/{u['id']}/camera/image",
                                     lambda m, i=i: self._on_image(i, m), SENSOR_QOS)
        # wall-clock timer: the window stays responsive however slowly the simulation runs
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("fps").value)), self._draw,
                          clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(f"drone POV window: {', '.join(self.labels)}  (f = fullscreen, q = close)")

    def _on_image(self, i: int, msg: Image) -> None:
        frame = to_bgr(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        if frame is not None:
            self.frames[i] = frame
            self.dirty = True

    def _close(self, why: str) -> None:
        self.enabled = False
        try:
            cv2.destroyWindow(self.window)
        except cv2.error:
            pass
        self.get_logger().info(f"drone POV window closed ({why}); the run continues")

    def _draw(self) -> None:
        if not self.enabled:
            return
        try:
            if not self.shown:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            if self.dirty or not self.shown:
                canvas = compose_grid(self.frames, self.labels, self.tile, self.cols)
                if not self.shown:
                    cv2.resizeWindow(self.window, canvas.shape[1], canvas.shape[0])
                cv2.imshow(self.window, canvas)
                self.shown = True
                self.dirty = False
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return self._close("key")
            if key == ord("f"):
                self.fullscreen = not self.fullscreen
                cv2.setWindowProperty(self.window, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)
            if self.shown and cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                self._close("window closed")
        except cv2.error as exc:       # an OpenCV build without GUI support, or the display went away
            self.enabled = False
            self.get_logger().error(f"drone POV window disabled: {exc}")


def main(args=None):
    rclpy.init(args=args)
    spin(PovViewer(), on_exit=cv2.destroyAllWindows)


if __name__ == "__main__":
    main()
