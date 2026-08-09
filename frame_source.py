"""One place that decides where camera frames come from.

Picked by `camera.source` in config.yaml:

  local   USB webcam via cv2.VideoCapture(camera.index) — the SO-101 setup,
          where the webcam is plugged into the laptop.
  lekiwi  Pull frames out of the observation stream that lekiwi_host publishes
          over ZMQ. The arm joint positions ride along in the same message.
  url     cv2.VideoCapture(camera.url) for an MJPEG/RTSP endpoint — useful when
          a plain stream server runs on the Pi instead of lekiwi_host.

All three expose the cv2.VideoCapture surface the scripts already use
(`read()`, `release()`, `isOpened()`), so 01/02/04/05 only change one line.
"""
import base64
import json
import time

import numpy as np
import cv2


class FrameSourceError(RuntimeError):
    """Raised when a source cannot be opened — message is meant for the user."""


_ROTATIONS = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

# Pass as cam_name when the caller wants whatever the host happens to publish
# rather than a specific camera — 00.check_link.py uses it to discover the list
# before asking which one to show.
ANY_CAMERA = "*"

HOST_CMD = (
    "python -m lerobot.robots.lekiwi.lekiwi_host \\\n"
    "        --robot.id=<your_id> --host.connection_time_s=14400"
)


class _FrameSource:
    def __init__(self, rotate=0):
        deg = int(rotate) % 360
        if deg and deg not in _ROTATIONS:
            raise FrameSourceError(f"camera.rotate must be 0/90/180/270, got {rotate}")
        self._rot = _ROTATIONS.get(deg)

    def _orient(self, frame):
        if frame is not None and self._rot is not None:
            frame = cv2.rotate(frame, self._rot)
        return frame

    def isOpened(self):
        return True

    def release(self):
        pass

    def describe(self):
        return "frame source"


class LocalCamera(_FrameSource):
    """cv2.VideoCapture on a local device index."""

    def __init__(self, cam, rotate=0):
        super().__init__(rotate)
        self.index = int(cam["index"])
        self.cap = cv2.VideoCapture(self.index)
        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*str(cam.get("fourcc", "MJPG"))))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam["width"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam["height"]))
        self.cap.set(cv2.CAP_PROP_FPS, int(cam["fps"]))
        if not self.cap.isOpened():
            raise FrameSourceError(
                f"Cannot open local camera index {self.index}.\n"
                "  Check `ls /dev/video*` and camera.index in config.yaml."
            )

    def read(self):
        ok, frame = self.cap.read()
        return ok, self._orient(frame)

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.cap.release()

    def describe(self):
        return f"local camera index={self.index}"


class UrlStream(_FrameSource):
    """cv2.VideoCapture on an http/rtsp URL."""

    def __init__(self, url, rotate=0):
        super().__init__(rotate)
        self.url = str(url)
        if not self.url:
            raise FrameSourceError("camera.source is 'url' but camera.url is empty.")
        self.cap = cv2.VideoCapture(self.url)
        if not self.cap.isOpened():
            raise FrameSourceError(
                f"Cannot open stream {self.url}.\n"
                "  Is the stream server running, and is the port reachable?"
            )

    def read(self):
        ok, frame = self.cap.read()
        return ok, self._orient(frame)

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.cap.release()

    def describe(self):
        return f"url stream {self.url}"


class LeKiwiStream(_FrameSource):
    """Frames (and arm state) from the LeKiwi host's ZMQ observation stream.

    lekiwi_host publishes one JSON message per control cycle on a PUSH socket:
    float entries are the arm/base state, string entries are base64 JPEG frames
    keyed by camera name ("front", "wrist", ...).

    The socket is PUSH/PULL with CONFLATE, so the host round-robins between
    every connected consumer — run only ONE client against a host at a time.

    Colours arrive channel-swapped by default. lerobot's OpenCVCameraConfig
    defaults to color_mode=RGB, so the camera hands the host an RGB array,
    which the host then feeds straight to cv2.imencode — and that assumes BGR.
    `host_color_mode` says what the host actually sends so we can undo it;
    everything downstream (imshow, YOLO) wants BGR.
    """

    def __init__(self, remote_ip, cam_name="front", port=5556,
                 connect_timeout_s=10.0, read_timeout_s=2.0, rotate=0,
                 host_color_mode="rgb"):
        super().__init__(rotate)
        self.host_color_mode = str(host_color_mode).lower()
        if self.host_color_mode not in ("rgb", "bgr"):
            raise FrameSourceError(
                f"lekiwi.host_color_mode must be 'rgb' or 'bgr', got {host_color_mode!r}")
        try:
            import zmq
        except ImportError as e:
            raise FrameSourceError(
                "pyzmq is required for camera.source: lekiwi.\n"
                "  pip install pyzmq"
            ) from e

        self._zmq = zmq
        self.cam_name = str(cam_name)
        self.address = f"tcp://{remote_ip}:{int(port)}"
        self.read_timeout_s = float(read_timeout_s)
        self.state = {}          # latest non-image entries (arm joints, base vel)
        self.camera_names = []

        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PULL)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.connect(self.address)
        self._poller = zmq.Poller()
        self._poller.register(self._sock, zmq.POLLIN)

        obs = self._recv(float(connect_timeout_s) * 1000.0)
        if obs is None:
            self.release()
            raise FrameSourceError(
                f"No observation from the LeKiwi host at {self.address} "
                f"within {connect_timeout_s:g}s.\n"
                "  On the robot, start the host (and keep it running):\n"
                f"    {HOST_CMD}\n"
                "  Note the host exits after --host.connection_time_s seconds "
                "(default 30).\n"
                "  Also make sure no other client (teleop, recorder, another "
                "script) is already\n"
                "  draining the same stream — they steal every other frame."
            )

        self.camera_names = sorted(k for k, v in obs.items() if isinstance(v, str))
        if not self.camera_names:
            self.release()
            raise FrameSourceError(
                f"The host at {self.address} publishes no cameras.\n"
                "  Start it with --robot.cameras=... , or check that the Pi can "
                "open its /dev/video* devices."
            )
        if self.cam_name == ANY_CAMERA:
            self.cam_name = self.camera_names[0]
        elif self.cam_name not in self.camera_names:
            available = self.camera_names
            self.release()
            raise FrameSourceError(
                f"Camera '{self.cam_name}' is not in the stream.\n"
                f"  Available: {available}\n"
                "  Set camera.name in config.yaml to one of those."
            )

    def use_camera(self, cam_name):
        """Point at a different camera. No reconnect — every observation
        carries all of them, so this only changes which key we decode."""
        name = str(cam_name)
        if name not in self.camera_names:
            raise FrameSourceError(
                f"Camera '{name}' is not in the stream. Available: {self.camera_names}")
        self.cam_name = name

    def _recv(self, timeout_ms):
        """Return the newest observation dict, or None if nothing arrived."""
        try:
            socks = dict(self._poller.poll(timeout_ms))
        except self._zmq.ZMQError:
            return None
        if self._sock not in socks:
            return None

        latest = None
        while True:
            try:
                latest = self._sock.recv_string(self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
        if latest is None:
            return None
        try:
            return json.loads(latest)
        except json.JSONDecodeError:
            return None

    def read(self):
        """Block up to read_timeout_s for a fresh frame. (ok, frame) like cv2."""
        deadline = time.time() + self.read_timeout_s
        while True:
            remaining_ms = max((deadline - time.time()) * 1000.0, 0.0)
            obs = self._recv(remaining_ms)
            if obs is not None:
                self.state = {k: v for k, v in obs.items() if not isinstance(v, str)}
                encoded = obs.get(self.cam_name)
                if encoded:
                    buf = np.frombuffer(base64.b64decode(encoded), dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        if self.host_color_mode == "rgb":
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        return True, self._orient(frame)
            if time.time() >= deadline:
                return False, None

    def release(self):
        try:
            self._sock.close(linger=0)
            self._ctx.term()
        except Exception:
            pass

    def describe(self):
        return f"lekiwi '{self.cam_name}' @ {self.address}"


def open_frame_source(cfg, cam_name=None):
    """Build the frame source described by cfg (the parsed config.yaml)."""
    cam = cfg.get("camera", {})
    source = str(cam.get("source", "local")).lower()
    rotate = cam.get("rotate", 0)

    if source == "local":
        return LocalCamera(cam, rotate)
    if source == "url":
        return UrlStream(cam.get("url", ""), rotate)
    if source == "lekiwi":
        lk = cfg.get("lekiwi", {})
        if not lk.get("remote_ip"):
            raise FrameSourceError(
                "camera.source is 'lekiwi' but lekiwi.remote_ip is not set in config.yaml."
            )
        return LeKiwiStream(
            remote_ip=lk["remote_ip"],
            cam_name=cam_name or cam.get("name", "front"),
            port=lk.get("port_zmq_observations", 5556),
            connect_timeout_s=lk.get("connect_timeout_s", 10.0),
            read_timeout_s=lk.get("read_timeout_s", 2.0),
            rotate=rotate,
            host_color_mode=lk.get("host_color_mode", "rgb"),
        )
    raise FrameSourceError(
        f"Unknown camera.source '{source}' — expected local, lekiwi, or url."
    )
