"""Read a Smartglove as robot-hand control values: finger closure and wrist angles.

Where `rgmp_client_example.py` prints raw streams, this prints the two things a
robot hand actually needs to be driven:

    * per finger -- how closed it is, as 0 (open) .. 1 (fist) plus the raw curl
      in degrees and the individual MCP/PIP/DIP angles;
    * for the wrist -- flexion/extension, adduction/abduction and rotation.

Input
-----
This reads the *solved* stream from `rkk-hand-solver`, not the raw driver
stream. The raw stream (port 12276) carries eight sensor poses, which are not
finger angles; the solver turns them into a per-joint skeleton. Start it first:

    bin/rkk-hand-solver                 # serves solved hands on 127.0.0.1:12277

Run it:

    python3 rgmp_client_rh8d.py                     # live readout
    python3 rgmp_client_rh8d.py --json              # one JSON object per line
    python3 rgmp_client_rh8d.py --calibrate         # measure your open/fist range
    python3 rgmp_client_rh8d.py --check             # verify the wrist sign convention

The derivation itself lives in `hand_joints.py`, which documents the angle
conventions and has no dependency on this script.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import threading
import time
from pathlib import Path

from hand_joints import (
    FINGER_NAMES,
    Calibration,
    HandModel,
    HandPose,
    build_hand_model,
    is_solved_hand,
    pose_from_frame,
)
from rgmp_client import RgmpClient, StreamLayout

SOLVER_PORT = 12277
DEFAULT_CALIBRATION = Path(__file__).with_name("rh8d_calibration.json")


class HandReader:
    """Background RGMP reader that keeps the latest pose per solved hand.

    The socket loop runs on its own thread so the foreground can print at its
    own rate, or block on a calibration prompt, without stalling the stream or
    letting frames pile up.
    """

    def __init__(self, host: str, port: int, calibration: Calibration) -> None:
        self._client = RgmpClient(host, port)
        self._calibration = calibration
        self._models: dict[int, HandModel] = {}
        self._poses: dict[int, HandPose] = {}
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.saw_definition = threading.Event()
        self.finished = threading.Event()
        self.non_solved_device: str | None = None
        self.error: Exception | None = None

        self._client.on_definition = self._on_definition
        self._client.on_data = self._on_data
        self._client.on_disconnect = self._on_disconnect
        self._client.on_error = self._on_error

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_definition(self, definition: dict, layout: StreamLayout) -> None:
        if not is_solved_hand(definition):
            self.non_solved_device = str(definition.get("device_type", "?"))
            return
        model = build_hand_model(definition, layout)
        if model is None:
            return
        with self._lock:
            self._models[model.device_id] = model
        print(
            f"# {model.side} hand, device {model.device_id}, "
            f"{len(model.joints)} joints",
            file=sys.stderr,
        )
        self.saw_definition.set()

    def _on_data(
        self,
        device_id: int,
        group_id: int,
        ts_us: int,
        payload: bytes,
        layout: StreamLayout,
    ) -> None:
        model = self._models.get(device_id)
        if model is None or group_id != model.group_id:
            return
        floats = list(struct.unpack_from(f"<{len(payload) // 4}f", payload, 0))
        pose = pose_from_frame(model, ts_us, floats, self._calibration)
        with self._lock:
            self._poses[device_id] = pose

    def _on_disconnect(self, device_id: int) -> None:
        with self._lock:
            self._models.pop(device_id, None)
            self._poses.pop(device_id, None)

    def _on_error(self, exc: Exception) -> None:
        self.error = exc
        print(f"error: {exc}", file=sys.stderr)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def _run(self) -> None:
        try:
            self._client.run()
        finally:
            self.finished.set()

    def __enter__(self) -> "HandReader":
        self._client.__enter__()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self._client.__exit__(exc_type, exc, tb)

    def latest(self) -> list[HandPose]:
        """The most recent pose for each hand, ordered by device id."""
        with self._lock:
            return [self._poses[key] for key in sorted(self._poses)]


# ── Output ────────────────────────────────────────────────────────────────────


def _bar(value: float, width: int = 12) -> str:
    filled = int(round(max(0.0, min(1.0, value)) * width))
    return "█" * filled + "·" * (width - filled)


def format_pose(pose: HandPose) -> str:
    wrist = pose.wrist
    lines = [
        f"{pose.side} hand  (device {pose.device_id}, ts {pose.timestamp_us})",
        f"  wrist   flexion {wrist.flexion_deg:+7.1f}°   "
        f"adduction {wrist.adduction_deg:+7.1f}°   "
        f"rotation {wrist.rotation_deg:+7.1f}°",
        f"  cmc-add {_bar(pose.thumb.adduction)} {pose.thumb.adduction:4.2f}  "
        f"span {pose.thumb.span_deg:6.1f}°  "
        f"palmar-abd {pose.thumb.palmar_abduction_deg:+6.1f}°",
    ]
    for finger in pose.fingers:
        joints = " ".join(f"{a:5.1f}" for a in finger.joint_angles)
        lines.append(
            f"  {finger.name:<7} {_bar(finger.closure)} {finger.closure:4.2f}  "
            f"curl {finger.curl_deg:6.1f}°  joints [{joints}]  "
            f"spread {finger.spread_deg:+6.1f}°"
        )
    return "\n".join(lines)


# ── Calibration ───────────────────────────────────────────────────────────────


def _sample_pose(
    reader: HandReader, seconds: float = 0.6
) -> tuple[dict[str, float], float | None]:
    """Average curl per finger and thumb span over a short window.

    Averaged so one noisy frame can't set an endpoint. Uses the first hand only
    -- calibrate one glove at a time.
    """
    totals: dict[str, list[float]] = {name: [] for name in FINGER_NAMES}
    spans: list[float] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        poses = reader.latest()
        if poses:
            for finger in poses[0].fingers:
                totals[finger.name].append(finger.curl_deg)
            spans.append(poses[0].thumb.span_deg)
        time.sleep(0.02)
    curls = {
        name: sum(values) / len(values) for name, values in totals.items() if values
    }
    return curls, (sum(spans) / len(spans) if spans else None)


def run_calibration(reader: HandReader, path: Path) -> int:
    """Measure each axis over a pose that moves that axis and little else.

    The thumb needs two poses of its own. Closing a fist adducts the thumb and
    flexes it at the same time, so measuring both from one open/fist pair gives
    each axis only the travel it happened to make along that one diagonal --
    and the flexion range that comes out of it is so narrow that the flexion
    channel ends up tracking adduction instead of its own joint.
    """
    print(
        "Calibrating. Four poses, held for about a second each.\n"
        "Keep the motions separate -- each pose should move one thing.\n",
        file=sys.stderr,
    )

    input("1/4  Hand FLAT and OPEN, thumb straight and spread wide, then Enter... ")
    open_deg, open_span = _sample_pose(reader)
    if not open_deg:
        print("No frames received; is a glove streaming?", file=sys.stderr)
        return 1

    input("2/4  Curl the FOUR FINGERS into a fist, thumb still out and spread, Enter... ")
    fist_deg, _ = _sample_pose(reader)

    input("3/4  Thumb STRAIGHT, sweep it across the palm toward the little finger, Enter... ")
    _, closed_span = _sample_pose(reader)

    input("4/4  Curl the thumb IN toward the palm as far as it goes, then Enter... ")
    thumb_deg, _ = _sample_pose(reader)

    calibration = Calibration()
    calibration.open_deg.update(open_deg)
    # Fingers close in pose 2; the thumb's own flexion comes from pose 4, so the
    # thumb keeps whatever it was doing during the fist out of its range.
    calibration.closed_deg.update(
        {name: value for name, value in fist_deg.items() if name != "thumb"}
    )
    # Whichever pose bent the thumb further wins. Pose 4 targets it directly,
    # but a fist can out-curl a deliberate thumb curl, and the endpoint that
    # matters is the furthest the joint actually went.
    thumb_candidates = [
        value for value in (thumb_deg.get("thumb"), fist_deg.get("thumb"))
        if value is not None
    ]
    if thumb_candidates:
        calibration.closed_deg["thumb"] = max(thumb_candidates)
    if open_span is not None and closed_span is not None:
        calibration.thumb_open_span_deg = open_span
        calibration.thumb_closed_span_deg = closed_span

    _report_calibration(calibration, path)
    path.write_text(json.dumps(calibration.to_dict(), indent=2) + "\n")
    print(f"\nWrote {path}", file=sys.stderr)
    return 0


# Below this much travel a channel is too cramped to drive a joint smoothly:
# noise and cross-talk from neighbouring joints start to fill a real fraction of
# its range. Curl and adduction need different thresholds -- a thumb sweeping
# the width of the palm covers far fewer degrees than a finger closing, and is
# not cramped for it. Rough guides, not measurements.
MIN_CURL_RANGE_DEG = 60.0
MIN_SPAN_RANGE_DEG = 25.0


def _report_calibration(calibration: Calibration, path: Path) -> None:
    """Print the measured range per axis, and name any that came out cramped."""
    suspect: list[str] = []
    print("", file=sys.stderr)
    for name in FINGER_NAMES:
        lo = calibration.open_deg.get(name, 0.0)
        hi = calibration.closed_deg.get(name, 0.0)
        span = hi - lo
        flag = "  <- little travel" if span < MIN_CURL_RANGE_DEG else ""
        if flag:
            suspect.append(name)
        print(
            f"  {name:<7} curl {lo:6.1f}° -> {hi:6.1f}°   range {span:6.1f}°{flag}",
            file=sys.stderr,
        )

    span = abs(calibration.thumb_closed_span_deg - calibration.thumb_open_span_deg)
    flag = "  <- little travel" if span < MIN_SPAN_RANGE_DEG else ""
    if flag:
        suspect.append("thumb adduction")
    print(
        f"  {'thumb':<7} span {calibration.thumb_open_span_deg:6.1f}° -> "
        f"{calibration.thumb_closed_span_deg:6.1f}°   range {span:6.1f}°{flag}",
        file=sys.stderr,
    )

    if suspect:
        print(
            f"\nwarning: little range for {', '.join(suspect)} — those channels "
            "will be twitchy and may follow neighbouring joints. Redo that pose, "
            "moving the joint through everything it has.",
            file=sys.stderr,
        )


def run_check(reader: HandReader) -> int:
    """Live wrist readout with the motion each value should respond to.

    The three wrist signs are a convention this code asserts rather than
    measures (see hand_joints.WRIST_SIGNS); this is how you confirm them.
    """
    print(
        "Wrist sign check. Do each motion and watch which value moves:\n"
        "  flexion    should go POSITIVE as you bend the palm toward the forearm\n"
        "  adduction  should go POSITIVE toward the little-finger side\n"
        "  rotation   should go POSITIVE as the palm turns to face down\n"
        "  thumb adduction  should go POSITIVE as you close the thumb onto the palm\n"
        "  thumb palmar-abd should go POSITIVE as you lift the thumb off the palm\n"
        "If a wrist value reads inverted, flip its component in\n"
        "hand_joints.WRIST_SIGNS; for the thumb's zero point, adjust\n"
        "hand_joints.THUMB_NEUTRAL_SPAN_DEG so a relaxed hand reads near 0.\n"
        "Ctrl-C to stop.\n",
        file=sys.stderr,
    )
    while not reader.finished.is_set():
        poses = reader.latest()
        if poses:
            w = poses[0].wrist
            t = poses[0].thumb
            print(
                f"\rwrist: flex {w.flexion_deg:+6.1f}° "
                f"add {w.adduction_deg:+6.1f}° rot {w.rotation_deg:+6.1f}°   "
                f"thumb: add {t.adduction_deg:+6.1f}° "
                f"palm-abd {t.palmar_abduction_deg:+6.1f}°   ",
                end="",
                flush=True,
            )
        time.sleep(0.05)
    return 0


def run_thumb(reader: HandReader) -> int:
    """Live thumb readout: the two axes side by side, with the range each has
    covered so far.

    The point of the ranges is to answer "are these separable?" by doing rather
    than by argument: move one axis while holding the other, and only the one
    you are moving should grow.
    """
    print(
        "Thumb axis check. Do these one at a time and watch which row moves:\n"
        "  1. hold the thumb straight, sweep it across the palm and back\n"
        "     -> adduction should sweep, flexion should barely move\n"
        "  2. hold it spread, curl and straighten just the thumb\n"
        "     -> flexion should sweep, adduction should barely move\n"
        "Ctrl-C to stop; 'r' + Enter is not needed, ranges accumulate.\n",
        file=sys.stderr,
    )
    lo: dict[str, float] = {}
    hi: dict[str, float] = {}

    def track(key: str, value: float) -> str:
        lo[key] = min(lo.get(key, value), value)
        hi[key] = max(hi.get(key, value), value)
        return f"{value:+7.1f}° [range {hi[key] - lo[key]:6.1f}°]"

    while not reader.finished.is_set():
        poses = reader.latest()
        if poses:
            pose = poses[0]
            thumb = next((f for f in pose.fingers if f.name == "thumb"), None)
            if thumb is not None:
                mcp = thumb.joint_angles[0] if thumb.joint_angles else 0.0
                ip = thumb.joint_angles[1] if len(thumb.joint_angles) > 1 else 0.0
                print(
                    f"\r  adduction {track('add', pose.thumb.span_deg)}   "
                    f"flexion MCP {track('mcp', mcp)} IP {track('ip', ip)}   ",
                    end="",
                    flush=True,
                )
        time.sleep(0.05)
    return 0


# ── Main ──────────────────────────────────────────────────────────────────────


def main(args: argparse.Namespace) -> int:
    calibration = Calibration()
    if args.calibration.exists() and not args.calibrate:
        calibration = Calibration.from_dict(json.loads(args.calibration.read_text()))
        print(f"# using calibration {args.calibration}", file=sys.stderr)
    elif not args.calibrate:
        print(
            "# no calibration file; closure uses default ranges. "
            "Run with --calibrate for your hand.",
            file=sys.stderr,
        )

    try:
        with HandReader(args.host, args.port, calibration) as reader:
            return _run_session(reader, args)
    except OSError as exc:
        print(f"Cannot connect to {args.host}:{args.port} — {exc}", file=sys.stderr)
        print(
            "Is rkk-hand-solver running? It serves solved hands on "
            f"127.0.0.1:{SOLVER_PORT}.",
            file=sys.stderr,
        )
        return 1


def _run_session(reader: HandReader, args: argparse.Namespace) -> int:
    if not reader.saw_definition.wait(timeout=5.0):
        if reader.non_solved_device:
            print(
                f"Connected, but {args.host}:{args.port} is streaming "
                f"'{reader.non_solved_device}', not solved hands. Point this at "
                f"rkk-hand-solver (default port {SOLVER_PORT}), not the raw driver.",
                file=sys.stderr,
            )
        else:
            print("No solved hand appeared within 5s.", file=sys.stderr)
        return 1

    try:
        if args.calibrate:
            return run_calibration(reader, args.calibration)
        if args.check:
            return run_check(reader)
        if args.thumb:
            return run_thumb(reader)
        return run_readout(reader, args)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    return 0


def run_readout(reader: HandReader, args: argparse.Namespace) -> int:
    """Print each hand's values, at most `--rate` times per second.

    Poses are skipped when their timestamp has not advanced, so a rate faster
    than the stream repeats nothing and `--rate 0` means "every frame, once".
    """
    interval = 0.001 if args.rate <= 0 else 1.0 / args.rate
    printed: dict[int, int] = {}
    while not reader.finished.is_set():
        for pose in reader.latest():
            if printed.get(pose.device_id) == pose.timestamp_us:
                continue
            printed[pose.device_id] = pose.timestamp_us
            if args.json:
                print(json.dumps(pose.as_dict()), flush=True)
            else:
                print(format_pose(pose) + "\n", flush=True)
        time.sleep(interval)

    print("Stream ended.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default="127.0.0.1", help="solver host")
    parser.add_argument(
        "--port",
        type=int,
        default=SOLVER_PORT,
        help=f"rkk-hand-solver RGMP port (default: {SOLVER_PORT})",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="print rate in Hz; 0 prints as fast as frames arrive (default: 10)",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit one JSON object per line"
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="measure this hand's open/fist range and write it to the calibration file",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="live wrist readout for verifying the sign convention",
    )
    parser.add_argument(
        "--thumb",
        action="store_true",
        help="live thumb readout for checking that adduction and flexion move "
        "independently",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help=f"calibration file (default: {DEFAULT_CALIBRATION.name})",
    )
    sys.exit(main(parser.parse_args()))
