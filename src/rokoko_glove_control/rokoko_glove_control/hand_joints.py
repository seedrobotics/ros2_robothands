"""Turn rkk-hand-solver's solved-hand stream into robot-hand control values.

The Smartgloves raw stream (`rokoko-sdk`, port 12276) carries eight *sensor*
poses, not joint angles: one IMU per finger plus hub/arm, each a FLOAT[7]
TRANSFORM relative to the hub. Those are not directly usable as "how closed is
this finger".

`rkk-hand-solver` (port 12277) consumes that stream and re-emits a solved
skeleton: a `joints_local` group of FLOAT[4] ORIENTATION streams, one per joint,
each expressed relative to its parent joint, plus the rest pose in `static_data`.
That is the input this module expects.

From it, it derives:

  * per finger -- flexion at each joint, their sum as a total curl in degrees,
    and a normalized 0..1 closure (see `Calibration`), plus sideways spread.
  * for the wrist -- flexion/extension, adduction/abduction (radial-ulnar
    deviation) and rotation (pronation/supination), from the hand joint's
    rotation relative to the forearm.

Conventions
-----------
The solver's skeleton has bones pointing along local -Y, hinging about local X
(see `examples/web/rgmp-ws-bridge-web/src/solver/` for the reference solver this
mirrors). Finger curl here is measured geometrically, as the angle between
consecutive bone directions after forward kinematics, so it carries no sign
ambiguity: it is always >= 0 and grows as the finger closes.

The three wrist values are signed, and their sign convention is asserted, not
measured -- positive means flexion (palm toward the inner forearm), adduction
(ulnar/little-finger side) and pronation (palm turning down). Left-hand values
are mirrored so both hands read the same anatomically. Verify with
`rgmp_client_rh8d.py --check` and flip `WRIST_SIGNS` if a motion reads inverted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w

QUAT_IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)

# Bones point along local -Y and hinge about local X, which leaves +Z as the
# palm normal. Everything below that needs a body axis takes it from here.
BONE_AXIS: Vec3 = (0.0, -1.0, 0.0)
# The axis a finger joint flexes about, in the parent bone's frame. Bones run
# along -Y and the palm normal is +Z, so flexion is rotation about X. Used to
# give each joint angle a sign: without one, `acos` reports a hyperextended
# joint as though it were flexed by the same amount, and a thumb stretched back
# past straight reads as a thumb curling in. Mirrored per hand like the other
# signed quantities here; confirm with `rgmp_client_rh8d.py --check`.
FLEXION_AXIS: Vec3 = (1.0, 0.0, 0.0)

#: Which of the thumb's joints count toward its curl, as indices into
#: `FingerState.joint_angles` (0 = MCP, 1 = IP).
#:
#: The thumb's MCP abducts about as much as it flexes, so it is the joint where
#: adduction leaks into the flexion reading. The IP is a pure hinge with no
#: sideways freedom, so it cannot mix the two at all -- it reads as the more
#: reliable of the pair in the solver's own web view, and it is the default
#: here for that reason. Projection (above) also cleans the MCP up
#: geometrically, so `(0, 1)` is available if the extra range the MCP
#: contributes turns out to be worth the residual coupling.
#: Check either with `rgmp_client_rh8d.py --thumb`.
THUMB_CURL_JOINTS: tuple[int, ...] = (1,)
TWIST_AXIS: Vec3 = (0.0, 1.0, 0.0)
PALM_NORMAL: Vec3 = (0.0, 0.0, 1.0)

# In-plane thumb-to-index angle of a relaxed hand, in degrees. Only sets where
# zero falls on ThumbState.adduction_deg; it does not scale anything. Adjust it
# if your relaxed hand does not read near zero.
THUMB_NEUTRAL_SPAN_DEG: float = 40.0

# Signs applied to the wrist triple so that positive reads as flexion /
# adduction / pronation. Flip a component here if `--check` shows a motion
# reading inverted on your glove.
WRIST_SIGNS: tuple[float, float, float] = (1.0, 1.0, 1.0)


# ── Vector and quaternion helpers ─────────────────────────────────────────────


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_length(a: Vec3) -> float:
    return math.sqrt(v_dot(a, a))


def v_normalize(a: Vec3) -> Vec3 | None:
    """Unit vector, or None when `a` is too short to have a direction."""
    length = v_length(a)
    if length < 1e-12:
        return None
    return (a[0] / length, a[1] / length, a[2] / length)


def angle_between(a: Vec3, b: Vec3) -> float:
    """Unsigned angle between two vectors, in degrees; 0 if either is zero."""
    ua, ub = v_normalize(a), v_normalize(b)
    if ua is None or ub is None:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, v_dot(ua, ub)))))


def project_onto_plane(v: Vec3, normal: Vec3) -> Vec3:
    """`v` with its component along `normal` removed."""
    n = v_normalize(normal)
    if n is None:
        return v
    d = v_dot(v, n)
    return (v[0] - n[0] * d, v[1] - n[1] * d, v[2] - n[2] * d)


def signed_angle(a: Vec3, b: Vec3, axis: Vec3) -> float:
    """Angle from `a` to `b` measured about `axis` (right-handed), in degrees."""
    unsigned = angle_between(a, b)
    return unsigned if v_dot(v_cross(a, b), axis) >= 0 else -unsigned


def q_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product: apply `b`, then `a`."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def q_normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return QUAT_IDENTITY
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_rotate(v: Vec3, q: Quat) -> Vec3:
    """Rotate `v` by `q` (q v q⁻¹)."""
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    )


def swing_twist(q: Quat, axis: Vec3) -> tuple[Quat, Quat]:
    """Split `q` into `(swing, twist)` about `axis`, such that q == swing * twist.

    `twist` is the part of the rotation about `axis` itself; `swing` is what is
    left, and moves `axis` to where `q` puts it. Used to separate the wrist's
    pronation (twist along the forearm) from its bend (swing).
    """
    q = q_normalize(q)
    d = v_dot((q[0], q[1], q[2]), axis)
    twist = q_normalize((axis[0] * d, axis[1] * d, axis[2] * d, q[3]))
    swing = q_mul(q, q_conjugate(twist))
    return swing, twist



def rotation_vector(q: Quat) -> Vec3:
    """`q` as an axis-angle vector (axis scaled by the angle in degrees).

    For a rotation about a single coordinate axis the components are exactly the
    angle about that axis, which is what makes it a usable decomposition for the
    wrist's bend into flexion and deviation.
    """
    q = q_normalize(q)
    if q[3] < 0.0:  # shortest arc, so the angle stays in [0, 180]
        q = (-q[0], -q[1], -q[2], -q[3])
    sin_half = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2])
    if sin_half < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = math.degrees(2.0 * math.atan2(sin_half, q[3]))
    scale = angle / sin_half
    return (q[0] * scale, q[1] * scale, q[2] * scale)


def twist_angle(q: Quat, axis: Vec3) -> float:
    """Signed rotation of `q` about `axis`, in degrees."""
    _, twist = swing_twist(q, axis)
    d = v_dot((twist[0], twist[1], twist[2]), axis)
    return math.degrees(2.0 * math.atan2(d, twist[3]))


# ── Skeleton, built from the stream definition ────────────────────────────────

FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
# The solver names fingers finger1..finger5, thumb first.
FINGER_SLUGS = ("finger1", "finger2", "finger3", "finger4", "finger5")
# Joint names along a finger, distal-most last. The thumb has no medial joint,
# so its chain is one shorter; `_finger_chain` drops what a device doesn't have.
FINGER_JOINTS = ("metacarpal", "proximal", "medial", "distal", "tip")


@dataclass
class Joint:
    frame: str
    parent: int  # index into HandModel.joints, -1 for the root
    rest_position: Vec3
    rest_rotation: Quat


@dataclass
class HandModel:
    """A solved hand's skeleton and where each joint's rotation sits on the wire.

    Built once per device from its stream definition; `pose()` then decodes any
    number of data frames against it.
    """

    device_id: int
    side: str  # "left" or "right"
    group_id: int
    joints: list[Joint]
    index_of: dict[str, int]
    # Joint index -> (offset in floats within the group payload, value count).
    # A TRANSFORM stream (the root) carries position+quaternion; ORIENTATION
    # streams carry the quaternion alone.
    slots: dict[int, tuple[int, int]]
    fingers: list[list[int]] = field(default_factory=list)
    wrist: int = -1

    @property
    def mirror(self) -> float:
        """-1 for a left hand: it is the mirror image of the authored right."""
        return -1.0 if self.side == "left" else 1.0


def is_solved_hand(definition: dict[str, Any]) -> bool:
    return str(definition.get("device_type", "")).lower() == "solved_hand"


def _rest_transforms(definition: dict[str, Any]) -> dict[str, tuple[Vec3, Quat, str]]:
    """Rest pose per frame: `(local position, local rotation, parent frame)`.

    The solver publishes it as FLOAT[7] TRANSFORM entries in `static_data`, each
    relative to its parent joint. The root's entry has no `reference_frame`.
    """
    rest: dict[str, tuple[Vec3, Quat, str]] = {}
    for entry in definition.get("static_data", []):
        if entry.get("measure_type") != "TRANSFORM":
            continue
        value = entry.get("value") or []
        if len(value) < 7:
            continue
        target = entry.get("target_frame", "")
        parent = entry.get("reference_frame") or ""
        if parent == target:
            parent = ""
        position = (float(value[0]), float(value[1]), float(value[2]))
        rotation = (float(value[3]), float(value[4]), float(value[5]), float(value[6]))
        rest[target] = (position, rotation, parent)
    return rest


def _finger_chain(model_frames: set[str], side: str, slug: str) -> list[str]:
    chain = [f"{side}_{slug}_{joint}" for joint in FINGER_JOINTS]
    return [frame for frame in chain if frame in model_frames]


def build_hand_model(definition: dict[str, Any], layout: Any) -> HandModel | None:
    """Build a `HandModel` from a solved-hand definition, or None if it isn't one.

    `layout` is the `StreamLayout` that `rgmp_client` derived from the same
    definition; only its group/slot bookkeeping is used.
    """
    if not is_solved_hand(definition):
        return None

    side = str((definition.get("device_info") or {}).get("hand", "left")).lower()
    if side not in ("left", "right"):
        side = "left"

    rest = _rest_transforms(definition)

    # The joints_local group: per-joint rotations relative to the parent joint.
    group_id = -1
    for gid, group in enumerate(definition.get("groups", [])):
        if group.get("name") == "joints_local":
            group_id = gid
            break
    if group_id < 0:
        return None

    # Walk the group's streams in wire order, recording where each joint's
    # quaternion starts. Offsets are in floats, not bytes: every stream in this
    # group is FLOAT[4] or FLOAT[7].
    slots_by_frame: dict[str, tuple[int, int]] = {}
    parents: dict[str, str] = {}
    float_offset = 0
    for entry in definition["groups"][group_id].get("streams", []):
        count = 7 if entry.get("data_type") == "FLOAT[7]" else 4
        target = entry.get("target_frame", "")
        if entry.get("measure_type") in ("ORIENTATION", "TRANSFORM"):
            slots_by_frame[target] = (float_offset, count)
            reference = entry.get("reference_frame") or ""
            # The root's rotation is given against a world frame, not a parent.
            parents[target] = "" if reference.startswith("LTP_") else reference
        float_offset += count

    # Parent links: prefer the rest pose (it is the model's own hierarchy) and
    # fall back to the streams' reference frames.
    for frame, (_, _, parent) in rest.items():
        if parent or frame not in parents:
            parents[frame] = parent

    if not slots_by_frame:
        return None

    # Topological order, parents first, so forward kinematics is a single pass.
    ordered: list[str] = []
    seen: set[str] = set()

    def visit(frame: str) -> None:
        if frame in seen or frame not in slots_by_frame:
            return
        seen.add(frame)
        parent = parents.get(frame, "")
        if parent:
            visit(parent)
        ordered.append(frame)

    for frame in slots_by_frame:
        visit(frame)

    index_of = {frame: i for i, frame in enumerate(ordered)}
    joints: list[Joint] = []
    for frame in ordered:
        position, rotation, _ = rest.get(frame, ((0.0, 0.0, 0.0), QUAT_IDENTITY, ""))
        parent = parents.get(frame, "")
        joints.append(
            Joint(
                frame=frame,
                parent=index_of.get(parent, -1),
                rest_position=position,
                rest_rotation=rotation,
            )
        )

    model = HandModel(
        device_id=int(definition["device_id"]),
        side=side,
        group_id=group_id,
        joints=joints,
        index_of=index_of,
        slots={index_of[f]: slots_by_frame[f] for f in ordered},
    )
    model.wrist = index_of.get(f"{side}_hand", -1)
    frame_set = set(ordered)
    model.fingers = [
        [index_of[f] for f in _finger_chain(frame_set, side, slug)]
        for slug in FINGER_SLUGS
    ]
    return model


# ── Derived per-frame values ──────────────────────────────────────────────────


@dataclass
class FingerState:
    name: str
    #: Flexion at each joint along the finger, base first, in degrees:
    #: MCP, PIP, DIP for the fingers; MCP and IP for the thumb. Positive is
    #: flexion (curling toward the palm); negative is hyperextension, the joint
    #: bending back past straight.
    joint_angles: list[float]
    #: Sum of `joint_angles`: total curl from straight, in degrees. Goes
    #: negative for a hand bent back past flat, so it stays monotonic across
    #: the full travel instead of turning back on itself at straight.
    curl_deg: float
    #: `curl_deg` mapped through the calibration to 0 (open) .. 1 (closed).
    closure: float
    #: Sideways deviation of the finger from the palm's forward direction, in
    #: degrees; positive is toward the thumb.
    spread_deg: float


@dataclass
class ThumbState:
    """The thumb's carpometacarpal (CMC) joint -- the base of the thumb.

    Not covered by `FingerState`: the thumb's adduction happens *at* the
    metacarpal, and `FingerState.curl_deg` starts measuring from the metacarpal
    bone, so CMC motion swings the whole chain without changing the curl.

    Both angles are measured against the index metacarpal rather than against a
    rest pose, so neither needs calibration to be meaningful.
    """

    #: In the plane of the palm: positive is adduction, the thumb closing toward
    #: the index finger. Zeroed on `THUMB_NEUTRAL_SPAN_DEG`, so a relaxed hand
    #: reads near 0 and spreading the thumb goes negative.
    adduction_deg: float
    #: `span_deg` mapped through the calibration to 0 (thumb spread wide) ..
    #: 1 (thumb closed against the index). The counterpart to
    #: `FingerState.closure`, for the thumb's sideways axis.
    adduction: float
    #: Out of the plane of the palm: positive is palmar abduction, the thumb
    #: lifting off the palm toward opposition.
    palmar_abduction_deg: float
    #: The raw measurement behind `adduction_deg`: the in-plane angle between
    #: the thumb and index metacarpals. Falls through zero if the thumb crosses
    #: the index, so that `adduction_deg` stays monotonic across full travel.
    span_deg: float


@dataclass
class WristState:
    #: Positive flexes the palm toward the inner forearm.
    flexion_deg: float
    #: Positive is adduction, i.e. ulnar (little-finger side) deviation.
    adduction_deg: float
    #: Positive is pronation (palm turning down).
    rotation_deg: float


@dataclass
class HandPose:
    device_id: int
    side: str
    timestamp_us: int
    wrist: WristState
    thumb: ThumbState
    fingers: list[FingerState]

    def as_dict(self) -> dict[str, Any]:
        """Plain-data form, for `--json` output or feeding a controller."""
        return {
            "device_id": self.device_id,
            "hand": self.side,
            "ts_us": self.timestamp_us,
            "wrist": {
                "flexion_deg": round(self.wrist.flexion_deg, 2),
                "adduction_deg": round(self.wrist.adduction_deg, 2),
                "rotation_deg": round(self.wrist.rotation_deg, 2),
            },
            "thumb_cmc": {
                "adduction": round(self.thumb.adduction, 4),
                "adduction_deg": round(self.thumb.adduction_deg, 2),
                "palmar_abduction_deg": round(self.thumb.palmar_abduction_deg, 2),
                "span_deg": round(self.thumb.span_deg, 2),
            },
            "fingers": {
                f.name: {
                    "closure": round(f.closure, 4),
                    "curl_deg": round(f.curl_deg, 2),
                    "spread_deg": round(f.spread_deg, 2),
                    "joints_deg": [round(a, 2) for a in f.joint_angles],
                }
                for f in self.fingers
            },
        }


def _normalize(value: float, lo: float, hi: float) -> float:
    """Where `value` falls in [lo, hi], clamped to 0..1. `hi` may be below `lo`."""
    if abs(hi - lo) < 1e-6:
        return 0.0
    return max(0.0, min(1.0, (value - lo) / (hi - lo)))


@dataclass
class Calibration:
    """Curl in degrees at the open and closed extremes, per finger.

    The defaults are rough anatomical spans. They are meant to be replaced by a
    measured pair -- `rgmp_client_rh8d.py --calibrate` captures one -- because
    the usable range varies with hand size and how the glove sits.
    """

    #: Thumb-to-index span, in degrees, at the two ends of the thumb's sideways
    #: travel: spread wide, then closed against the index. Unlike the finger
    #: entries these count *down*, since the span shrinks as the thumb adducts.
    thumb_open_span_deg: float = 60.0
    thumb_closed_span_deg: float = 15.0

    open_deg: dict[str, float] = field(
        default_factory=lambda: {name: 0.0 for name in FINGER_NAMES}
    )
    closed_deg: dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 90.0,
            "index": 200.0,
            "middle": 200.0,
            "ring": 200.0,
            "little": 200.0,
        }
    )

    def normalize(self, finger: str, curl_deg: float) -> float:
        return _normalize(
            curl_deg, self.open_deg.get(finger, 0.0), self.closed_deg.get(finger, 200.0)
        )

    def normalize_thumb(self, span_deg: float) -> float:
        """Thumb span to 0 (spread) .. 1 (adducted).

        The endpoints run downward -- the span *shrinks* as the thumb closes --
        which `_normalize` handles; it does not require hi > lo.
        """
        return _normalize(
            span_deg, self.thumb_open_span_deg, self.thumb_closed_span_deg
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "open_deg": self.open_deg,
            "closed_deg": self.closed_deg,
            "thumb_open_span_deg": self.thumb_open_span_deg,
            "thumb_closed_span_deg": self.thumb_closed_span_deg,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Calibration":
        calibration = cls()
        calibration.open_deg.update(
            {k: float(v) for k, v in (data.get("open_deg") or {}).items()}
        )
        calibration.closed_deg.update(
            {k: float(v) for k, v in (data.get("closed_deg") or {}).items()}
        )
        for name in ("thumb_open_span_deg", "thumb_closed_span_deg"):
            if data.get(name) is not None:
                setattr(calibration, name, float(data[name]))
        return calibration


def _forward_kinematics(
    model: HandModel, payload_floats: list[float]
) -> tuple[list[Vec3], list[Quat]]:
    """Joint positions and orientations in forearm space, parents before children.

    The root is pinned at the origin with identity rotation: the derived angles
    are all joint-relative, so where the arm is in the world does not matter.
    """
    positions: list[Vec3] = [(0.0, 0.0, 0.0)] * len(model.joints)
    rotations: list[Quat] = [QUAT_IDENTITY] * len(model.joints)

    for i, joint in enumerate(model.joints):
        offset, count = model.slots[i]
        if count == 7:  # TRANSFORM: position then quaternion
            local_rotation = (
                payload_floats[offset + 3],
                payload_floats[offset + 4],
                payload_floats[offset + 5],
                payload_floats[offset + 6],
            )
        else:
            local_rotation = (
                payload_floats[offset],
                payload_floats[offset + 1],
                payload_floats[offset + 2],
                payload_floats[offset + 3],
            )

        if joint.parent < 0:
            # The root's streamed pose is world-space; drop it and work in
            # forearm space instead.
            positions[i] = (0.0, 0.0, 0.0)
            rotations[i] = QUAT_IDENTITY
            continue

        parent_rotation = rotations[joint.parent]
        positions[i] = v_add(
            positions[joint.parent], q_rotate(joint.rest_position, parent_rotation)
        )
        rotations[i] = q_normalize(q_mul(parent_rotation, q_normalize(local_rotation)))

    return positions, rotations


def _finger_state(
    model: HandModel,
    name: str,
    chain: list[int],
    positions: list[Vec3],
    rotations: list[Quat],
    calibration: Calibration,
) -> FingerState:
    """Curl and spread for one finger, from the bone directions FK produced.

    Each joint's flexion is the angle between the bones it joins, measured in
    the joint's flexion plane and signed about its flexion axis: sideways
    deviation is projected out rather than counted as flexion, and bending back
    past straight reads negative rather than as more curl.
    """
    bones: list[Vec3] = []
    for a, b in zip(chain, chain[1:]):
        bones.append(v_sub(positions[b], positions[a]))

    joint_angles: list[float] = []
    for k in range(len(bones) - 1):
        # The flexion axis of the joint between bones[k] and bones[k+1], taken
        # from the parent bone's orientation so the axis does not itself swing
        # with the joint it is measuring.
        axis = q_rotate(FLEXION_AXIS, rotations[chain[k]])
        # Flatten both bones into the joint's flexion plane first. The bare
        # angle between them is a 3D angle, so a joint that swings sideways
        # reports that swing as flexion -- on the thumb's MCP, which abducts as
        # much as it flexes, that is enough to make the flexion channel follow
        # adduction around. Projecting keeps only the component that is
        # actually rotation about the flexion axis.
        u = project_onto_plane(bones[k], axis)
        v = project_onto_plane(bones[k + 1], axis)
        joint_angles.append(signed_angle(u, v, axis) * model.mirror)

    if name == "thumb":
        curl_deg = sum(
            joint_angles[i] for i in THUMB_CURL_JOINTS if i < len(joint_angles)
        )
    else:
        curl_deg = sum(joint_angles)

    # Spread: how far the first phalanx swings sideways within the palm plane,
    # measured against the palm's forward direction in the hand's own frame.
    spread_deg = 0.0
    if len(bones) >= 2 and model.wrist >= 0:
        hand_rotation = rotations[model.wrist]
        local = q_rotate(bones[1], q_conjugate(hand_rotation))
        flat = (local[0], local[1], 0.0)  # drop the flexion component
        spread_deg = signed_angle(BONE_AXIS, flat, PALM_NORMAL) * model.mirror

    return FingerState(
        name=name,
        joint_angles=joint_angles,
        curl_deg=curl_deg,
        closure=calibration.normalize(name, curl_deg),
        spread_deg=spread_deg,
    )


def _thumb_state(
    model: HandModel,
    positions: list[Vec3],
    rotations: list[Quat],
    calibration: Calibration,
) -> ThumbState:
    """The thumb's CMC angles, from where its metacarpal points in hand space.

    The thumb metacarpal's direction relative to the palm carries both CMC
    degrees of freedom; splitting it into its in-palm-plane and out-of-plane
    parts separates adduction from palmar abduction. The index metacarpal is
    the in-plane reference, which is also the standard clinical one.
    """
    empty = ThumbState(0.0, 0.0, 0.0, 0.0)
    if model.wrist < 0 or len(model.fingers) < 2:
        return empty
    thumb_chain, index_chain = model.fingers[0], model.fingers[1]
    if len(thumb_chain) < 2 or len(index_chain) < 2:
        return empty

    into_hand = q_conjugate(rotations[model.wrist])
    thumb = v_normalize(
        q_rotate(v_sub(positions[thumb_chain[1]], positions[thumb_chain[0]]), into_hand)
    )
    index = v_normalize(
        q_rotate(v_sub(positions[index_chain[1]], positions[index_chain[0]]), into_hand)
    )
    if thumb is None or index is None:
        return empty

    # Flatten both into the palm plane for the in-plane angle; what the thumb
    # loses in that projection is its palmar abduction.
    thumb_flat = (thumb[0], thumb[1], 0.0)
    index_flat = (index[0], index[1], 0.0)
    # Signed, not the bare angle: if the thumb crosses the index metacarpal the
    # span has to keep falling through zero, or adduction turns back on itself
    # at the far end of its travel.
    span_deg = signed_angle(index_flat, thumb_flat, PALM_NORMAL) * model.mirror

    out_of_plane = max(-1.0, min(1.0, v_dot(thumb, PALM_NORMAL)))
    palmar_abduction_deg = math.degrees(math.asin(out_of_plane)) * model.mirror

    return ThumbState(
        adduction_deg=THUMB_NEUTRAL_SPAN_DEG - span_deg,
        adduction=calibration.normalize_thumb(span_deg),
        palmar_abduction_deg=palmar_abduction_deg,
        span_deg=span_deg,
    )


def _wrist_state(model: HandModel, payload_floats: list[float]) -> WristState:
    """The wrist's three degrees of freedom.

    The hand joint's streamed rotation is already relative to the forearm, i.e.
    it *is* the wrist joint, so no forward kinematics is involved. The bend and
    the twist are separated first (a wrist that is both flexed and pronated
    would otherwise cross-contaminate the two), then the bend is split into its
    flexion and deviation components.
    """
    if model.wrist < 0:
        return WristState(0.0, 0.0, 0.0)

    offset, count = model.slots[model.wrist]
    base = offset + 3 if count == 7 else offset
    wrist_rotation = q_normalize(
        (
            payload_floats[base],
            payload_floats[base + 1],
            payload_floats[base + 2],
            payload_floats[base + 3],
        )
    )

    swing, _ = swing_twist(wrist_rotation, TWIST_AXIS)
    bend = rotation_vector(swing)
    rotation_deg = twist_angle(wrist_rotation, TWIST_AXIS)

    # Left hands mirror the authored right-hand convention on the two axes that
    # reverse under reflection; flexion (about X) is the same on both sides.
    mirror = model.mirror
    sx, sy, sz = WRIST_SIGNS
    return WristState(
        flexion_deg=bend[0] * sx,
        adduction_deg=bend[2] * sz * mirror,
        rotation_deg=rotation_deg * sy * mirror,
    )


def pose_from_frame(
    model: HandModel,
    timestamp_us: int,
    payload_floats: list[float],
    calibration: Calibration | None = None,
) -> HandPose:
    """Derive one frame's wrist and finger values from a `joints_local` payload."""
    calibration = calibration or Calibration()
    positions, rotations = _forward_kinematics(model, payload_floats)
    fingers = [
        _finger_state(model, name, chain, positions, rotations, calibration)
        for name, chain in zip(FINGER_NAMES, model.fingers)
        if len(chain) >= 3
    ]
    return HandPose(
        device_id=model.device_id,
        side=model.side,
        timestamp_us=timestamp_us,
        wrist=_wrist_state(model, payload_floats),
        thumb=_thumb_state(model, positions, rotations, calibration),
        fingers=fingers,
    )
