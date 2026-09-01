"""Glove channels to RH8D Dynamixel positions.

The last step of the chain: the glove side produces eight normalized channels,
and this turns them into the tick values the hand's servos take. Kept free of
Qt and of any transport so the demo tool, a ROS node, or anything else can
share one definition of what a channel means -- the joint order and the
direction of travel are exactly the kind of thing that goes wrong twice if it
is written down twice.

The channels
------------
Five finger channels, each 0..1, already calibrated by whoever produced them:

    thumb_abduction   0 = thumb spread wide, 1 = closed onto the palm
    thumb_flexion     0 = extended,          1 = curled
    index_flexion     0 = open,              1 = closed
    middle_flexion
    ring_flexion      drives the ring and little finger together, one motor

Three wrist channels, in degrees rather than normalized, because they are
signed and centred on zero rather than running from an open to a closed end:

    wrist_rotation    +/- 90, positive is pronation (palm turning down)
    wrist_flexion     +/- 45, positive flexes the palm toward the forearm
    wrist_abduction   +/- 45, positive is ulnar (little-finger side)

How a channel becomes ticks
---------------------------
The servos take 0..4095 over their full travel, so both kinds of channel end
up as a linear map onto that span, differing only in what the endpoints mean:

    finger:  0..1 maps straight onto 0..4095, clamped. 0.5 -> 2048.
    wrist:   the angle is clamped to its range, then that range maps onto
             0..4095. The ranges are symmetric, so 0 degrees -> 2048 and the
             joint sits centred when the glove is neutral.

A joint the glove has no channel for is parked at the midpoint rather than at
zero: 2048 is a centred wrist, where 0 would be a hard stop.

Joint order comes from the config (`config/RH8D.yaml`), not from this module.
`channel_order()` turns the config's joint names into channel keys, so the
returned tick list always lines up with the config's own ordering.
"""

from __future__ import annotations

#: Full travel of a Dynamixel servo, in ticks.
DYNAMIXEL_MIN = 0
DYNAMIXEL_MAX = 4095

#: Where an unmapped joint is parked: centred, not at a hard stop.
DYNAMIXEL_MID = (DYNAMIXEL_MIN + DYNAMIXEL_MAX) // 2

#: The angle each wrist channel spans, mapped onto the servo's full travel.
WRIST_ROTATION_RANGE = (-90.0, 90.0)
WRIST_FLEXION_RANGE = (-45.0, 45.0)
WRIST_ABDUCTION_RANGE = (-45.0, 45.0)

#: Where the thumb's flexion travel begins, as a function of how adducted the
#: thumb is: at adduction 0 it begins at ..._ADD_0, at adduction 1 at
#: ..._ADD_1, and anything below that start reads as no flexion. With _ADD_1
#: above _ADD_0 an adducted thumb gives up the bottom of its flexion range,
#: which couples the two axes -- a thumb that only adducts still drives the
#: flexion motor. Both equal (the default) disables the coupling.
THUMB_FLEX_START_AT_ADD_0 = 0.0
THUMB_FLEX_START_AT_ADD_1 = 0.0

#: The wrist channels, which arrive in degrees and are clamped to a range.
#: Everything else is a finger channel running 0..1.
WRIST_CHANNEL_RANGES = {
    "wrist_rotation": WRIST_ROTATION_RANGE,
    "wrist_flexion": WRIST_FLEXION_RANGE,
    "wrist_abduction": WRIST_ABDUCTION_RANGE,
}


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def angle_to_dynamixel(angle_deg: float, deg_range: tuple[float, float]) -> int:
    """A signed angle onto the servo's travel; the range's midpoint lands mid."""
    lo, hi = deg_range
    if hi <= lo:
        return DYNAMIXEL_MID
    clamped = max(lo, min(hi, angle_deg))
    t = (clamped - lo) / (hi - lo)
    return int(round(DYNAMIXEL_MIN + t * (DYNAMIXEL_MAX - DYNAMIXEL_MIN)))


def finger_to_dynamixel(normalized: float) -> int:
    """A 0..1 finger channel onto the servo's travel."""
    return int(
        round(DYNAMIXEL_MIN + clamp01(normalized) * (DYNAMIXEL_MAX - DYNAMIXEL_MIN))
    )


def classify_joint(name: str) -> str | None:
    """The glove channel that drives a config joint name, or None.

    Two of these are crossed on purpose: the config's "Wrist Adduction" motor
    is driven by the wrist_flexion channel and "Wrist Flexation" by
    wrist_abduction. That is how the hand is built, not a mistake -- see the
    comments inline.
    """
    lower = name.lower()
    if "wrist" in lower and "rotation" in lower:
        return "wrist_rotation"
    if "wrist" in lower and "adduction" in lower:
        # config "Wrist Adduction" is driven by the glove's flexion channel
        return "wrist_flexion"
    if "wrist" in lower and ("flexation" in lower or "flexion" in lower):
        # config "Wrist Flexation" is driven by the glove's abduction channel
        return "wrist_abduction"
    if "thumb" in lower and "adduction" in lower:
        return "thumb_abduction"
    if "thumb" in lower and ("flexation" in lower or "flexion" in lower):
        return "thumb_flexion"
    if "index" in lower:
        return "index_flexion"
    if "middle" in lower:
        return "middle_flexion"
    if "ring" in lower or "little" in lower:
        return "ring_flexion"
    return None


def channel_order(joint_mapping) -> list[str | None]:
    """Channel key per joint, in the config's own joint order.

    Takes the joint mapping from the config (a dict of joint name -> servo id,
    or any iterable of joint names); only the names and their order matter.
    """
    return [classify_joint(name) for name in joint_mapping]


def apply_thumb_gate(thumb_abduction: float, thumb_flexion: float) -> float:
    """Rescale thumb flexion by how adducted the thumb is.

    A pass-through unless THUMB_FLEX_START_AT_ADD_0/1 differ; see their note.
    """
    start = clamp01(
        THUMB_FLEX_START_AT_ADD_0
        + thumb_abduction * (THUMB_FLEX_START_AT_ADD_1 - THUMB_FLEX_START_AT_ADD_0)
    )
    if thumb_flexion <= start:
        return 0.0
    return clamp01((thumb_flexion - start) / (1.0 - start))


def channel_to_dynamixel(channel: str | None, value: float) -> int:
    """One channel's value as a servo position."""
    if channel is None:
        return DYNAMIXEL_MID
    deg_range = WRIST_CHANNEL_RANGES.get(channel)
    if deg_range is not None:
        return angle_to_dynamixel(value, deg_range)
    return finger_to_dynamixel(value)


def joint_positions(glove: dict[str, float], order: list[str | None]) -> list[int]:
    """Servo positions for every joint in `order`.

    `glove` maps channel key to value: the five finger channels as 0..1, the
    three wrist channels in degrees. A channel `order` names but `glove` omits
    is parked at the midpoint, same as an unmapped joint.
    """
    return [
        DYNAMIXEL_MID
        if channel is not None and channel not in glove
        else channel_to_dynamixel(channel, glove.get(channel, 0.0))
        for channel in order
    ]
