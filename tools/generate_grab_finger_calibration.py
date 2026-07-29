#!/usr/bin/env python3
"""
Generate ROCK's baked grab-finger calibration table from hFRIK's authored hand
pose source and original FRIK dependency math conventions.

Runtime code must not parse hFRIK source or probe FRIK to build these tables.
This tool is the offline boundary: it reads the local source authorities,
rebuilds hFRIK's authored local transforms for ROCK grab curl samples, samples
tip/outer/inner probe points, and writes a constexpr table consumed by gameplay.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import math
import pathlib
import re
import sys
from typing import Iterable


SAMPLE_COUNT = 201
GENERATOR_VERSION = 4
# hFRIK slerps joint flex in [-1, 2]: t in (1, 2] extrapolates past the
# authored open pose (hyper-extension). The bake samples the full over-open
# range so the runtime sweep can plant fingers on surfaces thicker than the
# default open span; the zero-angle reference stays the value-1.0 pose, so
# over-open rows carry NEGATIVE angles and every [0, 1] row is unchanged
# in meaning.
OVER_OPEN_MAX = 2.0
OUTPUT_RELATIVE = pathlib.Path("src/physics-interaction/grab/GeneratedGrabFingerCalibration.h")
THUMB_LANES = (
    {
        "name": "Wrap",
        "curve_source": "HandThumb",
        "normal_blend": 0.0,
        "local_correction_strength": 0.0,
        "surface_thickness_scale": 0.060,
        "apply_authored_normal_sign": True,
    },
    {
        "name": "Opposition",
        "curve_source": "HandThumb",
        "normal_blend": 1.0,
        "local_correction_strength": 1.0,
        "surface_thickness_scale": 0.066,
        "apply_authored_normal_sign": False,
    },
    {
        "name": "SidePad",
        "curve_source": "SidePad",
        "normal_blend": 0.48,
        "local_correction_strength": 0.82,
        "surface_thickness_scale": 0.078,
        "apply_authored_normal_sign": False,
    },
)


@dataclasses.dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def scale(self, value: float) -> "Vec3":
        return Vec3(self.x * value, self.y * value, self.z * value)


@dataclasses.dataclass(frozen=True)
class Transform:
    rotate: list[list[float]]
    translate: Vec3


@dataclasses.dataclass(frozen=True)
class HandBonePoseData:
    bone_name: str
    closed_rotation: list[float]
    open_rotation: list[float]
    open_translation: Vec3
    open_translation_power_armor: Vec3


@dataclasses.dataclass(frozen=True)
class RawSample:
    open_value: float
    angle: float
    reach_scale: float


def repo_root_from_script() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def workspace_root_from_repo(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root.parents[1]


def read_text(path: pathlib.Path) -> str:
    return path.read_text(encoding="utf-8")


def sha256_for_sources(sources: Iterable[tuple[str, pathlib.Path]]) -> str:
    digest = hashlib.sha256()
    for label, path in sources:
        digest.update(label.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def parse_float_list(raw: str) -> list[float]:
    values: list[float] = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        values.append(float(token.rstrip("Ff")))
    return values


def parse_vec3(raw: str) -> Vec3:
    values = parse_float_list(raw)
    if len(values) != 3:
        raise ValueError(f"expected NiPoint3 with 3 values, got {len(values)}")
    return Vec3(values[0], values[1], values[2])


def parse_hand_bone_pose_data(path: pathlib.Path) -> list[HandBonePoseData]:
    text = read_text(path)
    pattern = re.compile(
        r'HandBonePoseData\{\s*\.boneName = "([^"]+)",\s*'
        r"\.closedRotation = RotationData\{([^}]*)\},\s*"
        r"\.openRotation = RotationData\{([^}]*)\},\s*"
        r"\.openTranslation = RE::NiPoint3\(([^)]*)\),\s*"
        r"\.openTranslationInPowerArmor = RE::NiPoint3\(([^)]*)\) \}",
        re.MULTILINE | re.DOTALL,
    )
    result: list[HandBonePoseData] = []
    for match in pattern.finditer(text):
        closed = parse_float_list(match.group(2))
        opened = parse_float_list(match.group(3))
        if len(closed) != 12 or len(opened) != 12:
            raise ValueError(f"{match.group(1)} rotation data must contain 12 values")
        result.append(
            HandBonePoseData(
                bone_name=match.group(1),
                closed_rotation=closed,
                open_rotation=opened,
                open_translation=parse_vec3(match.group(4)),
                open_translation_power_armor=parse_vec3(match.group(5)),
            )
        )
    if len(result) != 30:
        raise ValueError(f"expected 30 hFRIK hand bone pose entries, got {len(result)}")
    return result


def matrix_from_rotation_data(values: list[float]) -> list[list[float]]:
    return [
        [values[0], values[1], values[2]],
        [values[4], values[5], values[6]],
        [values[8], values[9], values[10]],
    ]


def matrix_identity() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def matrix_multiply(lhs: list[list[float]], rhs: list[list[float]]) -> list[list[float]]:
    return [
        [sum(lhs[row][k] * rhs[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def matrix_transpose_vector(matrix: list[list[float]], vector: Vec3) -> Vec3:
    return Vec3(
        matrix[0][0] * vector.x + matrix[1][0] * vector.y + matrix[2][0] * vector.z,
        matrix[0][1] * vector.x + matrix[1][1] * vector.y + matrix[2][1] * vector.z,
        matrix[0][2] * vector.x + matrix[1][2] * vector.y + matrix[2][2] * vector.z,
    )


def matrix_from_euler(heading: float, roll: float, attitude: float) -> list[list[float]]:
    sin_x = math.sin(heading)
    cos_x = math.cos(heading)
    sin_y = math.sin(roll)
    cos_y = math.cos(roll)
    sin_z = math.sin(attitude)
    cos_z = math.cos(attitude)
    return [
        [cos_y * cos_z, -cos_y * sin_z, sin_y],
        [sin_x * sin_y * cos_z + sin_z * cos_x, cos_x * cos_z - sin_x * sin_y * sin_z, -sin_x * cos_y],
        [sin_x * sin_z - cos_x * sin_y * cos_z, cos_x * sin_y * sin_z + sin_x * cos_z, cos_x * cos_y],
    ]


@dataclasses.dataclass(frozen=True)
class Quaternion:
    w: float
    x: float
    y: float
    z: float


def quaternion_from_matrix(rot: list[list[float]]) -> Quaternion:
    w = math.sqrt(max(0.0, 1.0 + rot[0][0] + rot[1][1] + rot[2][2])) / 2.0
    x_raw = math.sqrt(max(0.0, 1.0 + rot[0][0] - rot[1][1] - rot[2][2])) / 2.0
    y_raw = math.sqrt(max(0.0, 1.0 - rot[0][0] + rot[1][1] - rot[2][2])) / 2.0
    z_raw = math.sqrt(max(0.0, 1.0 - rot[0][0] - rot[1][1] + rot[2][2])) / 2.0
    return Quaternion(
        w,
        math.copysign(x_raw, rot[2][1] - rot[1][2]),
        math.copysign(y_raw, rot[0][2] - rot[2][0]),
        math.copysign(z_raw, rot[1][0] - rot[0][1]),
    )


def quaternion_dot(lhs: Quaternion, rhs: Quaternion) -> float:
    return lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z


def quaternion_slerp(source: Quaternion, interp: float, target: Quaternion) -> Quaternion:
    save = source
    dot = quaternion_dot(source, target)
    if dot < 0.0:
        source = Quaternion(-source.w, -source.x, -source.y, -source.z)
        dot = -dot
    if dot > 0.999995:
        return save
    dot = max(-1.0, min(1.0, dot))
    theta0 = math.acos(dot)
    theta = theta0 * interp
    sin_theta = math.sin(theta)
    sin_theta0 = math.sin(theta0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return Quaternion(
        s0 * source.w + s1 * target.w,
        s0 * source.x + s1 * target.x,
        s0 * source.y + s1 * target.y,
        s0 * source.z + s1 * target.z,
    )


def quaternion_to_matrix(q: Quaternion) -> list[list[float]]:
    return [
        [2.0 * (q.w * q.w + q.x * q.x) - 1.0, 2.0 * (q.x * q.y - q.w * q.z), 2.0 * (q.x * q.z + q.w * q.y)],
        [2.0 * (q.x * q.y + q.w * q.z), 2.0 * (q.w * q.w + q.y * q.y) - 1.0, 2.0 * (q.y * q.z - q.w * q.x)],
        [2.0 * (q.x * q.z - q.w * q.y), 2.0 * (q.y * q.z + q.w * q.x), 2.0 * (q.w * q.w + q.z * q.z) - 1.0],
    ]


def dot(lhs: Vec3, rhs: Vec3) -> float:
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z


def cross(lhs: Vec3, rhs: Vec3) -> Vec3:
    return Vec3(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    )


def length(value: Vec3) -> float:
    return math.sqrt(dot(value, value))


def normalize(value: Vec3, fallback: Vec3 = Vec3(1.0, 0.0, 0.0)) -> Vec3:
    magnitude = length(value)
    if not math.isfinite(magnitude) or magnitude <= 0.000001:
        fallback_length = length(fallback)
        if not math.isfinite(fallback_length) or fallback_length <= 0.000001:
            return Vec3(1.0, 0.0, 0.0)
        return fallback.scale(1.0 / fallback_length)
    return value.scale(1.0 / magnitude)


def signed_angle(vector: Vec3, zero: Vec3, normal: Vec3) -> float:
    v = normalize(vector)
    z = normalize(zero)
    n = normalize(normal, Vec3(0.0, 0.0, -1.0))
    clamped = max(-1.0, min(1.0, dot(v, z)))
    angle = math.acos(clamped)
    if dot(n, cross(z, v)) < 0.0:
        angle *= -1.0
    return angle


def bone_to_finger_index(name: str) -> int:
    return ord(name[-2]) - ord("1")


def bone_to_flat_index(name: str) -> int:
    return bone_to_finger_index(name) * 3 + (ord(name[-1]) - ord("1"))


def expand_rock_grab_open_value(open_value: float, finger: int) -> tuple[float, float, float]:
    # Mirrors ROCK's expandFingerCurlsToJointValues: every finger may
    # hyper-extend to OVER_OPEN_MAX (hFRIK slerp accepts flex up to 2); the
    # proximal/distal biases vanish past 1.0, so over-open joints move
    # uniformly.
    value = max(0.0, min(open_value, OVER_OPEN_MAX))
    closed = 1.0 - min(value, 1.0)
    proximal_open_bias = 0.15 if finger == 0 else 0.25
    distal_close_bias = 0.10 if finger == 0 else 0.15
    return (
        max(0.0, min(OVER_OPEN_MAX, value + closed * proximal_open_bias)),
        value,
        max(0.0, min(OVER_OPEN_MAX, value - closed * distal_close_bias)),
    )


def build_local_transform(bone: HandBonePoseData, joint_values: list[float], in_power_armor: bool) -> Transform:
    open_transform = matrix_from_rotation_data(bone.open_rotation)
    closed_transform = matrix_from_rotation_data(bone.closed_rotation)
    q_open = quaternion_from_matrix(open_transform)
    q_closed = quaternion_from_matrix(closed_transform)
    flex = max(-1.0, min(2.0, joint_values[bone_to_flat_index(bone.bone_name)]))
    rotation = quaternion_to_matrix(quaternion_slerp(q_closed, flex, q_open))
    if bone.bone_name[-1] == "1":
        # Grab calibration uses neutral splay, but this keeps the builder aligned
        # with hFRIK's transform path if static probe poses gain splay later.
        splay = 0.0
        sign = -1.0 if bone.bone_name[0] == "L" else 1.0
        if splay != 0.0:
            rotation = matrix_multiply(matrix_from_euler(0.0, sign * splay, 0.0), rotation)
    translation = bone.open_translation_power_armor if in_power_armor else bone.open_translation
    return Transform(rotation, translation)


def build_world_transforms(bones: list[HandBonePoseData], is_left: bool, in_power_armor: bool, open_value: float) -> list[Transform]:
    joint_values: list[float] = []
    for finger in range(5):
        joint_values.extend(expand_rock_grab_open_value(open_value, finger))

    locals_by_index: list[Transform | None] = [None] * 15
    for bone in bones:
        if is_left != bone.bone_name.startswith("L"):
            continue
        locals_by_index[bone_to_flat_index(bone.bone_name)] = build_local_transform(bone, joint_values, in_power_armor)

    result: list[Transform] = []
    for finger in range(5):
        parent = Transform(matrix_identity(), Vec3(0.0, 0.0, 0.0))
        for segment in range(3):
            local = locals_by_index[finger * 3 + segment]
            if local is None:
                raise ValueError(f"missing local transform for finger={finger} segment={segment}")
            world_translate = parent.translate + matrix_transpose_vector(parent.rotate, local.translate)
            world_rotate = matrix_multiply(local.rotate, parent.rotate)
            parent = Transform(world_rotate, world_translate)
            result.append(parent)
    return result


def distal_open_length(bones: list[HandBonePoseData], is_left: bool, finger: int, in_power_armor: bool) -> float:
    target_suffix = f"Finger{finger + 1}3"
    for bone in bones:
        if is_left == bone.bone_name.startswith("L") and bone.bone_name.endswith(target_suffix):
            translation = bone.open_translation_power_armor if in_power_armor else bone.open_translation
            return max(0.0001, abs(translation.x))
    raise ValueError(f"missing distal bone length for finger {finger}")


def probe_offset(probe: str, distal_length: float, finger: int, is_left: bool, thumb_lane: str = "Wrap") -> Vec3:
    side = -1.0 if is_left else 1.0
    if probe == "Tip":
        if finger == 0 and thumb_lane == "SidePad":
            return Vec3(distal_length * 0.70, distal_length * 0.24, side * distal_length * 0.18)
        return Vec3(distal_length, 0.0, 0.0)
    if finger == 0:
        if thumb_lane == "SidePad":
            if probe == "Outer":
                return Vec3(distal_length * 0.76, distal_length * 0.26, side * distal_length * 0.22)
            return Vec3(distal_length * 0.54, distal_length * 0.18, side * distal_length * 0.08)
        if probe == "Outer":
            return Vec3(distal_length * 0.72, distal_length * 0.20, side * distal_length * 0.12)
        return Vec3(distal_length * 0.48, distal_length * 0.16, -side * distal_length * 0.12)
    if probe == "Outer":
        return Vec3(distal_length * 0.74, distal_length * 0.16, side * distal_length * 0.10)
    return Vec3(distal_length * 0.52, distal_length * 0.12, -side * distal_length * 0.10)


def transformed_probe_point(distal: Transform, offset: Vec3) -> Vec3:
    return distal.translate + matrix_transpose_vector(distal.rotate, offset)


def runtime_landmark_reference_length(world: list[Transform], finger: int) -> float:
    base = world[finger * 3].translate
    mid = world[finger * 3 + 1].translate
    distal = world[finger * 3 + 2].translate
    return max(0.0001, length(mid - base) + length(distal - mid))


def sample_raw_probe_curve(
    bones: list[HandBonePoseData],
    is_left: bool,
    in_power_armor: bool,
    finger: int,
    probe: str,
    thumb_lane: str = "Wrap",
) -> tuple[float, list[RawSample]]:
    open_world = build_world_transforms(bones, is_left, in_power_armor, 1.0)
    base_open = open_world[finger * 3].translate
    distal_length = distal_open_length(bones, is_left, finger, in_power_armor)
    distal_open = open_world[finger * 3 + 2].translate
    zero = normalize(distal_open - base_open)
    canonical_normal = Vec3(0.0, 0.0, -1.0)
    reference_length = runtime_landmark_reference_length(open_world, finger)

    closed_world = build_world_transforms(bones, is_left, in_power_armor, 0.0)
    closed_probe = transformed_probe_point(closed_world[finger * 3 + 2], probe_offset("Tip", distal_length, finger, is_left, thumb_lane))
    closed_angle = signed_angle(closed_probe - closed_world[finger * 3].translate, zero, canonical_normal)
    normal_sign = -1.0 if closed_angle < 0.0 else 1.0

    raw: list[RawSample] = []
    for index in range(SAMPLE_COUNT):
        open_value = OVER_OPEN_MAX * (1.0 - (index / (SAMPLE_COUNT - 1)))
        world = build_world_transforms(bones, is_left, in_power_armor, open_value)
        base = world[finger * 3].translate
        point = transformed_probe_point(world[finger * 3 + 2], probe_offset(probe, distal_length, finger, is_left, thumb_lane))
        angle = signed_angle(point - base, zero, canonical_normal) * normal_sign
        if not math.isfinite(angle):
            raise ValueError(f"non-finite angle for finger={finger} probe={probe} thumb_lane={thumb_lane}")
        # Over-open rows rotate away from the closing direction: their angle
        # relative to the value-1.0 zero is legitimately negative. Only rows
        # inside [0, 1] are floored at zero to absorb numeric noise around
        # the reference pose.
        clamped_angle = angle if open_value > 1.0 else max(0.0, angle)
        raw.append(RawSample(open_value, clamped_angle, length(point - base) / reference_length))
    return normal_sign, raw


def resample_curve(raw: list[RawSample]) -> list[RawSample]:
    ordered = sorted(raw, key=lambda item: (item.angle, -item.open_value))
    deduped: list[RawSample] = []
    for sample in ordered:
        if deduped and abs(sample.angle - deduped[-1].angle) <= 0.000001:
            previous = deduped[-1]
            deduped[-1] = RawSample(
                max(previous.open_value, sample.open_value),
                previous.angle,
                max(previous.reach_scale, sample.reach_scale),
            )
        else:
            deduped.append(sample)

    max_angle = max(sample.angle for sample in deduped)
    if max_angle <= 0.0001:
        raise ValueError("calibration curve has no usable curl angle")
    # Over-open rows extend the arc on the negative-angle side of the
    # value-1.0 zero; the resample grid spans the full baked arc.
    min_angle = min(sample.angle for sample in deduped)

    resampled: list[RawSample] = []
    cursor = 0
    for index in range(SAMPLE_COUNT):
        target_angle = min_angle + (max_angle - min_angle) * (index / (SAMPLE_COUNT - 1))
        while cursor + 1 < len(deduped) and deduped[cursor + 1].angle < target_angle:
            cursor += 1
        if cursor + 1 >= len(deduped):
            source = deduped[-1]
            resampled.append(RawSample(source.open_value, target_angle, source.reach_scale))
            continue
        a = deduped[cursor]
        b = deduped[cursor + 1]
        denom = b.angle - a.angle
        t = 0.0 if abs(denom) <= 0.000001 else max(0.0, min(1.0, (target_angle - a.angle) / denom))
        open_value = a.open_value + (b.open_value - a.open_value) * t
        reach_scale = a.reach_scale + (b.reach_scale - a.reach_scale) * t
        resampled.append(RawSample(max(0.0, min(OVER_OPEN_MAX, open_value)), target_angle, max(0.0001, reach_scale)))

    # ROCK lookup expects increasing angles and generally decreasing open values.
    last_open = OVER_OPEN_MAX
    monotonic: list[RawSample] = []
    for sample in resampled:
        open_value = min(last_open, sample.open_value)
        monotonic.append(RawSample(open_value, sample.angle, sample.reach_scale))
        last_open = open_value
    return monotonic


def f32(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}f"


def build_finger_curve(
    bones: list[HandBonePoseData],
    is_left: bool,
    in_power_armor: bool,
    finger: int,
    surface_thickness_scale: float,
    thumb_lane: str = "Wrap",
) -> dict:
    normal_sign, tip_raw = sample_raw_probe_curve(bones, is_left, in_power_armor, finger, "Tip", thumb_lane)
    tip_max_angle = max(sample.angle for sample in tip_raw)
    if tip_max_angle <= 0.0001:
        raise ValueError(f"finger {finger} thumb_lane={thumb_lane} tip calibration has no usable curl angle")
    probes = []
    for probe in ("Tip", "Outer", "Inner"):
        _, raw = sample_raw_probe_curve(bones, is_left, in_power_armor, finger, probe, thumb_lane)
        probe_max_angle = max(sample.angle for sample in raw)
        if probe != "Tip" and probe_max_angle < tip_max_angle * 0.25:
            # Some hFRIK power-armor auxiliary pad offsets primarily change
            # reach rather than angular curl. Keep their baked reach model,
            # but use the tip's stable authored angle parameter so runtime
            # lookup remains monotonic and the probe still participates.
            raw = [
                RawSample(aux.open_value, tip.angle, aux.reach_scale)
                for aux, tip in zip(raw, tip_raw)
            ]
        probes.append((probe, resample_curve(raw)))
    return {
        "normal_sign": normal_sign,
        "surface_thickness_scale": surface_thickness_scale,
        "probes": probes,
    }


def generate_profile(bones: list[HandBonePoseData], is_left: bool, in_power_armor: bool) -> dict:
    fingers = []
    for finger in range(5):
        fingers.append(
            build_finger_curve(
                bones,
                is_left,
                in_power_armor,
                finger,
                0.060 if finger == 0 else 0.050)
        )
    thumb_side_pad_curve = build_finger_curve(
        bones,
        is_left,
        in_power_armor,
        0,
        0.078,
        "SidePad")
    thumb_lanes = []
    for lane in THUMB_LANES:
        thumb_lanes.append(
            {
                "name": lane["name"],
                "curve_source": lane["curve_source"],
                "normal_blend": lane["normal_blend"],
                "local_correction_strength": lane["local_correction_strength"],
                "surface_thickness_scale": lane["surface_thickness_scale"],
                "apply_authored_normal_sign": lane["apply_authored_normal_sign"],
            }
        )
    return {
        "is_left": is_left,
        "in_power_armor": in_power_armor,
        "fingers": fingers,
        "thumb_lanes": thumb_lanes,
        "thumb_side_pad_curve": thumb_side_pad_curve,
    }


def generate_header(hfrik_root: pathlib.Path) -> str:
    source_files = [
        ("main_projects/hFRIK/src/skeleton/HandPose.cpp", hfrik_root / "src/skeleton/HandPose.cpp"),
        ("main_projects/hFRIK/src/skeleton/HandPoseData.cpp", hfrik_root / "src/skeleton/HandPoseData.cpp"),
    ]
    source_paths = [path for _, path in source_files]
    missing = [path for path in source_paths if not path.exists()]
    if missing:
        raise FileNotFoundError("missing calibration source files: " + ", ".join(str(path) for path in missing))
    bones = parse_hand_bone_pose_data(source_paths[1])
    source_hash = sha256_for_sources(source_files)
    profiles = [
        generate_profile(bones, is_left=False, in_power_armor=False),
        generate_profile(bones, is_left=True, in_power_armor=False),
        generate_profile(bones, is_left=False, in_power_armor=True),
        generate_profile(bones, is_left=True, in_power_armor=True),
    ]

    def append_baked_curve(lines: list[str], curve: dict, indent: str) -> None:
        lines.append(f"{indent}BakedGrabFingerCurve{{")
        lines.append(f"{indent}    .normalSign = {f32(curve['normal_sign'])},")
        lines.append(f"{indent}    .surfaceThicknessScale = {f32(curve['surface_thickness_scale'])},")
        lines.append(f"{indent}    .probes = {{ {{")
        for probe_name, samples in curve["probes"]:
            lines.append(f"{indent}        BakedGrabFingerProbeCurve{{")
            lines.append(f"{indent}            .probe = BakedGrabFingerProbe::{probe_name},")
            lines.append(f"{indent}            .samples = {{ {{")
            for sample in samples:
                lines.append(
                    f"{indent}                BakedGrabFingerCurveSample{{ "
                    f"{f32(sample.open_value)}, {f32(sample.angle)}, {f32(sample.reach_scale)} }},"
                )
            lines.append(f"{indent}            }} }},")
            lines.append(f"{indent}        }},")
        lines.append(f"{indent}    }} }},")
        lines.append(f"{indent}}},")

    lines: list[str] = []
    lines.extend([
        "#pragma once",
        "",
        "// Generated by tools/generate_grab_finger_calibration.py. Do not edit by hand.",
        "// Source authority:",
    ])
    lines.extend(f"// - {label}" for label, _ in source_files)
    lines.extend(
        [
            f"// Source SHA-256: {source_hash}",
            "",
            "#include <array>",
            "#include <cstddef>",
            "#include <cstdint>",
            "",
            "namespace rock::grab_finger_calibration_data",
            "{",
            f"    inline constexpr std::uint32_t kGrabFingerCalibrationGeneratorVersion = {GENERATOR_VERSION};",
            f"    inline constexpr std::size_t kGrabFingerCalibrationSampleCount = {SAMPLE_COUNT};",
            f"    inline constexpr const char* kGrabFingerCalibrationSourceHash = \"{source_hash}\";",
            "",
            "    enum class BakedGrabFingerProbe : std::uint8_t",
            "    {",
            "        Tip,",
            "        Outer,",
            "        Inner",
            "    };",
            "",
            "    struct BakedGrabFingerCurveSample",
            "    {",
            "        float openValue = 1.0f;",
            "        float angleRadians = 0.0f;",
            "        float reachScale = 1.0f;",
            "    };",
            "",
            "    struct BakedGrabFingerProbeCurve",
            "    {",
            "        BakedGrabFingerProbe probe = BakedGrabFingerProbe::Tip;",
            "        std::array<BakedGrabFingerCurveSample, kGrabFingerCalibrationSampleCount> samples{};",
            "    };",
            "",
            "    struct BakedGrabFingerCurve",
            "    {",
            "        float normalSign = 1.0f;",
            "        float surfaceThicknessScale = 0.05f;",
            "        std::array<BakedGrabFingerProbeCurve, 3> probes{};",
            "    };",
            "",
            "    enum class BakedGrabThumbLane : std::uint8_t",
            "    {",
            "        Wrap,",
            "        Opposition,",
            "        SidePad",
            "    };",
            "",
            "    enum class BakedGrabThumbCurveSource : std::uint8_t",
            "    {",
            "        HandThumb,",
            "        SidePad",
            "    };",
            "",
            "    struct BakedGrabThumbLaneCurve",
            "    {",
            "        BakedGrabThumbLane lane = BakedGrabThumbLane::Wrap;",
            "        BakedGrabThumbCurveSource curveSource = BakedGrabThumbCurveSource::HandThumb;",
            "        float normalBlend = 0.0f;",
            "        float localCorrectionStrength = 0.0f;",
            "        float surfaceThicknessScale = 0.06f;",
            "        bool applyAuthoredNormalSign = true;",
            "    };",
            "",
            "    struct BakedGrabFingerHandProfile",
            "    {",
            "        bool isLeft = false;",
            "        bool inPowerArmor = false;",
            "        std::array<BakedGrabFingerCurve, 5> fingers{};",
            "    };",
            "",
            "    struct BakedGrabThumbProfile",
            "    {",
            "        bool isLeft = false;",
            "        bool inPowerArmor = false;",
            "        std::array<BakedGrabThumbLaneCurve, 3> lanes{};",
            "        BakedGrabFingerCurve sidePadCurve{};",
            "    };",
            "",
            "    inline constexpr std::array<BakedGrabFingerHandProfile, 4> kBakedGrabFingerHandProfiles{ {",
        ]
    )

    for profile_index, profile in enumerate(profiles):
        lines.append("        BakedGrabFingerHandProfile{")
        lines.append(f"            .isLeft = {'true' if profile['is_left'] else 'false'},")
        lines.append(f"            .inPowerArmor = {'true' if profile['in_power_armor'] else 'false'},")
        lines.append("            .fingers = { {")
        for finger in profile["fingers"]:
            append_baked_curve(lines, finger, "                ")
        lines.append("            } },")
        lines.append("        },")
        if profile_index + 1 < len(profiles):
            lines.append("")

    lines.extend(
        [
            "    } };",
            "",
            "    inline constexpr std::array<BakedGrabThumbProfile, 4> kBakedGrabThumbProfiles{ {",
        ]
    )

    for profile_index, profile in enumerate(profiles):
        lines.append("        BakedGrabThumbProfile{")
        lines.append(f"            .isLeft = {'true' if profile['is_left'] else 'false'},")
        lines.append(f"            .inPowerArmor = {'true' if profile['in_power_armor'] else 'false'},")
        lines.append("            .lanes = { {")
        for lane in profile["thumb_lanes"]:
            lines.append("                BakedGrabThumbLaneCurve{")
            lines.append(f"                    .lane = BakedGrabThumbLane::{lane['name']},")
            lines.append(f"                    .curveSource = BakedGrabThumbCurveSource::{lane['curve_source']},")
            lines.append(f"                    .normalBlend = {f32(lane['normal_blend'])},")
            lines.append(f"                    .localCorrectionStrength = {f32(lane['local_correction_strength'])},")
            lines.append(f"                    .surfaceThicknessScale = {f32(lane['surface_thickness_scale'])},")
            lines.append(f"                    .applyAuthoredNormalSign = {'true' if lane['apply_authored_normal_sign'] else 'false'},")
            lines.append("                },")
        lines.append("            } },")
        lines.append("            .sidePadCurve =")
        append_baked_curve(lines, profile["thumb_side_pad_curve"], "                ")
        lines.append("        },")
        if profile_index + 1 < len(profiles):
            lines.append("")

    lines.extend(
        [
            "    } };",
            "",
            "    [[nodiscard]] constexpr const BakedGrabFingerHandProfile& bakedGrabFingerHandProfile(bool isLeft, bool inPowerArmor)",
            "    {",
            "        return kBakedGrabFingerHandProfiles[(inPowerArmor ? 2 : 0) + (isLeft ? 1 : 0)];",
            "    }",
            "",
            "    [[nodiscard]] constexpr const BakedGrabThumbProfile& bakedGrabThumbProfile(bool isLeft, bool inPowerArmor)",
            "    {",
            "        return kBakedGrabThumbProfiles[(inPowerArmor ? 2 : 0) + (isLeft ? 1 : 0)];",
            "    }",
            "}",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    repo_root = repo_root_from_script()
    workspace_root = workspace_root_from_repo(repo_root)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hfrik-root", type=pathlib.Path, default=workspace_root / "main_projects" / "hFRIK")
    parser.add_argument("--output", type=pathlib.Path, default=repo_root / OUTPUT_RELATIVE)
    parser.add_argument("--check", action="store_true", help="fail if the generated header differs")
    args = parser.parse_args()

    generated = generate_header(args.hfrik_root.resolve())
    output = args.output.resolve()
    if args.check:
        if not output.exists():
            print(f"missing generated calibration header: {output}", file=sys.stderr)
            return 1
        existing = output.read_text(encoding="utf-8").replace("\r\n", "\n")
        if existing != generated:
            print(f"{output} is stale; rerun tools/generate_grab_finger_calibration.py", file=sys.stderr)
            return 1
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(generated, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
