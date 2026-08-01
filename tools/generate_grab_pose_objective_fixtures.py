"""Regenerate tests/GrabPoseObjectiveFixtures.h from ground-truth captures.

Reads saved-grab capture files (the write-only ground truth the save gesture
emits), converts a small representative set into proxy-local fixture data, and
computes golden values with the SAME math GrabPoseObjective.h implements:
unweighted-vertex-mean pivot, area-weighted PCA, station-sweep girth profile,
one-sided terms with the 1e9 reject sentinel. The C++ parity tests then pin
the header against these goldens - if either side drifts, the test fails.

Usage:
    python tools/generate_grab_pose_objective_fixtures.py
        [--captures <dir>] [--out tests/GrabPoseObjectiveFixtures.h]

Provenance: Docs\\ROCK\\docs\\2026-07-26-grab-pose-objective-fit.md.
"""
import argparse
import glob
import json
import math
import os

MAX_TRIS = 260
PALM_SLACK = 2.5
PEN_LIMIT = 1.5
WRAP_CLAMP = 4.0
GIRTH_WINDOW = 3.0
GIRTH_OK = 3.0
GIRTH_MIN_TOL = 0.5
ROD_MIN_R12 = 2.0
PLATE_MIN_R23 = 1.25

# (objectName, hand) -> one fixture per shape class + the thin-rod shove case.
FIXTURE_PICKS = [
    ("Beer Bottle", "right"),
    ("Cigar", "right"),
    ("Grognak the Barbarian", "right"),
    ("Tin Can", "right"),
]

TIP_ROLES = ("ThumbTip", "IndexTip", "MiddleTip", "RingTip", "PinkyTip")


def sub(a, b): return [a[i] - b[i] for i in range(3)]
def add(a, b): return [a[i] + b[i] for i in range(3)]
def mul(a, s): return [x * s for x in a]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def length(a): return math.sqrt(max(0.0, dot(a, a)))
def norm(a):
    n = length(a)
    return [x / n for x in a] if n > 1e-9 else [0.0, 0.0, 0.0]


def rows_of(frame):
    r = frame["rotate"]
    return [r[0:3], r[3:6], r[6:9]]


def apply_frame(frame, p):
    t, rows, s = frame["translate"], rows_of(frame), frame.get("scale", 1.0) or 1.0
    return [t[i] + s * (p[0] * rows[0][i] + p[1] * rows[1][i] + p[2] * rows[2][i]) for i in range(3)]


def axis_angle(axis, radians):
    x, y, z = norm(axis)
    c, s, t = math.cos(radians), math.sin(radians), 1.0 - math.cos(radians)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def matvec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def closest_point_on_triangle(p, t):
    a, b, c = t
    ab, ac, ap = sub(b, a), sub(c, a), sub(p, a)
    d1, d2 = dot(ab, ap), dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        return a
    bp = sub(p, b); d3, d4 = dot(ab, bp), dot(ac, bp)
    if d3 >= 0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        return add(a, mul(ab, d1 / (d1 - d3) if (d1 - d3) else 0.0))
    cp = sub(p, c); d5, d6 = dot(ab, cp), dot(ac, cp)
    if d6 >= 0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        return add(a, mul(ac, d2 / (d2 - d6) if (d2 - d6) else 0.0))
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        den = (d4 - d3) + (d5 - d6)
        return add(b, mul(sub(c, b), (d4 - d3) / den if den else 0.0))
    den = va + vb + vc
    if den == 0:
        return a
    return add(a, add(mul(ab, vb / den), mul(ac, vc / den)))


def point_triangle_distance(p, t):
    return length(sub(p, closest_point_on_triangle(p, t)))


def _closest_segment_segment(p1, q1, p2, q2):
    d1, d2, r = sub(q1, p1), sub(q2, p2), sub(p1, p2)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    if a <= 1e-9 and e <= 1e-9:
        return p1, p2
    if a <= 1e-9:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = dot(d1, r)
        if e <= 1e-9:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = dot(d1, d2); den = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / den)) if den else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    return add(p1, mul(d1, s)), add(p2, mul(d2, t))


def segment_triangle_distance(p, q, t):
    best = min(point_triangle_distance(p, t), point_triangle_distance(q, t))
    for i in range(3):
        c1, c2 = _closest_segment_segment(p, q, t[i], t[(i + 1) % 3])
        best = min(best, length(sub(c1, c2)))
    return best


def jacobi_pca(tris):
    """Area-weighted vertex PCA, Jacobi eigendecomposition, axes descending."""
    wsum = 0.0
    mean = [0.0] * 3
    for t in tris:
        e0, e1 = sub(t[1], t[0]), sub(t[2], t[0])
        area = 0.5 * length(cross(e0, e1))
        if area <= 1e-9:
            continue
        wsum += area
        centroid = [(t[0][i] + t[1][i] + t[2][i]) / 3.0 for i in range(3)]
        for i in range(3):
            mean[i] += area * centroid[i]
    if wsum <= 1e-9:
        return None, None
    mean = [m / wsum for m in mean]
    cov = [[0.0] * 3 for _ in range(3)]
    for t in tris:
        e0, e1 = sub(t[1], t[0]), sub(t[2], t[0])
        area = 0.5 * length(cross(e0, e1))
        if area <= 1e-9:
            continue
        for v in t:
            d = sub(v, mean)
            for i in range(3):
                for j in range(3):
                    cov[i][j] += (area / 3.0) * d[i] * d[j]
    for i in range(3):
        for j in range(3):
            cov[i][j] /= wsum
    a = [row[:] for row in cov]
    vecs = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(64):
        p, q, off = 0, 1, 0.0
        for i in range(3):
            for j in range(i + 1, 3):
                if abs(a[i][j]) > off:
                    off, p, q = abs(a[i][j]), i, j
        if off < 1e-12:
            break
        theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = (1.0 if theta >= 0 else -1.0) / (abs(theta) + math.sqrt(theta * theta + 1.0))
        c = 1.0 / math.sqrt(t * t + 1.0)
        s = t * c
        for k in range(3):
            akp, akq = a[k][p], a[k][q]
            a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
        for k in range(3):
            apk, aqk = a[p][k], a[q][k]
            a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
        for k in range(3):
            vkp, vkq = vecs[k][p], vecs[k][q]
            vecs[k][p], vecs[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    eig = [(a[i][i], [vecs[0][i], vecs[1][i], vecs[2][i]]) for i in range(3)]
    eig.sort(key=lambda e: -e[0])
    return [e[1] for e in eig], [max(0.0, e[0]) for e in eig]


def girth_profile_min(tris, centroid, axis):
    stations = []
    for t in tris:
        for v in t:
            stations.append(dot(sub(v, centroid), axis))
    lo, hi = min(stations), max(stations)
    best = None
    station = lo
    while station <= hi:
        slab_max, slab_count = 0.0, 0
        for t in tris:
            for v in t:
                from_centroid = sub(v, centroid)
                s = dot(from_centroid, axis)
                if abs(s - station) <= GIRTH_WINDOW:
                    radial = sub(from_centroid, mul(axis, s))
                    slab_max = max(slab_max, length(radial))
                    slab_count += 1
        if slab_count >= 4:
            if best is None or slab_max < best:
                best = slab_max
        station += 1.5
    return best if best is not None else 0.0


def score_terms(tris, centroid, axes, shape, girth_min, hand, rotation, translation):
    """Mirror of GrabPoseObjective.h scorePose (one-sided terms, 1e9 sentinel)."""
    moved = []
    for t in tris:
        moved.append([add(add(matvec(rotation, sub(v, centroid)), centroid), translation) for v in t])

    palm_gap = min(point_triangle_distance(hand["palm"], t) for t in moved) - hand["palmRadius"]

    worst_pen = 0.0
    min_gap = 1e9
    for a, b, radius in hand["capsules"]:
        mid = [(a[i] + b[i]) * 0.5 for i in range(3)]
        reach = length(sub(b, a)) * 0.5 + radius
        near = 1e9
        for t in moved:
            center = [(t[0][i] + t[1][i] + t[2][i]) / 3.0 for i in range(3)]
            bound = max(length(sub(v, center)) for v in t)
            if length(sub(center, mid)) > reach + bound:
                continue
            distance = segment_triangle_distance(a, b, t)
            near = min(near, distance)
            worst_pen = max(worst_pen, radius - distance)
        min_gap = min(min_gap, near - radius)

    wrap_sum = 0.0
    for tip, radius in hand["tips"]:
        gap = min(point_triangle_distance(tip, t) for t in moved) - radius
        wrap_sum += min(max(gap, 0.0), WRAP_CLAMP) ** 2

    terms = dict(
        touch=max(0.0, min_gap) ** 2,
        palmProx=max(0.0, palm_gap - PALM_SLACK) ** 2,
        overPen=max(0.0, worst_pen - PEN_LIMIT) ** 2,
        wrap=wrap_sum / max(1, len(hand["tips"])),
        rodAxis=0.0,
        girth=0.0,
    )
    if shape == "rod":
        moved_axis = norm(matvec(rotation, axes[0]))
        alignment = dot(moved_axis, hand["palmNormal"])
        terms["rodAxis"] = alignment * alignment
        moved_centroid = add(centroid, translation)
        palm_station = dot(sub(hand["palm"], moved_centroid), moved_axis)
        slab_max = 0.0
        for t in moved:
            for v in t:
                from_centroid = sub(v, moved_centroid)
                s = dot(from_centroid, moved_axis)
                if abs(s - palm_station) <= GIRTH_WINDOW:
                    slab_max = max(slab_max, length(sub(from_centroid, mul(moved_axis, s))))
        threshold = max(GIRTH_OK, girth_min + GIRTH_MIN_TOL)
        terms["girth"] = max(0.0, slab_max - threshold) ** 2
    return terms


def load_fixture(path):
    with open(path, encoding="utf-8") as handle:
        document = json.load(handle)
    capture = document.get("capture")
    if not capture or not capture.get("mesh", {}).get("valid"):
        return None
    label = capture["objectProxyLocal"]
    vertices = capture["mesh"]["verticesObjectLocal"]
    tris = [[apply_frame(label, vertices[i + k * 3:i + k * 3 + 3]) for k in range(3)]
            for i in range(0, len(vertices), 9)]
    if len(tris) > MAX_TRIS:
        step = len(tris) / MAX_TRIS
        tris = [tris[int(i * step)] for i in range(MAX_TRIS)]

    segments = {s["role"]: s for s in capture["fingerSegments"] if s["valid"]}
    required = ("PalmFace", "PalmHeel", "IndexBase", "MiddleBase", "RingBase", "PinkyBase")
    if not all(role in segments for role in required):
        return None

    def position(role):
        return segments[role]["frameProxyLocal"]["translate"]

    capsules = []
    for segment in capture["fingerSegments"]:
        if not segment["valid"]:
            continue
        frame = segment["frameProxyLocal"]
        rows = rows_of(frame)
        half = 0.5 * segment["length"]
        a = [frame["translate"][i] - half * rows[0][i] for i in range(3)]
        b = [frame["translate"][i] + half * rows[0][i] for i in range(3)]
        capsules.append((a, b, segment["radius"]))

    tips = [(position(role), segments[role]["radius"]) for role in TIP_ROLES if role in segments]

    centroid = [sum(t[k][i] for t in tris for k in range(3)) / (3 * len(tris)) for i in range(3)]
    axes, eigenvalues = jacobi_pca(tris)
    if axes is None:
        return None
    r12 = math.sqrt(eigenvalues[0] / max(eigenvalues[1], eigenvalues[0] * 1e-4))
    r23 = math.sqrt(eigenvalues[1] / max(eigenvalues[2], eigenvalues[1] * 1e-4))
    shape = "rod" if r12 >= ROD_MIN_R12 else ("plate" if r23 >= PLATE_MIN_R23 else "compact")

    bases_mean = [sum(position(role)[i] for role in ("IndexBase", "MiddleBase", "RingBase", "PinkyBase")) / 4.0
                  for i in range(3)]
    fingers = norm(sub(bases_mean, position("PalmHeel")))
    cross_palm = norm(sub(position("IndexBase"), position("PinkyBase")))
    cross_palm = norm(sub(cross_palm, mul(fingers, dot(cross_palm, fingers))))
    normal = norm(cross(fingers, cross_palm))
    if dot(normal, sub(centroid, position("PalmFace"))) < 0:
        normal = [-x for x in normal]

    girth_min = girth_profile_min(tris, centroid, axes[0]) if shape == "rod" else 0.0

    hand = dict(
        capsules=capsules,
        palm=position("PalmFace"),
        palmRadius=segments["PalmFace"]["radius"],
        tips=tips,
        palmNormal=normal,
    )

    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    pose_a = (axis_angle([1.0, 2.0, 3.0], math.radians(20.0)), [1.0, -0.5, 0.25])
    pose_b = (axis_angle([0.0, 0.0, 1.0], math.radians(45.0)), [0.0, 2.0, 0.0])

    return dict(
        name=document["objectName"],
        hand_name=document["hand"],
        shape=shape,
        tris=tris,
        capsules=capsules,
        palm=position("PalmFace"),
        palm_radius=segments["PalmFace"]["radius"],
        tips=tips,
        palm_normal=normal,
        index_base=position("IndexBase"),
        middle_base=position("MiddleBase"),
        ring_base=position("RingBase"),
        pinky_base=position("PinkyBase"),
        palm_heel=position("PalmHeel"),
        centroid=centroid,
        eigenvalues=eigenvalues,
        r12=r12,
        r23=r23,
        girth_min=girth_min,
        terms_identity=score_terms(tris, centroid, axes, shape, girth_min, hand, identity, [0.0, 0.0, 0.0]),
        terms_pose_a=score_terms(tris, centroid, axes, shape, girth_min, hand, *pose_a),
        terms_pose_b=score_terms(tris, centroid, axes, shape, girth_min, hand, *pose_b),
    )


def flt(value):
    text = f"{value:.9g}"
    if "." not in text and "e" not in text and "inf" not in text and "nan" not in text:
        text += ".0"
    return text + "f"


def emit(fixtures, out_path):
    lines = []
    lines.append("// Generated by tools/generate_grab_pose_objective_fixtures.py - do not hand-edit.")
    lines.append("// Fixture data: proxy-local ground-truth captures; goldens computed with the")
    lines.append("// mirrored offline math. Regenerate after any objective-term or convention change.")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <array>")
    lines.append("#include <cstddef>")
    lines.append("")
    lines.append("namespace rock::grab_pose_objective_fixtures")
    lines.append("{")
    lines.append("    struct FixtureTerms")
    lines.append("    {")
    lines.append("        float touch;")
    lines.append("        float palmProx;")
    lines.append("        float overPen;")
    lines.append("        float wrap;")
    lines.append("        float rodAxis;")
    lines.append("        float girth;")
    lines.append("    };")
    lines.append("")
    lines.append("    struct Fixture")
    lines.append("    {")
    lines.append("        const char* name;")
    lines.append("        const char* shape;")
    lines.append("        std::size_t triangleCount;")
    lines.append("        const float* vertices;   // 9 floats per triangle")
    lines.append("        std::size_t capsuleCount;")
    lines.append("        const float* capsules;   // 7 floats per capsule: ax ay az bx by bz radius")
    lines.append("        float palmCenter[3];")
    lines.append("        float palmRadius;")
    lines.append("        std::size_t tipCount;")
    lines.append("        const float* tips;       // 4 floats per tip: x y z radius")
    lines.append("        float palmNormal[3];")
    lines.append("        float indexBase[3];")
    lines.append("        float middleBase[3];")
    lines.append("        float ringBase[3];")
    lines.append("        float pinkyBase[3];")
    lines.append("        float palmHeel[3];")
    lines.append("        float centroid[3];")
    lines.append("        float eigenvalues[3];")
    lines.append("        float elongationRatio;")
    lines.append("        float secondElongationRatio;")
    lines.append("        float girthProfileMin;")
    lines.append("        FixtureTerms termsIdentity;")
    lines.append("        FixtureTerms termsPoseA;  // 20deg about normalize(1,2,3), translate (1,-0.5,0.25)")
    lines.append("        FixtureTerms termsPoseB;  // 45deg about (0,0,1), translate (0,2,0)")
    lines.append("    };")
    lines.append("")

    for index, fixture in enumerate(fixtures):
        vertex_values = []
        for t in fixture["tris"]:
            for v in t:
                vertex_values.extend(v)
        capsule_values = []
        for a, b, radius in fixture["capsules"]:
            capsule_values.extend(a)
            capsule_values.extend(b)
            capsule_values.append(radius)
        tip_values = []
        for tip, radius in fixture["tips"]:
            tip_values.extend(tip)
            tip_values.append(radius)

        def emit_array(label, values):
            lines.append(f"    inline constexpr float kFixture{index}{label}[] = {{")
            for i in range(0, len(values), 9):
                chunk = ", ".join(flt(v) for v in values[i:i + 9])
                lines.append(f"        {chunk},")
            lines.append("    };")

        emit_array("Vertices", vertex_values)
        emit_array("Capsules", capsule_values)
        emit_array("Tips", tip_values)
        lines.append("")

    lines.append(f"    inline constexpr std::array<Fixture, {len(fixtures)}> kFixtures{{ {{")
    for index, fixture in enumerate(fixtures):
        def vec(v):
            return "{ " + ", ".join(flt(x) for x in v) + " }"

        def terms(t):
            return ("{ " + ", ".join(flt(t[k]) for k in ("touch", "palmProx", "overPen", "wrap", "rodAxis", "girth")) + " }")

        lines.append("        Fixture{")
        lines.append(f"            \"{fixture['name']} ({fixture['hand_name']})\",")
        lines.append(f"            \"{fixture['shape']}\",")
        lines.append(f"            {len(fixture['tris'])}, kFixture{index}Vertices,")
        lines.append(f"            {len(fixture['capsules'])}, kFixture{index}Capsules,")
        lines.append(f"            {vec(fixture['palm'])}, {flt(fixture['palm_radius'])},")
        lines.append(f"            {len(fixture['tips'])}, kFixture{index}Tips,")
        lines.append(f"            {vec(fixture['palm_normal'])},")
        lines.append(f"            {vec(fixture['index_base'])}, {vec(fixture['middle_base'])}, "
                     f"{vec(fixture['ring_base'])}, {vec(fixture['pinky_base'])}, {vec(fixture['palm_heel'])},")
        lines.append(f"            {vec(fixture['centroid'])}, {vec(fixture['eigenvalues'])},")
        lines.append(f"            {flt(fixture['r12'])}, {flt(fixture['r23'])}, {flt(fixture['girth_min'])},")
        lines.append(f"            {terms(fixture['terms_identity'])},")
        lines.append(f"            {terms(fixture['terms_pose_a'])},")
        lines.append(f"            {terms(fixture['terms_pose_b'])},")
        lines.append("        },")
    lines.append("    } };")
    lines.append("}")

    with open(out_path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--captures",
        default=os.path.expandvars(
            r"%USERPROFILE%\Documents\My Games\Fallout4VR\ROCK_Config\SavedGrabOffsets\captures"),
    )
    parser.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "tests", "GrabPoseObjectiveFixtures.h"))
    args = parser.parse_args()

    by_key = {}
    for path in sorted(glob.glob(os.path.join(args.captures, "*.capture.json"))):
        with open(path, encoding="utf-8") as handle:
            document = json.load(handle)
        by_key[(document.get("objectName"), document.get("hand"))] = path

    fixtures = []
    for name, hand in FIXTURE_PICKS:
        path = by_key.get((name, hand))
        if not path:
            raise SystemExit(f"capture not found for ({name}, {hand}) in {args.captures}")
        fixture = load_fixture(path)
        if not fixture:
            raise SystemExit(f"capture unusable for ({name}, {hand}): {path}")
        fixtures.append(fixture)
        print(f"  {fixture['name']} ({fixture['hand_name']}): {fixture['shape']}, "
              f"{len(fixture['tris'])} tris, girthMin={fixture['girth_min']:.2f}")

    emit(fixtures, os.path.normpath(args.out))
    print(f"wrote {os.path.normpath(args.out)}")


if __name__ == "__main__":
    main()
