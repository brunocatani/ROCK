#include "physics-interaction/grab/GrabLocalGraspSolve.h"

#include <cstdio>
#include <limits>
#include <vector>

namespace
{
    using rock::grab_local_grasp::Config;
    using rock::grab_local_grasp::Kind;
    using rock::grab_local_grasp::Result;
    using rock::grab_local_grasp::Vec3;
    namespace math = rock::grab_local_grasp::detail;

    struct Triangle { Vec3 v0, v1, v2; };
    using Mesh = std::vector<Triangle>;

    constexpr float kDegrees = 0.01745329252f;

    // In-palm-plane unit direction, measured from the cross-palm axis.
    Vec3 inPlane(float degreesFromCrossPalm)
    {
        return Vec3{ std::sin(degreesFromCrossPalm * kDegrees), std::cos(degreesFromCrossPalm * kDegrees), 0.0f };
    }

    Vec3 at(const Vec3& centre, const Vec3& u, float a, const Vec3& v, float b, const Vec3& w, float c)
    {
        return math::add(centre, math::add(math::mul(u, a), math::add(math::mul(v, b), math::mul(w, c))));
    }

    void addQuad(Mesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
    {
        mesh.push_back({ a, b, c });
        mesh.push_back({ a, c, d });
    }

    void addBox(Mesh& mesh, const Vec3& centre, const Vec3& u, float hu, const Vec3& v, float hv, const Vec3& w, float hw)
    {
        for (const float sign : { -1.0f, 1.0f }) {
            addQuad(mesh, at(centre, u, sign * hu, v, -hv, w, -hw), at(centre, u, sign * hu, v, hv, w, -hw),
                at(centre, u, sign * hu, v, hv, w, hw), at(centre, u, sign * hu, v, -hv, w, hw));
            addQuad(mesh, at(centre, u, -hu, v, sign * hv, w, -hw), at(centre, u, hu, v, sign * hv, w, -hw),
                at(centre, u, hu, v, sign * hv, w, hw), at(centre, u, -hu, v, sign * hv, w, hw));
            addQuad(mesh, at(centre, u, -hu, v, -hv, w, sign * hw), at(centre, u, hu, v, -hv, w, sign * hw),
                at(centre, u, hu, v, hv, w, sign * hw), at(centre, u, -hu, v, hv, w, sign * hw));
        }
    }

    void addCylinder(Mesh& mesh, const Vec3& centre, const Vec3& axis, float radius, float halfLength, int segments = 24)
    {
        const Vec3 reference = std::abs(axis.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
        const Vec3 p = math::normalizeOrZero(math::cross(axis, reference));
        const Vec3 q = math::cross(axis, p);
        auto ring = [&](int index, float station) {
            const float angle = 6.283185307f * static_cast<float>(index) / static_cast<float>(segments);
            return at(centre, axis, station, p, radius * std::cos(angle), q, radius * std::sin(angle));
        };
        for (int i = 0; i < segments; ++i) {
            addQuad(mesh, ring(i, -halfLength), ring(i + 1, -halfLength), ring(i + 1, halfLength), ring(i, halfLength));
            for (const float sign : { -1.0f, 1.0f }) {
                mesh.push_back({ math::add(centre, math::mul(axis, sign * halfLength)), ring(i, sign * halfLength), ring(i + 1, sign * halfLength) });
            }
        }
    }

    // The runtime seed: the mesh point nearest the palm centre (the origin).
    Vec3 nearestToPalm(const Mesh& mesh)
    {
        return rock::grab_local_grasp::nearestPointToOrigin(mesh, [](const Vec3& point) { return point; }).point;
    }

    Result solveMesh(const Mesh& mesh, const Vec3& gripLine = Vec3{ 0.0f, 1.0f, 0.0f })
    {
        return rock::grab_local_grasp::solve(mesh, nearestToPalm(mesh), gripLine, Config{}, [](const Vec3& point) { return point; });
    }

    bool ok = true;

    void expect(bool condition, const char* message, const Result& result)
    {
        if (!condition) {
            ok = false;
            std::printf("FAIL: %s\n  kind=%s reason=%s valid=%d align=%.2f(%d) elong=%.2f width=%.2f thick=%.2f length=%.2f gap=%.2f thread=%d limit=%.2f "
                        "station=%.2f pivot=(%.2f,%.2f,%.2f) from=(%.2f,%.2f,%.2f) samples=%u near=%u\n",
                message, rock::grab_local_grasp::kindName(result.kind), result.reason, result.valid, result.alignDegrees, result.hasAlignment,
                result.elongation, result.widthGameUnits, result.thicknessGameUnits, result.lengthGameUnits, result.gapBehindGameUnits,
                result.threadFingers, result.threadDepthLimitGameUnits, result.stationShiftGameUnits,
                result.pivot.x, result.pivot.y, result.pivot.z, result.alignFrom.x, result.alignFrom.y, result.alignFrom.z,
                result.sampleCount, result.nearSampleCount);
        }
    }

    bool near(float value, float expected, float tolerance) { return std::abs(value - expected) <= tolerance; }
    bool nearPoint(const Vec3& value, const Vec3& expected, float tolerance)
    {
        return std::sqrt(math::lengthSquared(math::sub(value, expected))) <= tolerance;
    }
    bool parallel(const Vec3& lhs, const Vec3& rhs, float degrees)
    {
        return std::abs(math::dot(math::normalizeOrZero(lhs), math::normalizeOrZero(rhs))) >= std::cos(degrees * kDegrees);
    }

    // A mug with its handle in front of the palm: bar, two bridges, body behind.
    Mesh mugWithHandle(float handleDegrees)
    {
        const Vec3 along = inPlane(handleDegrees);
        const Vec3 side = inPlane(handleDegrees + 90.0f);
        const Vec3 depth{ 0.0f, 0.0f, 1.0f };
        Mesh mesh;
        addBox(mesh, Vec3{ 0.0f, 0.0f, 1.4f }, along, 1.75f, side, 0.35f, depth, 0.4f);
        for (const float sign : { -1.0f, 1.0f }) {
            addBox(mesh, math::add(Vec3{ 0.0f, 0.0f, 2.9f }, math::mul(along, sign * 1.55f)), along, 0.2f, side, 0.35f, depth, 1.1f);
        }
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 6.75f }, along, 2.75f, 2.75f);
        return mesh;
    }
}

int main()
{
    {
        const auto result = solveMesh(mugWithHandle(30.0f));
        expect(result.valid && result.kind == Kind::Bar, "a handle in front of the palm is a bar, not the mug", result);
        expect(result.threadFingers && near(result.gapBehindGameUnits, 2.2f, 0.45f), "the gap behind the handle is found for the fingers", result);
        expect(nearPoint(result.pivot, Vec3{ 0.0f, 0.0f, 1.4f }, 0.3f), "the handle centreline is the seat point", result);
        expect(parallel(result.alignFrom, inPlane(30.0f), 4.0f) && result.hasAlignment && near(result.alignDegrees, 30.0f, 4.0f),
            "the handle turns onto the grip line by the smallest rotation", result);
        expect(result.threadDepthLimitGameUnits > 0.4f && result.threadDepthLimitGameUnits < 2.6f,
            "the thread limit lies inside the gap, behind the handle and in front of the body", result);
        // Finger closure: body triangles that span the loop's opening are
        // dropped; the handle, the bridges that close the loop and the body's
        // end caps beyond them are kept. The mug is built as 12 handle
        // triangles, 24 bridge triangles, then the body.
        const auto mug = mugWithHandle(30.0f);
        const Vec3 along = inPlane(30.0f);
        std::size_t handleDropped = 0, bridgesDropped = 0, spanningKept = 0, capsDropped = 0;
        for (std::size_t i = 0; i < mug.size(); ++i) {
            const bool dropped = rock::grab_local_grasp::behindThreadGap(result, mug[i].v0, mug[i].v1, mug[i].v2);
            const float s0 = math::dot(mug[i].v0, along), s1 = math::dot(mug[i].v1, along), s2 = math::dot(mug[i].v2, along);
            const bool spansOpening = (std::min)({ s0, s1, s2 }) < 0.0f && (std::max)({ s0, s1, s2 }) > 0.0f;
            if (i < 12) {
                handleDropped += dropped ? 1 : 0;
            } else if (i < 36) {
                bridgesDropped += dropped ? 1 : 0;
            } else if (spansOpening) {
                spanningKept += dropped ? 0 : 1;
            } else {
                capsDropped += dropped ? 1 : 0;
            }
        }
        expect(handleDropped == 0, "fingers still close on the handle", result);
        expect(bridgesDropped == 0, "the arms that close the loop still stop the fingers", result);
        expect(spanningKept == 0, "the body behind the opening no longer stops the fingers", result);
        expect(capsDropped == 0, "mesh beyond the loop's ends still stops the fingers", result);
    }
    {
        // Same mug, hand's grip line tilted: the turn follows the hand.
        const auto result = solveMesh(mugWithHandle(30.0f), inPlane(15.0f));
        expect(result.kind == Kind::Bar && near(result.alignDegrees, 15.0f, 4.0f) && math::dot(result.alignTo, inPlane(15.0f)) > 0.99f,
            "alignment targets the supplied grip line", result);
    }
    {
        // Handle pointing the other way round: the line is unoriented.
        const auto result = solveMesh(mugWithHandle(150.0f));
        expect(result.kind == Kind::Bar && near(result.alignDegrees, 30.0f, 4.0f), "a bar never turns more than a quarter turn", result);
    }
    {
        Mesh mesh;
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 1.6f }, inPlane(50.0f), 0.6f, 8.0f);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Bar && !result.threadFingers && result.gapBehindGameUnits < 0.0f, "a free rod is a bar with nothing to thread", result);
        expect(!rock::grab_local_grasp::behindThreadGap(result, Vec3{ 0.0f, 0.0f, 50.0f }, Vec3{ 1.0f, 0.0f, 50.0f }, Vec3{ 0.0f, 1.0f, 50.0f }),
            "nothing is dropped from finger closure without a gap to thread", result);
        expect(near(result.alignDegrees, 50.0f, 4.0f) && nearPoint(result.pivot, Vec3{ 0.0f, 0.0f, 1.6f }, 0.3f), "a rod seats by its centreline", result);
    }
    {
        // Rod leaning out of the palm plane.
        Mesh mesh;
        const Vec3 axis{ 0.0f, std::cos(25.0f * kDegrees), std::sin(25.0f * kDegrees) };
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 1.8f }, axis, 0.5f, 8.0f);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Bar && parallel(result.alignFrom, axis, 4.0f) && near(result.alignDegrees, 25.0f, 4.0f),
            "a leaning rod is laid into the palm plane", result);
    }
    {
        // Short rod grabbed near one end: the hand moves onto the rod.
        Mesh mesh;
        addCylinder(mesh, Vec3{ 0.0f, 2.2f, 1.6f }, inPlane(0.0f), 0.6f, 3.0f);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Bar && result.stationShiftGameUnits > 0.8f && result.pivot.y > 0.8f,
            "the seat slides along a bar to keep the hand on it", result);
    }
    {
        // Plank lying flat under the palm.
        Mesh mesh;
        addBox(mesh, Vec3{ 0.0f, 0.0f, 1.15f }, inPlane(35.0f), 8.0f, inPlane(125.0f), 1.0f, Vec3{ 0.0f, 0.0f, 1.0f }, 0.15f);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Bar && near(result.alignDegrees, 35.0f, 4.0f) && near(result.widthGameUnits, 2.0f, 0.4f),
            "a narrow plank is wrapped, not laid flat", result);
    }
    {
        // Plate seen edge-on.
        Mesh mesh;
        addBox(mesh, Vec3{ 0.0f, 0.0f, 7.0f }, inPlane(40.0f), 7.0f, inPlane(130.0f), 0.25f, Vec3{ 0.0f, 0.0f, 1.0f }, 6.0f);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Edge && near(result.alignDegrees, 40.0f, 4.0f), "a plate rim is an edge held along the grip line", result);
        expect(nearPoint(result.pivot, Vec3{ 0.0f, 0.0f, 1.0f }, 0.35f) && !result.threadFingers, "an edge seats by its crest", result);
    }
    {
        // Bottle body.
        Mesh mesh;
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 3.3f }, inPlane(60.0f), 2.3f, 6.0f, 32);
        const auto result = solveMesh(mesh);
        expect(result.kind == Kind::Cylinder && near(result.alignDegrees, 60.0f, 5.0f), "a bottle body turns its axis onto the grip line", result);
        expect(nearPoint(result.pivot, Vec3{ 0.0f, 0.0f, 1.0f }, 0.2f), "a wide ridge keeps the nearest point as its seat", result);
    }
    {
        // Mug-sized body: round and cuppable, but about as tall as it is wide.
        Mesh mesh;
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 4.0f }, inPlane(40.0f), 3.0f, 3.5f, 32);
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None && result.elongation < 1.9f,
            "a round body with no clear direction keeps the current seat", result);
        // The same girth, clearly longer: a can the fingers wrap.
        Mesh can;
        addCylinder(can, Vec3{ 0.0f, 0.0f, 3.0f }, inPlane(40.0f), 2.0f, 3.4f, 32);
        const auto canResult = solveMesh(can);
        expect(canResult.kind == Kind::Cylinder && near(canResult.alignDegrees, 40.0f, 5.0f), "a can turns its axis onto the grip line", canResult);
    }
    {
        // Mug body with no handle in front: compact, nothing to align.
        Mesh mesh;
        addCylinder(mesh, Vec3{ 0.0f, 0.0f, 3.75f }, inPlane(20.0f), 2.75f, 2.75f, 32);
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None, "a compact body keeps the current seat", result);
    }
    {
        // Face tilted against the palm.
        Mesh mesh;
        const float slope = std::tan(20.0f * kDegrees);
        addQuad(mesh, Vec3{ -20.0f, -20.0f, 1.0f - 20.0f * slope }, Vec3{ 20.0f, -20.0f, 1.0f + 20.0f * slope },
            Vec3{ 20.0f, 20.0f, 1.0f + 20.0f * slope }, Vec3{ -20.0f, 20.0f, 1.0f - 20.0f * slope });
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None && !result.hasAlignment, "a tilted flat face keeps the current seat", result);
    }
    {
        Mesh mesh;
        addQuad(mesh, Vec3{ -20.0f, -20.0f, 1.0f }, Vec3{ 20.0f, -20.0f, 1.0f }, Vec3{ 20.0f, 20.0f, 1.0f }, Vec3{ -20.0f, 20.0f, 1.0f });
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None && !result.hasAlignment, "a face flat on the palm keeps the current seat", result);
    }
    {
        // Steep single face: neither a face to flatten nor an edge.
        Mesh mesh;
        const float slope = std::tan(50.0f * kDegrees);
        addQuad(mesh, Vec3{ -20.0f, -20.0f, 1.0f - 20.0f * slope }, Vec3{ 20.0f, -20.0f, 1.0f + 20.0f * slope },
            Vec3{ 20.0f, 20.0f, 1.0f + 20.0f * slope }, Vec3{ -20.0f, 20.0f, 1.0f - 20.0f * slope });
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None, "a steep face keeps the current seat", result);
    }
    {
        // Box edge towards the palm.
        Mesh mesh;
        const Vec3 along = inPlane(25.0f);
        const Vec3 side = inPlane(115.0f);
        const Vec3 crest{ 0.0f, 0.0f, 1.0f };
        for (const float sign : { -1.0f, 1.0f }) {
            const Vec3 slopeOut = math::add(math::mul(side, sign * 14.0f), Vec3{ 0.0f, 0.0f, 14.0f });
            addQuad(mesh, math::add(crest, math::mul(along, -14.0f)), math::add(crest, math::mul(along, 14.0f)),
                math::add(math::add(crest, math::mul(along, 14.0f)), slopeOut), math::add(math::add(crest, math::mul(along, -14.0f)), slopeOut));
        }
        const auto result = solveMesh(mesh);
        expect(result.valid && result.kind == Kind::None, "a box edge is nothing the hand closes on and keeps the current seat", result);
    }
    {
        const Mesh empty;
        const auto result = rock::grab_local_grasp::solve(empty, Vec3{}, Vec3{ 0.0f, 1.0f, 0.0f }, Config{}, [](const Vec3& point) { return point; });
        expect(!result.valid && result.kind == Kind::None, "no mesh keeps the current seat", result);
    }
    {
        auto mesh = mugWithHandle(30.0f);
        mesh[0].v0.x = std::numeric_limits<float>::quiet_NaN();
        const auto result = rock::grab_local_grasp::solve(mesh, Vec3{ 0.0f, 0.0f, 1.0f }, Vec3{ 0.0f, 1.0f, 0.0f }, Config{}, [](const Vec3& point) { return point; });
        expect(!result.valid, "non-finite mesh cannot author a seat", result);
        const auto zeroLine = rock::grab_local_grasp::solve(mugWithHandle(30.0f), Vec3{ 0.0f, 0.0f, 1.0f }, Vec3{}, Config{}, [](const Vec3& point) { return point; });
        expect(!zeroLine.valid, "a missing grip line cannot author a seat", zeroLine);
    }
    {
        Config tiny{};
        tiny.maxSamples = 64;
        const auto mesh = mugWithHandle(30.0f);
        const auto result = rock::grab_local_grasp::solve(mesh, nearestToPalm(mesh), Vec3{ 0.0f, 1.0f, 0.0f }, tiny, [](const Vec3& point) { return point; });
        expect(!result.valid && result.kind == Kind::None, "an exhausted sample budget keeps the current seat", result);
    }
    {
        // Seat position along the fingers. Finger bases 3.5 ahead of the palm
        // centre, finger radius 0.55.
        using rock::grab_local_grasp::fingerBaseSeat;
        const Vec3 bases{ 3.5f, 0.0f, -0.5f };
        const Vec3 straightLine{ 0.0f, 1.0f, 0.0f };
        Result bar{};
        bar.kind = Kind::Bar;
        bar.widthGameUnits = 0.6f;
        expect(near(fingerBaseSeat(bar, bases, 0.55f, straightLine), 3.5f - 0.55f - 0.3f, 0.001f),
            "a thin bar seats in the corner under the finger bases", bar);
        Result bottle{};
        bottle.kind = Kind::Bar;
        bottle.widthGameUnits = 2.5f;
        expect(near(fingerBaseSeat(bottle, bases, 0.55f, straightLine), 3.5f - 0.55f - 1.25f, 0.001f),
            "a thicker part seats further back by its own half width", bottle);
        Result body{};
        body.kind = Kind::Cylinder;
        body.widthGameUnits = 6.0f;
        expect(fingerBaseSeat(body, bases, 0.55f, straightLine) == 0.0f, "a part too wide for the finger corner seats on the palm centre", body);
        // A tilted grip line through off-centre finger bases still crosses
        // the palm centre's cross-palm coordinate at the right place.
        const Vec3 offCentre{ 3.5f, 1.0f, 0.0f };
        const float tilted = fingerBaseSeat(bar, offCentre, 0.55f, inPlane(15.0f));
        expect(near(tilted, 3.5f - 0.55f - 0.3f - std::tan(15.0f * kDegrees), 0.001f), "the seat follows the grip line to the palm centre", bar);
        expect(fingerBaseSeat(bar, Vec3{ 0.4f, 0.0f, 0.0f }, 0.55f, straightLine) == 0.0f, "the seat never moves behind the palm centre", bar);
        expect(fingerBaseSeat(Result{}, bases, 0.55f, straightLine) == 0.0f, "an unrecognised window keeps the palm centre", bar);
        expect(fingerBaseSeat(bar, Vec3{ -1.0f, 0.0f, 0.0f }, 0.55f, straightLine) == 0.0f &&
               fingerBaseSeat(bar, bases, std::numeric_limits<float>::quiet_NaN(), straightLine) == 0.0f &&
               fingerBaseSeat(bar, bases, 0.55f, Vec3{ 1.0f, 0.0f, 0.0f }) == 0.0f,
            "unusable hand measurements keep the palm centre", bar);
    }
    {
        // Same answer on every call: no hidden state.
        const auto mesh = mugWithHandle(30.0f);
        const auto first = solveMesh(mesh);
        const auto second = solveMesh(mesh);
        expect(first.kind == second.kind && first.alignDegrees == second.alignDegrees && first.pivot.x == second.pivot.x &&
               first.pivot.y == second.pivot.y && first.pivot.z == second.pivot.z, "the solve is deterministic", second);
    }
    return ok ? 0 : 1;
}
