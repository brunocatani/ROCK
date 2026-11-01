#include "physics-interaction/grab/GrabPoseReplay.h"

#include <nlohmann/json.hpp>

namespace rock::grab_pose_replay
{
    using nlohmann::json;

    void to_json(json& j, const Transform& t) { j = json{ { "rotate", t.rotate }, { "translate", t.translate }, { "scale", t.scale } }; }
    void from_json(const json& j, Transform& t)
    {
        j.at("rotate").get_to(t.rotate);
        j.at("translate").get_to(t.translate);
        j.at("scale").get_to(t.scale);
    }

    void to_json(json& j, const FingerChain& c)
    {
        j = json{ { "points", c.points }, { "tipSegmentCenter", c.tipSegmentCenter }, { "tipDirection", c.tipDirection },
            { "tipGeometryValid", c.tipGeometryValid }, { "valid", c.valid } };
    }
    void from_json(const json& j, FingerChain& c)
    {
        j.at("points").get_to(c.points);
        j.at("tipSegmentCenter").get_to(c.tipSegmentCenter);
        j.at("tipDirection").get_to(c.tipDirection);
        j.at("tipGeometryValid").get_to(c.tipGeometryValid);
        j.at("valid").get_to(c.valid);
    }

    void to_json(json& j, const FingerSkeleton& s)
    {
        j = json{ { "fingers", s.fingers }, { "palmNormal", s.palmNormal }, { "palmNormalValid", s.palmNormalValid },
            { "inPowerArmor", s.inPowerArmor }, { "valid", s.valid } };
    }
    void from_json(const json& j, FingerSkeleton& s)
    {
        j.at("fingers").get_to(s.fingers);
        j.at("palmNormal").get_to(s.palmNormal);
        j.at("palmNormalValid").get_to(s.palmNormalValid);
        j.at("inPowerArmor").get_to(s.inPowerArmor);
        j.at("valid").get_to(s.valid);
    }

    void to_json(json& j, const PoseTargets& p)
    {
        j = json{ { "targets", p.targets }, { "targetNormals", p.targetNormals }, { "targetValid", p.targetValid },
            { "targetNormalValid", p.targetNormalValid }, { "seatPoint", p.seatPoint }, { "seatNormal", p.seatNormal },
            { "seatPointValid", p.seatPointValid }, { "seatNormalValid", p.seatNormalValid },
            { "useSeatPointForMissingTargets", p.useSeatPointForMissingTargets },
            { "useWholeMeshForMissingTargets", p.useWholeMeshForMissingTargets }, { "targetCount", p.targetCount } };
    }
    void from_json(const json& j, PoseTargets& p)
    {
        j.at("targets").get_to(p.targets);
        j.at("targetNormals").get_to(p.targetNormals);
        j.at("targetValid").get_to(p.targetValid);
        j.at("targetNormalValid").get_to(p.targetNormalValid);
        j.at("seatPoint").get_to(p.seatPoint);
        j.at("seatNormal").get_to(p.seatNormal);
        j.at("seatPointValid").get_to(p.seatPointValid);
        j.at("seatNormalValid").get_to(p.seatNormalValid);
        j.at("useSeatPointForMissingTargets").get_to(p.useSeatPointForMissingTargets);
        j.at("useWholeMeshForMissingTargets").get_to(p.useWholeMeshForMissingTargets);
        j.at("targetCount").get_to(p.targetCount);
    }

    void to_json(json& j, const SolveOptions& o)
    {
        j = json{ { "minValue", o.minValue }, { "maxTriangleDistanceSquared", o.maxTriangleDistanceSquared },
            { "rejectBacksideHits", o.rejectBacksideHits }, { "surfacePlaneToleranceGameUnits", o.surfacePlaneToleranceGameUnits },
            { "allowSurfaceAimTargets", o.allowSurfaceAimTargets }, { "sweepContactRadiusGameUnits", o.sweepContactRadiusGameUnits },
            { "thumbSweepMaxOpenValue", o.thumbSweepMaxOpenValue }, { "fingerSweepMaxOpenValue", o.fingerSweepMaxOpenValue },
            { "mode", o.mode } };
    }
    void from_json(const json& j, SolveOptions& o)
    {
        j.at("minValue").get_to(o.minValue);
        j.at("maxTriangleDistanceSquared").get_to(o.maxTriangleDistanceSquared);
        j.at("rejectBacksideHits").get_to(o.rejectBacksideHits);
        j.at("surfacePlaneToleranceGameUnits").get_to(o.surfacePlaneToleranceGameUnits);
        j.at("allowSurfaceAimTargets").get_to(o.allowSurfaceAimTargets);
        j.at("sweepContactRadiusGameUnits").get_to(o.sweepContactRadiusGameUnits);
        j.at("thumbSweepMaxOpenValue").get_to(o.thumbSweepMaxOpenValue);
        j.at("fingerSweepMaxOpenValue").get_to(o.fingerSweepMaxOpenValue);
        j.at("mode").get_to(o.mode);
    }

    std::string serialize(const Record& r)
    {
        const json j{
            { "formatVersion", r.formatVersion },
            { "object", r.object },
            { "objectName", r.objectName },
            { "isLeft", r.isLeft },
            { "acquisition", r.acquisition },
            { "gripSource", r.gripSource },
            { "capturedLocal", r.capturedLocal },
            { "trianglesObjectLocal", r.trianglesObjectLocal },
            { "solveObjectWorld", r.solveObjectWorld },
            { "solveHandWorld", r.solveHandWorld },
            { "liveHandWorld", r.liveHandWorld },
            { "grabAnchorWorld", r.grabAnchorWorld },
            { "targets", r.targets },
            { "solveSkeleton", r.solveSkeleton },
            { "commandedOpenDirectionsWorld", r.commandedOpenDirectionsWorld },
            { "commandedOpenDirectionsValid", r.commandedOpenDirectionsValid },
            { "options", r.options },
            { "solvedValues", r.solvedValues },
            { "solvedHitKind", r.solvedHitKind },
            { "finalValues", r.finalValues },
            { "finalJointValues", r.finalJointValues },
            { "frikClosedLocal", r.frikClosedLocal },
            { "frikOpenLocal", r.frikOpenLocal },
            { "frikRigValid", r.frikRigValid },
            { "publishedJointValues", r.publishedJointValues },
            { "publishedSplayRadians", r.publishedSplayRadians },
            { "publishedLocalTransforms", r.publishedLocalTransforms },
            { "publishedLocalTransformMask", r.publishedLocalTransformMask },
            { "renderedObjectWorld", r.renderedObjectWorld },
            { "renderedVisualHandWorld", r.renderedVisualHandWorld },
            { "rendered", r.rendered },
            { "presentedHandWorld", r.presentedHandWorld },
            { "presentedHandValid", r.presentedHandValid },
        };
        return j.dump(1);
    }

    std::optional<Record> parse(std::string_view text, std::string* outError)
    {
        try {
            const json j = json::parse(text);
            Record r{};
            j.at("formatVersion").get_to(r.formatVersion);
            if (r.formatVersion != kFormatVersion) {
                if (outError) *outError = "unsupported formatVersion " + std::to_string(r.formatVersion);
                return std::nullopt;
            }
            j.at("object").get_to(r.object);
            j.at("objectName").get_to(r.objectName);
            j.at("isLeft").get_to(r.isLeft);
            j.at("acquisition").get_to(r.acquisition);
            j.at("gripSource").get_to(r.gripSource);
            j.at("capturedLocal").get_to(r.capturedLocal);
            j.at("trianglesObjectLocal").get_to(r.trianglesObjectLocal);
            j.at("solveObjectWorld").get_to(r.solveObjectWorld);
            j.at("solveHandWorld").get_to(r.solveHandWorld);
            j.at("liveHandWorld").get_to(r.liveHandWorld);
            j.at("grabAnchorWorld").get_to(r.grabAnchorWorld);
            j.at("targets").get_to(r.targets);
            j.at("solveSkeleton").get_to(r.solveSkeleton);
            j.at("commandedOpenDirectionsWorld").get_to(r.commandedOpenDirectionsWorld);
            j.at("commandedOpenDirectionsValid").get_to(r.commandedOpenDirectionsValid);
            j.at("options").get_to(r.options);
            j.at("solvedValues").get_to(r.solvedValues);
            j.at("solvedHitKind").get_to(r.solvedHitKind);
            j.at("finalValues").get_to(r.finalValues);
            j.at("finalJointValues").get_to(r.finalJointValues);
            j.at("frikClosedLocal").get_to(r.frikClosedLocal);
            j.at("frikOpenLocal").get_to(r.frikOpenLocal);
            j.at("frikRigValid").get_to(r.frikRigValid);
            j.at("publishedJointValues").get_to(r.publishedJointValues);
            j.at("publishedSplayRadians").get_to(r.publishedSplayRadians);
            j.at("publishedLocalTransforms").get_to(r.publishedLocalTransforms);
            j.at("publishedLocalTransformMask").get_to(r.publishedLocalTransformMask);
            j.at("renderedObjectWorld").get_to(r.renderedObjectWorld);
            j.at("renderedVisualHandWorld").get_to(r.renderedVisualHandWorld);
            j.at("rendered").get_to(r.rendered);
            if (j.contains("presentedHandWorld")) {
                j.at("presentedHandWorld").get_to(r.presentedHandWorld);
                j.at("presentedHandValid").get_to(r.presentedHandValid);
            }
            return r;
        } catch (const std::exception& error) {
            if (outError) *outError = error.what();
            return std::nullopt;
        }
    }
}
