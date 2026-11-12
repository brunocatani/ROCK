#include "physics-interaction/grab/GrabPoseReplay.h"

#include <nlohmann/json.hpp>

namespace rock::grab_pose_replay
{
    using nlohmann::json;

    void to_json(json& j, const Transform& t)
    {
        j = json{ { "rotate", t.rotate }, { "translate", t.translate }, { "scale", t.scale } };
    }

    void from_json(const json& j, Transform& t)
    {
        j.at("rotate").get_to(t.rotate);
        j.at("translate").get_to(t.translate);
        j.at("scale").get_to(t.scale);
    }

    void to_json(json& j, const FingerChain& c)
    {
        j = json{ { "points", c.points }, { "tipSegmentCenter", c.tipSegmentCenter },
            { "tipDirection", c.tipDirection }, { "tipGeometryValid", c.tipGeometryValid }, { "valid", c.valid } };
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

    void to_json(json& j, const CapsuleOptions& o)
    {
        j = json{ { "minValue", o.minValue }, { "step", o.step } };
    }

    void from_json(const json& j, CapsuleOptions& o)
    {
        j.at("minValue").get_to(o.minValue);
        j.at("step").get_to(o.step);
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
            { "solveSkeleton", r.solveSkeleton },
            { "options", r.options },
            { "solvedValues", r.solvedValues },
            { "solvedContact", r.solvedContact },
            { "solvedClosedPastIntersection", r.solvedClosedPastIntersection },
            { "solvedBlocked", r.solvedBlocked },
            { "finalValues", r.finalValues },
            { "finalJointValues", r.finalJointValues },
            { "frikClosedLocal", r.frikClosedLocal },
            { "frikOpenLocal", r.frikOpenLocal },
            { "frikRigValid", r.frikRigValid },
            { "publishedJointValues", r.publishedJointValues },
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
            if (r.formatVersion != kFormatVersion && r.formatVersion != 2) {
                if (outError)
                    *outError = "unsupported formatVersion " + std::to_string(r.formatVersion);
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
            j.at("solveSkeleton").get_to(r.solveSkeleton);
            j.at("options").get_to(r.options);
            j.at("solvedValues").get_to(r.solvedValues);
            j.at("solvedContact").get_to(r.solvedContact);
            j.at(r.formatVersion == 2 ? "solvedOpenedToClear" : "solvedClosedPastIntersection")
                .get_to(r.solvedClosedPastIntersection);
            j.at("solvedBlocked").get_to(r.solvedBlocked);
            j.at("finalValues").get_to(r.finalValues);
            j.at("finalJointValues").get_to(r.finalJointValues);
            j.at("frikClosedLocal").get_to(r.frikClosedLocal);
            j.at("frikOpenLocal").get_to(r.frikOpenLocal);
            j.at("frikRigValid").get_to(r.frikRigValid);
            j.at("publishedJointValues").get_to(r.publishedJointValues);
            j.at("renderedObjectWorld").get_to(r.renderedObjectWorld);
            j.at("renderedVisualHandWorld").get_to(r.renderedVisualHandWorld);
            j.at("rendered").get_to(r.rendered);
            j.at("presentedHandWorld").get_to(r.presentedHandWorld);
            j.at("presentedHandValid").get_to(r.presentedHandValid);
            return r;
        } catch (const std::exception& error) {
            if (outError)
                *outError = error.what();
            return std::nullopt;
        }
    }
}
