#include "physics-interaction/grab/SavedGrabCaptureFormat.h"

#include <nlohmann/json.hpp>

#include <cstdio>

namespace rock::saved_grab_capture
{
    namespace
    {
        using nlohmann::json;

        std::string formIdToHex(std::uint32_t id)
        {
            char buffer[16]{};
            std::snprintf(buffer, sizeof(buffer), "0x%08X", id);
            return buffer;
        }

        template <std::size_t N>
        json floatArrayToJson(const float (&values)[N])
        {
            json array = json::array();
            for (const float value : values) {
                array.push_back(value);
            }
            return array;
        }

        json frameToJson(const Frame& frame)
        {
            return json{
                { "valid", frame.valid },
                { "translate", floatArrayToJson(frame.translate) },
                { "rotate", floatArrayToJson(frame.rotate) },
                { "scale", frame.scale },
            };
        }

        json colliderSlotToJson(const ColliderSlot& slot)
        {
            return json{
                { "valid", slot.valid },
                { "role", slot.role },
                { "frameProxyLocal", frameToJson(slot.frameProxyLocal) },
                { "length", slot.length },
                { "radius", slot.radius },
                { "convexRadius", slot.convexRadius },
            };
        }

        json meshToJson(const MeshCapture& mesh)
        {
            json vertices = json::array();
            vertices.get_ref<json::array_t&>().reserve(mesh.verticesObjectLocal.size());
            for (const float value : mesh.verticesObjectLocal) {
                vertices.push_back(value);
            }
            return json{
                { "valid", mesh.valid },
                { "triangleCount", mesh.triangleCount },
                { "verticesObjectLocal", std::move(vertices) },
                { "aabbMinObjectLocal", floatArrayToJson(mesh.aabbMinObjectLocal) },
                { "aabbMaxObjectLocal", floatArrayToJson(mesh.aabbMaxObjectLocal) },
                { "areaCentroidObjectLocal", floatArrayToJson(mesh.areaCentroidObjectLocal) },
                { "nodeInObjectRoot", frameToJson(mesh.nodeInObjectRoot) },
            };
        }

        json physicsToJson(const PhysicsCapture& physics)
        {
            return json{
                { "valid", physics.valid },
                { "bodyId", physics.bodyId },
                { "mass", physics.mass },
                { "bodyInObjectNode", frameToJson(physics.bodyInObjectNode) },
                { "hasCenterOfMass", physics.hasCenterOfMass },
                { "comTrusted", physics.comTrusted },
                { "comObjectLocal", floatArrayToJson(physics.comObjectLocal) },
            };
        }

        json fingerContactsToJson(const std::array<FingerContactCapture, kFingerCount>& contacts)
        {
            json array = json::array();
            for (const auto& contact : contacts) {
                array.push_back(json{
                    { "touching", contact.touching },
                    { "pointObjectLocal", floatArrayToJson(contact.pointObjectLocal) },
                    { "normalObjectLocal", floatArrayToJson(contact.normalObjectLocal) },
                    { "curl", contact.curl },
                });
            }
            return array;
        }

        json seatToJson(const SeatCapture& seat)
        {
            return json{
                { "valid", seat.valid },
                { "objectProxyLocal", frameToJson(seat.objectProxyLocal) },
                { "shapeClass", seat.shapeClass },
                { "elongationRatio", seat.elongationRatio },
                { "secondElongationRatio", seat.secondElongationRatio },
                { "alignmentAngleDegrees", seat.alignmentAngleDegrees },
                { "alignmentReason", seat.alignmentReason },
                { "rollAngleDegrees", seat.rollAngleDegrees },
                { "rollReason", seat.rollReason },
                { "depthGameUnits", seat.depthGameUnits },
                { "depthOffsetGameUnits", seat.depthOffsetGameUnits },
                { "depthReason", seat.depthReason },
                { "penetrationBackstopGameUnits", seat.penetrationBackstopGameUnits },
                { "penetrationBackstopReason", seat.penetrationBackstopReason },
                { "gripPointObjectLocal", floatArrayToJson(seat.gripPointObjectLocal) },
                { "pivotProxyLocal", floatArrayToJson(seat.pivotProxyLocal) },
                { "seatMode", seat.seatMode },
                { "pivotAuthoritySource", seat.pivotAuthoritySource },
            };
        }

        json tuningToJson(const TuningCapture& tuning)
        {
            return json{
                { "seatDepthMaxGameUnits", tuning.seatDepthMaxGameUnits },
                { "seatDepthFootprintRadiusGameUnits", tuning.seatDepthFootprintRadiusGameUnits },
                { "seatPenetrationBackstopFootprintRadiusGameUnits", tuning.seatPenetrationBackstopFootprintRadiusGameUnits },
                { "seatDepthSkinGameUnits", tuning.seatDepthSkinGameUnits },
                { "gripInsetGameUnits", tuning.gripInsetGameUnits },
                { "pullPresentationMinElongationRatio", tuning.pullPresentationMinElongationRatio },
                { "pullPresentationGripAxisTiltDegrees", tuning.pullPresentationGripAxisTiltDegrees },
                { "seatRollMinSecondElongationRatio", tuning.seatRollMinSecondElongationRatio },
                { "pocketDepthGameUnits", tuning.pocketDepthGameUnits },
                { "pocketRadiusGameUnits", tuning.pocketRadiusGameUnits },
            };
        }

        json handToJson(const HandCapture& hand)
        {
            json fingerSegments = json::array();
            for (const auto& slot : hand.fingerSegments) {
                fingerSegments.push_back(colliderSlotToJson(slot));
            }

            json j{
                { "acquisition", hand.acquisition },
                { "objectProxyLocal", frameToJson(hand.objectProxyLocal) },
                { "objectScale", hand.objectScale },
                { "mesh", meshToJson(hand.mesh) },
                { "physics", physicsToJson(hand.physics) },
                { "palm", colliderSlotToJson(hand.palm) },
                { "fingerSegments", std::move(fingerSegments) },
                { "fingerContacts", fingerContactsToJson(hand.fingerContacts) },
                { "fingerCurls", floatArrayToJson(hand.fingerCurls) },
                { "seat", seatToJson(hand.seat) },
                { "tuning", tuningToJson(hand.tuning) },
            };
            if (hand.hasFingerJointValues) {
                j["fingerJointValues"] = floatArrayToJson(hand.fingerJointValues);
            }
            return j;
        }
    }

    std::string serialize(const SavedGrabCaptureFile& file)
    {
        json document{
            { "format", file.formatVersion },
            { "object", json{ { "plugin", file.object.plugin }, { "id", formIdToHex(file.object.localFormId) } } },
            { "objectName", file.objectName },
            { "hand", file.hand },
            { "rockVersion", file.rockVersion },
            { "capturedUtc", file.capturedUtc },
        };
        if (file.capture.present) {
            document["capture"] = handToJson(file.capture);
        }
        // Two-space indent: captures are large but must stay diffable and
        // readable when a single object's ground truth is inspected by hand.
        return document.dump(2);
    }
}
