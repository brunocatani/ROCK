#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiNode.h"

#include <array>
#include <cmath>
#include <string_view>

namespace rock::vanilla_weapon_grip_frame
{
    void correctPipeWrist(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* model, RE::NiTransform& hand)
    {
        if (!weapon || !matchesVanillaPipePose(weapon->formID, hand)) return;
        if (frik_weapon_offset_cache::findCustomGripOverride(weapon, model).found) return;
        const auto native = frik_weapon_offset_cache::findPrimaryWeaponOffset(weapon, model);
        if (!native.found || native.source != frik_weapon_offset_cache::OffsetSource::EmbeddedResource) return;
        // Approved vanilla-only wrist posture. The baseline weapon carrier
        // already uses this placement; no Weapon rotation or translation is
        // taken from the preset here. Explicit files keep their precedence.
        const auto nativeHand = transform_math::invertTransform(native.offset);
        hand = withWristRotationAtPalm(hand, nativeHand.rotate,
            authoredHandspaceToRawHandspace(computeGrabLegacyPalmPivotAHandspacePosition(false)),
            [](const auto& frame, const auto& point) { return transform_math::localPointToWorld(frame, point); });
        ROCK_LOG_SAMPLE_DEBUG(Animation, 2000, "Vanilla pipe wrist corrected at authored palm formID={:08X}", weapon->formID);
    }

    bool resolveModelTranslation(const std::uint32_t formId,
        const RE::NiAVObject* model, RE::NiPoint3& outTranslation)
    {
        outTranslation = {};
        if (!hasVanillaModelRegistration(formId) || !model) return true;

        const RE::NiAVObject* receiver = nullptr;
        const RE::NiAVObject* marker = nullptr;
        bool ambiguous = false;
        const auto traversal = weapon_scene::visitScene(
            const_cast<RE::NiAVObject*>(model), [&](const RE::NiAVObject* node) {
                const std::string_view name{ node->name.c_str() ? node->name.c_str() : "" };
                if (name == "TGunReceiver") {
                    ambiguous = ambiguous || receiver != nullptr;
                    receiver = node;
                } else if (name == "WeaponOffset") {
                    ambiguous = ambiguous || marker != nullptr;
                    marker = node;
                }
                return true;
            });
        // Replacement models without this vanilla assembly are outside this
        // correction. A partial/ambiguous vanilla assembly cannot define a grip.
        if (!traversal.truncated && !receiver && !marker) return true;
        if (!traversal.truncated && !ambiguous && receiver && !marker &&
            std::abs(receiver->local.translate.x) <= 0.001f &&
            std::abs(receiver->local.translate.y) <= 0.001f &&
            std::abs(receiver->local.translate.z) <= 0.001f) {
            // The undisplaced loose model needs no registration adjustment.
            return true;
        }
        bool valid = !traversal.truncated && !ambiguous && receiver && marker &&
            receiver->parent && receiver->parent == marker->parent;
        if (valid) {
            // The SMG's receiver and independent WeaponOffset marker carry
            // the same model registration translation. It survives both menu
            // and gameplay states; the idle Weapon track does not include it.
            // Read that registration rather than hard-coding the observed Y.
            valid = registrationAgrees(receiver->local, marker->local);
        }
        if (!valid) {
            ROCK_LOG_SAMPLE_WARN(Animation, 1000, "SMG authored grip unavailable: ambiguous or mismatched model registration");
            return false;
        }

        std::array<const RE::NiAVObject*, 16> chain{};
        std::size_t count = 0;
        const auto* parent = static_cast<const RE::NiAVObject*>(receiver->parent);
        while (parent && parent != model && count < chain.size()) {
            chain[count++] = parent;
            parent = parent->parent;
        }
        if (parent != model) {
            ROCK_LOG_SAMPLE_WARN(Animation, 1000, "SMG authored grip unavailable: model registration parent chain is incomplete");
            return false;
        }
        auto parentInModel = transform_math::makeIdentityTransform<RE::NiTransform>();
        while (count) parentInModel = transform_math::composeTransforms(parentInModel, chain[--count]->local);
        // Transform the displacement as a vector: wrapper translations are
        // already represented by the model frame and must not be added twice.
        const auto shifted = transform_math::localPointToWorld(parentInModel, receiver->local.translate);
        outTranslation = shifted - parentInModel.translate;
        if (!std::isfinite(outTranslation.x) || !std::isfinite(outTranslation.y) || !std::isfinite(outTranslation.z)) {
            outTranslation = {};
            ROCK_LOG_SAMPLE_WARN(Animation, 1000, "SMG authored grip unavailable: model registration is non-finite");
            return false;
        }
        return true;
    }
}
