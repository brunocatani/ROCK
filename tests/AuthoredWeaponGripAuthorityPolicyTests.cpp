#include "physics-interaction/weapon/AuthoredWeaponGripAuthorityPolicy.h"
#include "physics-interaction/weapon/PipeFiringGripPolicy.h"
#include <limits>

int main()
{
    using namespace rock::authored_weapon_grip_authority_policy;

    static_assert(completeFiringFingerPose(0x7FFF));
    static_assert(!completeFiringFingerPose(0x3FFF));
    static_assert(!completeFiringFingerPose(0));
    static_assert(publicationHasRequiredFingerPose(false, false));
    static_assert(publicationHasRequiredFingerPose(false, true));
    static_assert(publicationHasRequiredFingerPose(true, true));
    static_assert(!publicationHasRequiredFingerPose(true, false));

    static_assert(shouldAcceptPublication(false, PublicationAuthority::FreshNativeIdle, PublicationAuthority::LiveEquippedGraph));
    static_assert(shouldAcceptPublication(true, PublicationAuthority::LiveEquippedGraph, PublicationAuthority::LiveEquippedGraph));
    static_assert(shouldAcceptPublication(true, PublicationAuthority::LiveEquippedGraph, PublicationAuthority::PersistedNativeIdle));
    static_assert(shouldAcceptPublication(true, PublicationAuthority::PersistedNativeIdle, PublicationAuthority::FreshNativeIdle));
    static_assert(shouldAcceptPublication(true, PublicationAuthority::FreshNativeIdle, PublicationAuthority::FreshNativeIdle));
    static_assert(!shouldAcceptPublication(true, PublicationAuthority::FreshNativeIdle, PublicationAuthority::PersistedNativeIdle));
    static_assert(!shouldAcceptPublication(true, PublicationAuthority::PersistedNativeIdle, PublicationAuthority::LiveEquippedGraph));

    struct MockRotation
    {
        float entry[3][3];
    };
    struct MockTranslation
    {
        float x;
        float y;
        float z;
    };
    struct MockTransform
    {
        MockRotation rotate;
        MockTranslation translate;
    };
    constexpr MockTransform identityRelation{
        { { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } } },
        { 5.0f, -3.0f, 8.0f },
    };
    static_assert(handRelationValueMatches(identityRelation, identityRelation));
    static_assert([=] {
        auto swayed = identityRelation;
        swayed.rotate.entry[0][1] += 0.03f;
        swayed.translate.x += 0.5f;
        return handRelationValueMatches(identityRelation, swayed);
    }());
    static_assert([=] {
        auto rotatedAway = identityRelation;
        rotatedAway.rotate.entry[0][0] = -1.0f;
        return !handRelationValueMatches(identityRelation, rotatedAway);
    }());
    static_assert([=] {
        auto translatedAway = identityRelation;
        translatedAway.translate.z += 2.0f;
        return !handRelationValueMatches(identityRelation, translatedAway);
    }());

    static_assert(selectLookup(true, true, false, 2, 3) == LookupSelection::ExactVariant);
    static_assert(selectLookup(true, false, true, 1, 2) == LookupSelection::SoleNativeIdleVariant);
    static_assert(selectLookup(true, false, false, 1, 2) == LookupSelection::ExactVariant);
    static_assert(selectLookup(false, false, false, 1, 2) == LookupSelection::SoleNativeIdleVariant);
    static_assert(selectLookup(true, false, true, 2, 3) == LookupSelection::ExactVariant);
    static_assert(selectLookup(false, false, false, 2, 3) == LookupSelection::None);
    static_assert(selectLookup(false, false, false, 0, 1) == LookupSelection::SoleFormVariant);
    static_assert(selectLookup(false, false, false, 0, 0) == LookupSelection::None);

    // Vanilla pipe offsets are physical-right defaults. Replacements keep
    // authority even when they reuse the original idle path.
    namespace pipe = rock::pipe_firing_grip_policy;
    static_assert(pipe::isPipe(0x24F55) && pipe::isPipe(0x14831A) && pipe::isPipe(0x14831B));
    static_assert(!pipe::isPipe(0x4822));
    static_assert(pipe::useCompiledDefault(true, false, true));
    static_assert(!pipe::useCompiledDefault(true, true, true));
    static_assert(!pipe::useCompiledDefault(true, false, false));
    static_assert(!pipe::useCompiledDefault(false, false, true));
    struct PipeTransform { MockRotation rotate; MockTranslation translate; float scale; };
    struct PipeFingers { std::array<PipeTransform, 15> localTransforms; unsigned enabledMask; };
    const auto copyTransform = [](const pipe::Transform& src) {
        PipeTransform dst{};
        dst.translate = {src.translate[0], src.translate[1], src.translate[2]};
        dst.scale = src.scale;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) dst.rotate.entry[r][c] = src.rotate[r * 3 + c];
        return dst;
    };
    const auto calibration = pipe::weaponInHand<PipeTransform>();
    if (!pipe::matches(calibration, pipe::kWeaponInHand)) return 78;
    if (!pipe::isPromotedCalibration(0x24F55, false, calibration)) return 79;
    if (pipe::isPromotedCalibration(0x24F55, true, calibration)) return 80;
    if (pipe::isPromotedCalibration(0x4822, false, calibration)) return 81;
    auto editedCalibration = calibration;
    editedCalibration.translate.z += 0.25f;
    if (pipe::isPromotedCalibration(0x24F55, false, editedCalibration)) return 82;
    for (const auto& vanilla : pipe::kVanillaPoses) {
        const auto hand = copyTransform(vanilla.hand);
        PipeFingers fingers{};
        fingers.enabledMask = 0x7FFF;
        for (std::size_t i = 0; i < fingers.localTransforms.size(); ++i)
            fingers.localTransforms[i] = copyTransform(vanilla.fingers[i]);
        if (!pipe::recognizesVanilla(0x24F55, vanilla.clip, hand, fingers)) return 70;
        if (pipe::recognizesVanilla(0x4822, vanilla.clip, hand, fingers)) return 71;
        if (pipe::recognizesVanilla(0x24F55, "Custom/WPNIdleReady.hkx", hand, fingers)) return 72;
        auto replacement = hand;
        replacement.translate.z += 0.1f;
        if (pipe::recognizesVanilla(0x24F55, vanilla.clip, replacement, fingers)) return 73;
        replacement = hand;
        replacement.rotate.entry[0][1] += 0.01f;
        if (pipe::recognizesVanilla(0x24F55, vanilla.clip, replacement, fingers)) return 74;
        replacement = hand;
        replacement.scale = std::numeric_limits<float>::quiet_NaN();
        if (pipe::recognizesVanilla(0x24F55, vanilla.clip, replacement, fingers)) return 75;
        auto customFingers = fingers;
        customFingers.localTransforms[14].rotate.entry[1][0] += 0.01f;
        if (pipe::recognizesVanilla(0x24F55, vanilla.clip, hand, customFingers)) return 76;
        fingers.enabledMask = 0x3FFF;
        if (pipe::recognizesVanilla(0x24F55, vanilla.clip, hand, fingers)) return 77;
    }

    return 0;
}
