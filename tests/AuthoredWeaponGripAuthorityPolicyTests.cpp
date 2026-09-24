#include "physics-interaction/weapon/AuthoredWeaponGripAuthorityPolicy.h"

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
    static_assert(shouldAcceptPublication(true, PublicationAuthority::FreshNativeIdle, PublicationAuthority::FreshNativeIdle));
    static_assert(shouldAcceptPublication(true, PublicationAuthority::LiveEquippedGraph, PublicationAuthority::FreshNativeIdle));
    static_assert(!shouldAcceptPublication(true, PublicationAuthority::FreshNativeIdle, PublicationAuthority::LiveEquippedGraph));

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

    return 0;
}
