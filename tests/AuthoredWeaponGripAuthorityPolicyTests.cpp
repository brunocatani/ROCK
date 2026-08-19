#include "physics-interaction/weapon/authored_grip/AuthoredWeaponGripAuthorityPolicy.h"

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
