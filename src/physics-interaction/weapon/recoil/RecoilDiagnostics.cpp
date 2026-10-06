#include "physics-interaction/weapon/TwoHandedGripInternal.h"

namespace rock
{
    void TwoHandedGrip::traceRecoilReadiness() const
    {
        if (g_rockConfig.rockImmersiveRecoil && _recoil.rightBaseValid &&
            !frik_hand_world_authority::hasCalibratedRawHandFrame(false)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Weapon recoil: right acquisition deferred; rawSource={} controller/body relation unavailable form={:08X} generation={:016X}",
                frik_hand_world_authority::rawHandSourceName(false), _recoil.equippedIdentity.formID,
                _recoil.equippedIdentity.weaponGeneration);
        }
    }

}
