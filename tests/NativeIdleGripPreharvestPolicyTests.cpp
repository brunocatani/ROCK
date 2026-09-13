#include "physics-interaction/weapon/NativeIdleGripPreharvestPolicy.h"

#include <array>

int main()
{
    using namespace rock::native_idle_grip_preharvest_policy;

    static_assert(!selectFirstPersonGraph(0, 0, 0).valid);
    static_assert(!selectFirstPersonGraph(1, 2, 2).valid);
    static_assert(!selectFirstPersonGraph(2, 1, 2).valid);
    static_assert(!selectFirstPersonGraph(2, 2, 1).valid);
    static_assert(selectFirstPersonGraph(2, 2, 2).valid);
    static_assert(selectFirstPersonGraph(2, 2, 2).graphIndex == 1);

    static_assert(shouldStartNativeIdleHarvest(false, false, false, 0x1234));
    static_assert(shouldStartNativeIdleHarvest(true, false, false, 0x1234));
    static_assert(!shouldStartNativeIdleHarvest(true, true, false, 0x1234));
    static_assert(shouldStartNativeIdleHarvest(true, true, true, 0x1234));
    static_assert(!shouldStartNativeIdleHarvest(true, true, true, 0));

    static_assert(findTransformTrackForBone(3, 8, {}) == 3);
    static_assert(findTransformTrackForBone(8, 8, {}) == -1);
    static_assert(findTransformTrackForBone(-1, 8, {}) == -1);

    constexpr std::array<std::int16_t, 5> mappedTracks{ 4, 7, 2, 9, 1 };
    static_assert(findTransformTrackForBone(2, 5, mappedTracks) == 2);
    static_assert(findTransformTrackForBone(8, 5, mappedTracks) == -1);
    static_assert(findTransformTrackForBone(2, 6, mappedTracks) == -1);

    constexpr std::array<std::int16_t, 6> parents{ -1, 0, 1, 2, 3, 3 };
    static_assert(weaponIsDirectChildOfHand(4, 3, parents));
    static_assert(!weaponIsDirectChildOfHand(4, 2, parents));
    static_assert(!weaponIsDirectChildOfHand(6, 3, parents));

    static_assert(animationResourceState(0x30000000u) == 3u);
    static_assert(animationResourceState(0x4FFFFFFFu) == 4u);
    static_assert(animationResourceCanExposeData(0x30000000u));
    static_assert(animationResourceCanExposeData(0x40000000u));
    static_assert(!animationResourceCanExposeData(0x20000000u));
    static_assert(!animationResourceCanExposeData(0x50000000u));

    static_assert(clipPathHasStem("UMPAnims\\VerticalGrip\\WPNIdleReady.hkx", "WPNIdleReady"));
    static_assert(clipPathHasStem("Actors\\AKsAR15s\\Character\\_1stPerson\\Animations\\SVD\\WPNIdleReady.hkx", "WPNIdleReady"));
    static_assert(clipPathHasStem("Animations/Weapons/wpnidle.HKX", "WPNIdle"));
    static_assert(clipPathHasStem("WPNIdle", "wpnidle"));
    static_assert(!clipPathHasStem("UMPAnims\\VerticalGrip\\WPNIdleReady.hkx", "WPNIdle"));
    static_assert(!clipPathHasStem("WPNIdleReadyExtra.hkx", "WPNIdleReady"));

    static_assert(idleClipPriority("SREP/WPNIdleReady.hkx") == IdleClipPriority::IdleReady);
    static_assert(idleClipPriority("SVD\\WPNIdle.HKX") == IdleClipPriority::Idle);
    static_assert(idleClipPriority("SREP/WPNFire.hkx") == IdleClipPriority::None);
    // The first-person melee idles were previously rejected before any fingers
    // could be sampled. Keep exact basename matching and firearm precedence.
    static_assert(idleClipPriority("Actors/Character/_1stPerson/Animations/1HM/Idle.hkx") == IdleClipPriority::GenericIdle);
    static_assert(idleClipPriority("2HM\\Idle.HKX") == IdleClipPriority::GenericIdle);
    static_assert(idleClipPriority("Board/Idle.hkx") == IdleClipPriority::GenericIdle);
    static_assert(idleClipPriority("1HM/WPNIdleSightedWobble.hkx") == IdleClipPriority::None);
    static_assert(idleClipPriority("1HM/Idle.hkx") < idleClipPriority("Rifle/WPNIdle.hkx"));
    static_assert(idleClipPriority("Rifle/WPNIdle.hkx") < idleClipPriority("Rifle/WPNIdleReady.hkx"));
    static_assert(sameClipPath("Actors/Character/SREP/WPNIdleReady.hkx", "actors\\character\\srep\\wpnidleready.HKX"));
    static_assert(!sameClipPath("SREP/WPNIdleReady.hkx", "SVD/WPNIdleReady.hkx"));

    static_assert(persistenceSampleTimeSeconds(10.0f, 0) == 0.0f);
    static_assert(persistenceSampleTimeSeconds(10.0f, 2) == 4.0f);
    static_assert(persistenceSampleTimeSeconds(10.0f, 4) == 8.0f);
    static_assert(stableForPersistence(5, 2.0f, 0.01f, 0.1f, 0.01f, 0.2f, 0.0001f));
    static_assert(!stableForPersistence(4, 2.0f, 0.01f, 0.1f, 0.01f, 0.2f, 0.0001f));
    static_assert(!stableForPersistence(5, 2.0f, 0.06f, 0.1f, 0.01f, 0.2f, 0.0001f));
    static_assert(!stableForPersistence(5, 2.0f, 0.01f, 0.6f, 0.01f, 0.2f, 0.0001f));
    static_assert(!stableForPersistence(5, 2.0f, 0.01f, 0.1f, 0.01f, 1.1f, 0.0001f));

    // Support hand tolerates idle sway (1gu / 3deg) but not travel; the
    // finger and scale limits stay as strict as the primary's.
    static_assert(supportStableForPersistence(5, 2.0f, 0.8f, 2.5f, 0.01f, 0.2f, 0.0001f));
    static_assert(!supportStableForPersistence(4, 2.0f, 0.8f, 2.5f, 0.01f, 0.2f, 0.0001f));
    static_assert(!supportStableForPersistence(5, 2.0f, 1.2f, 2.5f, 0.01f, 0.2f, 0.0001f));
    static_assert(!supportStableForPersistence(5, 2.0f, 0.8f, 3.5f, 0.01f, 0.2f, 0.0001f));
    static_assert(!supportStableForPersistence(5, 2.0f, 0.8f, 2.5f, 0.03f, 0.2f, 0.0001f));
    static_assert(!supportStableForPersistence(5, 2.0f, 0.8f, 2.5f, 0.01f, 1.1f, 0.0001f));
    static_assert(!supportStableForPersistence(5, 2.0f, 0.8f, 2.5f, 0.01f, 0.2f, 0.002f));

    // Bone chain walk: leaf first up to the root, failing closed on a bad
    // parent, a short buffer, or a cycle.
    static_assert([] {
        constexpr std::array<std::int16_t, 6> chainParents{ -1, 0, 1, 2, 3, 3 };
        std::array<int, 8> chain{};
        const auto length = collectBoneChainToRoot(5, chainParents, chain);
        return length == 5 && chain[0] == 5 && chain[1] == 3 && chain[2] == 2 && chain[3] == 1 && chain[4] == 0;
    }());
    static_assert([] {
        constexpr std::array<std::int16_t, 6> chainParents{ -1, 0, 1, 2, 3, 3 };
        std::array<int, 8> chain{};
        return collectBoneChainToRoot(0, chainParents, chain) == 1 && chain[0] == 0;
    }());
    static_assert([] {
        constexpr std::array<std::int16_t, 6> chainParents{ -1, 0, 1, 2, 3, 3 };
        std::array<int, 8> chain{};
        return collectBoneChainToRoot(6, chainParents, chain) == 0 && collectBoneChainToRoot(-1, chainParents, chain) == 0;
    }());
    static_assert([] {
        constexpr std::array<std::int16_t, 6> chainParents{ -1, 0, 1, 2, 3, 3 };
        std::array<int, 3> shortChain{};
        return collectBoneChainToRoot(5, chainParents, shortChain) == 0;
    }());
    static_assert([] {
        constexpr std::array<std::int16_t, 4> cyclicParents{ -1, 3, 1, 2 };
        std::array<int, 8> chain{};
        return collectBoneChainToRoot(3, cyclicParents, chain) == 0;
    }());
    static_assert([] {
        constexpr std::array<std::int16_t, 3> badParents{ -1, 7, 1 };
        std::array<int, 8> chain{};
        return collectBoneChainToRoot(2, badParents, chain) == 0;
    }());

    return 0;
}
