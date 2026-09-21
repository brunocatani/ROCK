#include "physics-interaction/weapon/NativeIdleGripPreharvestPolicy.h"
#include "physics-interaction/weapon/PipeFiringGripPolicy.h"

#include <array>
#include <iostream>
#include <limits>

namespace
{
    using PersistedTransform = rock::authored_weapon_grip_cache::PersistedTransform;

    struct SampleTransform
    {
        struct { float entry[3][3]{}; } rotate;
        struct { float x{}, y{}, z{}; } translate;
        float scale{ 1.0f };

        SampleTransform() = default;
        explicit SampleTransform(const PersistedTransform& source)
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    rotate.entry[r][c] = source.rotate[r * 3 + c];
            translate = { source.translate[0], source.translate[1], source.translate[2] };
            scale = source.scale;
        }
    };

    struct SampleFingers
    {
        std::array<SampleTransform, 15> localTransforms{};
        std::uint16_t enabledMask{ 0x7FFF };
    };

    bool checkPipePoseSelection()
    {
        using namespace rock::pipe_firing_grip_policy;
        bool ok = true;
        const auto check = [&ok](bool condition, const char* message) {
            if (!condition) std::cerr << message << '\n';
            ok &= condition;
        };
        for (const auto& pose : kVanillaPoses) {
            SampleFingers fingers{};
            for (std::size_t i = 0; i < fingers.localTransforms.size(); ++i)
                fingers.localTransforms[i] = SampleTransform{ pose.fingers[i] };
            check(recognizesVanilla(SampleTransform{ pose.hand }, fingers), "existing pipe pose lost its correction");
        }

        // Native Syringer idle capture, 2026-09-20: same pipe-rifle pose,
        // independent clip path and sampling roundoff in hand/finger locals.
        constexpr std::array<PersistedTransform, 16> syringerPose{{
            { { -0.00433158875f, 0.702639639f, -0.711532652f, 0.998639047f, 0.0400214195f, 0.0334417224f, 0.0519739389f, -0.710419416f, -0.701856613f }, { 2.08679962f, -9.78137493f, 2.116606f }, 1.0f },
            { { 0.625230908f, -0.582359731f, -0.519560933f, -0.770904839f, -0.357097268f, -0.527434707f, 0.121622935f, 0.730300426f, -0.672212124f }, { 1.58291626f, -1.26266479f, -1.85335541f }, 1.0f },
            { { 0.731590867f, -0.681743979f, 1.41930968E-07f, 0.681743979f, 0.731590867f, -3.60496614E-07f, 1.41930968E-07f, 3.60496614E-07f, 1.0f }, { 3.56959915f, 0.0f, 0.0f }, 1.0f },
            { { 0.954619527f, -0.297828108f, -3.10334997E-07f, 0.297828108f, 0.954619527f, 5.82259005E-08f, 2.78910534E-07f, -1.48010059E-07f, 1.0f }, { 2.40181732f, 0.0f, 0.0f }, 1.0f },
            { { 0.968315899f, 0.0758484975f, 0.237931341f, -0.106018625f, 0.987495899f, 0.116669938f, -0.226106972f, -0.13819851f, 0.964249372f }, { 7.50128937f, 0.430534363f, -2.277771f }, 1.0f },
            { { 0.930351973f, -0.366667718f, 0.000126815081f, 0.366667658f, 0.930351973f, 0.000238042514f, -0.000205265154f, -0.000174964327f, 0.99999994f }, { 3.01819611f, 0.0f, 0.0f }, 1.0f },
            { { 0.845665336f, -0.533713579f, 3.94246911E-07f, 0.533713579f, 0.845665336f, 8.89588762E-07f, -8.08186485E-07f, -5.41879501E-07f, 1.0f }, { 1.85023499f, 0.0f, 0.0f }, 1.0f },
            { { 0.425145268f, -0.901880383f, 0.0765716657f, 0.883396268f, 0.431876898f, 0.18191573f, -0.197135776f, -0.00969750434f, 0.980328262f }, { 7.59570312f, 0.621257782f, -0.457489014f }, 1.0f },
            { { 0.617354751f, -0.786684811f, -3.81397854E-07f, 0.786684811f, 0.617354751f, -7.48905677E-07f, 8.24610538E-07f, 1.6230058E-07f, 1.0f }, { 3.0916481f, 0.0f, 0.0f }, 1.0f },
            { { 0.482070148f, -0.876132667f, -3.96584852E-07f, 0.876132667f, 0.482070148f, 3.194817E-07f, -8.87266367E-08f, -5.01473494E-07f, 1.0f }, { 2.18797684f, 0.0f, 0.0f }, 1.0f },
            { { 0.228204489f, -0.973165631f, -0.0295165926f, 0.915212393f, 0.204075515f, 0.347475678f, -0.33212781f, -0.106309466f, 0.937224329f }, { 7.46389771f, 0.350522995f, 1.43872833f }, 1.0f },
            { { 0.854495645f, -0.519458592f, -4.2184837E-07f, 0.519458592f, 0.854495645f, -9.2478183E-07f, 8.40853374E-07f, 5.71089345E-07f, 1.0f }, { 2.66441536f, 0.0f, 0.0f }, 1.0f },
            { { 0.551069736f, -0.834459245f, -1.3203163E-08f, 0.834459245f, 0.551069736f, -2.2707286E-07f, 1.96758904E-07f, 1.14115487E-07f, 1.0f }, { 1.89974594f, 0.0f, 0.0f }, 1.0f },
            { { 0.294671595f, -0.951470673f, -0.0887271464f, 0.8906793f, 0.239827454f, 0.386229962f, -0.346207201f, -0.19283846f, 0.918125212f }, { 6.63713074f, -0.357089996f, 3.01842499f }, 1.0f },
            { { 0.813540816f, -0.581507862f, -3.53911673E-07f, 0.581507862f, 0.813540816f, -5.55549889E-07f, 6.10978191E-07f, 2.46160084E-07f, 1.0f }, { 2.23826027f, 0.0f, 0.0f }, 1.0f },
            { { 0.64481926f, -0.764335215f, -4.19130401E-07f, 0.764335215f, 0.64481926f, 1.89202854E-06f, -1.17588024E-06f, -1.54037264E-06f, 1.0f }, { 1.665905f, 0.0f, 0.0f }, 1.0f },
        }};
        SampleTransform hand{ syringerPose[0] };
        SampleFingers fingers{};
        for (std::size_t i = 0; i < fingers.localTransforms.size(); ++i)
            fingers.localTransforms[i] = SampleTransform{ syringerPose[i + 1] };
        constexpr auto syringerClip = "actors\\character\\_1stperson\\animations\\syringer\\WPNIDLEready.HKX";
        constexpr auto copiedClip = "Actors/Character/_1stPerson/Animations/ModdedWeapon/CopiedIdle.hkx";
        check(recognizesVanilla(hand, fingers), "recorded Syringer pose did not qualify");
        check(requiresFreshSample(syringerClip, hand, fingers), "Syringer cache skipped fresh qualification");
        check(requiresFreshSample(copiedClip, hand, fingers), "renamed shared pose skipped fresh qualification");

        auto shiftedHand = hand;
        shiftedHand.translate.x += 0.001f;
        check(!recognizesVanilla(shiftedHand, fingers), "different hand position received pipe correction");
        check(requiresFreshSample(kVanillaPoses[0].clip, shiftedHand, fingers), "pipe replacement cache skipped refresh");
        check(requiresFreshSample(syringerClip, shiftedHand, fingers), "Syringer replacement cache skipped refresh");
        check(!requiresFreshSample(copiedClip, shiftedHand, fingers), "unrelated custom pose lost cache reuse");
        shiftedHand = hand;
        shiftedHand.rotate.entry[0][0] += 0.001f;
        check(!recognizesVanilla(shiftedHand, fingers), "different hand rotation received pipe correction");
        shiftedHand = hand;
        shiftedHand.scale += 0.001f;
        check(!recognizesVanilla(shiftedHand, fingers), "different hand scale received pipe correction");
        shiftedHand = hand;
        shiftedHand.translate.x = std::numeric_limits<float>::quiet_NaN();
        check(!recognizesVanilla(shiftedHand, fingers), "nonfinite hand received pipe correction");

        for (std::size_t i = 0; i < fingers.localTransforms.size(); ++i) {
            auto changed = fingers;
            changed.enabledMask &= ~(1u << i);
            check(!recognizesVanilla(hand, changed), "incomplete finger pose received pipe correction");
            changed = fingers;
            changed.localTransforms[i].translate.x += 0.001f;
            check(!recognizesVanilla(hand, changed), "changed finger position received pipe correction");
            changed = fingers;
            changed.localTransforms[i].rotate.entry[0][0] += 0.001f;
            check(!recognizesVanilla(hand, changed), "changed finger rotation received pipe correction");
            changed = fingers;
            changed.localTransforms[i].scale = std::numeric_limits<float>::infinity();
            check(!recognizesVanilla(hand, changed), "nonfinite finger received pipe correction");
        }
        return ok;
    }
}

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
    static_assert(idleClipPriority("2HMWide/Idle.hkx") == IdleClipPriority::GenericIdle);
    static_assert(idleClipPriority("H2H/Idle.hkx") == IdleClipPriority::GenericIdle);
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

    // A common spine track does not constitute an offhand pose. An exclusive
    // arm track does; malformed mapping or skeleton data remains unqualified.
    constexpr std::array<std::int16_t, 6> armParents{ -1, 0, 1, 1, 2, 3 };
    constexpr std::array<std::int16_t, 4> primaryTracks{ 0, 1, 2, 4 };
    constexpr std::array<std::int16_t, 5> bothArmTracks{ 0, 1, 2, 4, 3 };
    static_assert(supportBranchHasAnimation(4, 5, armParents, 4, primaryTracks) == false);
    static_assert(supportBranchHasAnimation(4, 5, armParents, 5, bothArmTracks) == true);
    static_assert(supportBranchHasAnimation(4, 5, armParents, 6, {}) == true);
    static_assert(!supportBranchHasAnimation(4, 5, armParents, 5, primaryTracks).has_value());
    static_assert(!supportBranchHasAnimation(4, 6, armParents, 4, primaryTracks).has_value());
    static_assert(!supportBranchHasAnimation(4, 5, armParents, 0, {}).has_value());
    constexpr std::array<std::int16_t, 6> cyclicArm{ -1, 0, 1, 5, 2, 3 };
    static_assert(!supportBranchHasAnimation(4, 5, cyclicArm, 6, {}).has_value());
    return checkPipePoseSelection() ? 0 : 1;
}
