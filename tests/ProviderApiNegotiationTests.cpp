#include "api/ROCKProviderApi.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace
{
    using namespace rock::provider;

    std::uint32_t g_baseCalls{ 0 };
    std::uint32_t g_extendedCalls{ 0 };
    std::uint32_t g_reportedTableBytes{ 0 };
    std::uint32_t g_reportedFeatureBits{ 0 };
    std::uint32_t g_reportedFeatureBits2{ 0 };
    std::uint32_t g_baseReturnedBytes{ sizeof(RockProviderLimitsV1) };

    bool ROCK_PROVIDER_CALL fakeGetProviderLimitsV1(
        RockProviderLimitsV1* outLimits)
    {
        ++g_baseCalls;
        assert(outLimits);
        assert(outLimits->size == sizeof(RockProviderLimitsV1));

        RockProviderLimitsV1 limits{};
        limits.featureBits = g_reportedFeatureBits;
        limits.maxFrameCallbacks = 16;
        limits.providerApiByteSize = g_reportedTableBytes;
        const auto copyBytes = g_baseReturnedBytes < sizeof(limits) ?
            g_baseReturnedBytes :
            static_cast<std::uint32_t>(sizeof(limits));
        std::memcpy(outLimits, &limits, copyBytes);
        outLimits->size = copyBytes;
        return true;
    }

    bool ROCK_PROVIDER_CALL fakeGetProviderLimitsExtV1(
        RockProviderLimitsExtV1* outLimits)
    {
        ++g_extendedCalls;
        assert(outLimits);
        assert(outLimits->size == sizeof(RockProviderLimitsExtV1));

        RockProviderLimitsExtV1 limits{};
        limits.featureBits2 = g_reportedFeatureBits2;
        limits.providerApiByteSize = g_reportedTableBytes;
        limits.maxExternalScopes = ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1;
        limits.maxTouchGrabTargets =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1;
        limits.maxTouchGrabScopes =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1;
        limits.maxTouchGrabTargetLeaseFrames =
            ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1;
        std::memcpy(outLimits, &limits, sizeof(limits));
        return true;
    }

    void resetClientState()
    {
        RockProviderApi::inst = nullptr;
        RockProviderApi::negotiatedApiVersion = 0;
        RockProviderApi::negotiatedTableByteSize = 0;
        RockProviderApi::negotiatedFeatureBits = 0;
        RockProviderApi::negotiatedFeatureBits2 = 0;
        g_baseCalls = 0;
        g_extendedCalls = 0;
        g_reportedTableBytes = 0;
        g_reportedFeatureBits = static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::FrameCallbacks);
        g_reportedFeatureBits2 = 0;
        g_baseReturnedBytes = sizeof(RockProviderLimitsV1);
    }
}

int main()
{
    using namespace rock::provider;

    RockProviderApi table{};
    table.getProviderLimitsV1 = &fakeGetProviderLimitsV1;
    table.getProviderLimitsExtV1 = &fakeGetProviderLimitsExtV1;

    resetClientState();
    RockProviderLimitsV1 baseLimits{};
    RockProviderLimitsExtV1 extendedLimits{};
    assert(!queryProviderLimitsV1(baseLimits));
    assert(!queryProviderLimitsExtV1(extendedLimits));

    RockProviderApi::inst = &table;
    RockProviderApi::negotiatedApiVersion = ROCK_PROVIDER_API_VERSION;
    RockProviderApi::negotiatedTableByteSize =
        static_cast<std::uint32_t>(
            offsetof(RockProviderApi, getProviderLimitsV1));
    assert(!queryProviderLimitsV1(baseLimits));
    assert(g_baseCalls == 0);

    RockProviderApi::negotiatedTableByteSize =
        ROCK_PROVIDER_API_V1_PRESENTED_HAND_FRAMES_TABLE_BYTES;
    g_reportedTableBytes =
        ROCK_PROVIDER_API_V1_PRESENTED_HAND_FRAMES_TABLE_BYTES;
    g_baseReturnedBytes = static_cast<std::uint32_t>(
        offsetof(RockProviderLimitsV1, providerApiByteSize) +
        sizeof(RockProviderLimitsV1::providerApiByteSize));
    std::memset(&baseLimits, 0xCD, sizeof(baseLimits));
    assert(queryProviderLimitsV1(baseLimits));
    assert(g_baseCalls == 1);
    assert(baseLimits.size == g_baseReturnedBytes);
    assert(baseLimits.maxFrameCallbacks == 16);
    assert(baseLimits.providerApiByteSize == g_reportedTableBytes);
    assert(baseLimits.maxWeaponEmitters == 0);
    assert(!queryProviderLimitsExtV1(extendedLimits));
    assert(g_extendedCalls == 0);

    const auto allNewFeatureBits =
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::ExtendedLimits) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OwnerFrameCallbacks) |
        static_cast<std::uint32_t>(RockProviderFeatureBit2V1::OffhandReservationLeases) |
        static_cast<std::uint32_t>(
            RockProviderFeatureBit2V1::NativeVatsVansInputSuppression);
    RockProviderApi::negotiatedTableByteSize = sizeof(RockProviderApi);
    RockProviderApi::negotiatedFeatureBits =
        static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::EquippedWeaponHandRequest);
    RockProviderApi::negotiatedFeatureBits2 = allNewFeatureBits;
    g_reportedTableBytes = sizeof(RockProviderApi);
    g_reportedFeatureBits =
        static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::EquippedWeaponHandRequest);
    g_reportedFeatureBits2 = allNewFeatureBits;
    g_baseReturnedBytes = sizeof(RockProviderLimitsV1);
    assert(queryProviderLimitsExtV1(extendedLimits));
    assert(g_extendedCalls == 1);
    assert(extendedLimits.size == sizeof(RockProviderLimitsExtV1));
    assert(extendedLimits.providerApiByteSize == sizeof(RockProviderApi));
    assert(extendedLimits.maxExternalScopes ==
           ROCK_PROVIDER_MAX_EXTERNAL_SCOPES_V1);
    assert(extendedLimits.maxTouchGrabTargets ==
           ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGETS_V1);
    assert(extendedLimits.maxTouchGrabScopes ==
           ROCK_PROVIDER_MAX_TOUCH_GRAB_SCOPES_V1);
    assert(extendedLimits.maxTouchGrabTargetLeaseFrames ==
           ROCK_PROVIDER_MAX_TOUCH_GRAB_TARGET_LEASE_FRAMES_V1);
    assert(supportsExtendedLimitsV1());
    assert(supportsOwnerFrameCallbacksV1());
    assert(supportsOffhandReservationLeasesV1());
    assert(supportsNativeVatsVansInputSuppressionV1());
    assert(supportsEquippedWeaponHandRequestV1());

    g_reportedFeatureBits = 0;
    assert(!supportsEquippedWeaponHandRequestV1());
    g_reportedFeatureBits =
        static_cast<std::uint32_t>(
            RockProviderFeatureBitV1::EquippedWeaponHandRequest);
    g_reportedTableBytes =
        ROCK_PROVIDER_API_V1_EQUIPPED_WEAPON_HAND_REQUEST_TABLE_BYTES - 1;
    assert(!supportsEquippedWeaponHandRequestV1());
    g_reportedTableBytes = sizeof(RockProviderApi);

    RockProviderApi::negotiatedFeatureBits2 &=
        ~static_cast<std::uint32_t>(
            RockProviderFeatureBit2V1::NativeVatsVansInputSuppression);
    assert(!supportsNativeVatsVansInputSuppressionV1());
    RockProviderApi::negotiatedFeatureBits2 = allNewFeatureBits;
    RockProviderApi::negotiatedTableByteSize =
        ROCK_PROVIDER_API_V1_HAND_INPUT_SUPPRESSION_TABLE_BYTES - 1;
    assert(!supportsNativeVatsVansInputSuppressionV1());
    RockProviderApi::negotiatedTableByteSize = sizeof(RockProviderApi);

    RockProviderApi::negotiatedFeatureBits2 &=
        ~static_cast<std::uint32_t>(
            RockProviderFeatureBit2V1::OffhandReservationLeases);
    assert(!supportsOffhandReservationLeasesV1());
    RockProviderApi::negotiatedFeatureBits2 = allNewFeatureBits;
    RockProviderApi::negotiatedTableByteSize =
        ROCK_PROVIDER_API_V1_OFFHAND_RESERVATION_LEASES_TABLE_BYTES - 1;
    assert(!supportsOffhandReservationLeasesV1());

    RockProviderApi::negotiatedTableByteSize = 0;
    g_reportedTableBytes =
        ROCK_PROVIDER_API_V1_EXTENDED_LIMITS_TABLE_BYTES - 1;
    g_baseCalls = 0;
    g_extendedCalls = 0;
    assert(!queryProviderLimitsExtV1(extendedLimits));
    assert(g_baseCalls == 1);
    assert(g_extendedCalls == 0);

    g_reportedTableBytes = sizeof(RockProviderApi);
    g_reportedFeatureBits2 = allNewFeatureBits;
    assert(queryProviderLimitsExtV1(extendedLimits));
    assert(g_baseCalls == 2);
    assert(g_extendedCalls == 1);
    assert(supportsOffhandReservationLeasesV1());
    assert(supportsNativeVatsVansInputSuppressionV1());

    resetClientState();
    return 0;
}
