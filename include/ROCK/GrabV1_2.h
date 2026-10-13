#pragma once
#include "GrabV1_1.h"

#pragma pack(push, 8)
namespace rock::api::grab::v1_2 {
    inline constexpr InterfaceId kInterfaceId = grab::kInterfaceId;
    inline constexpr std::uint32_t kMajor = 1, kMinor = 2;

    enum class DefaultPoseFlag : std::uint32_t { Present = 1, FingerValues = 2, FingerJoints = 4 };
    struct DefaultHandPose {
        std::uint32_t flags{};
        float translateGame[3]{};
        float rotate[9]{1,0,0, 0,1,0, 0,0,1};
        float fingerValues[5]{};
        float fingerJointValues[15]{};
    };
    struct DefaultGrabPoses {
        std::uint32_t size{sizeof(DefaultGrabPoses)}, baseFormId{};
        DefaultHandPose right{}, left{};
    };

    // Negotiate Grab 1.2 and sizeof(Api), then bind Write. Both calls are
    // ROCK frame-thread-only (Core frame callbacks permitted), and copy inputs.
    // Object pose is relative to the physical hand's GrabAuthorityProxy:
    // game-unit translation, row-major proper orthonormal rotation, unit scale.
    // Finger order is thumb/index/middle/ring/pinky; joints are proximal to
    // distal per finger. Thumb values accept [0,2], other fingers [0,1].
    // Present is required for any hand flags; FingerJoints requires FingerValues.
    // At least one hand must be present. Unknown flags/nonfinite/bad rotations
    // return InvalidArgument; incorrect request size returns InvalidSize.
    //
    // One owner per base form, 64 forms globally; replacing the same owner's
    // entire pair is atomic. Competing owners get OwnerConflict; no eviction
    // occurs on CapacityFull. Missing/non-bound base forms return TargetInvalid.
    // Defaults only fill missing per-hand saved offsets on ROCK's existing
    // saved-offset path (loose non-weapons and throwables). Authored stimpak
    // poses and ordinary firearm/melee attachment keep their existing priority.
    // Set/clear affect subsequent grabs, never an already held object.
    // No files are written. Defaults survive world/skeleton resets for the
    // process's loaded form IDs, and retire on owner unregister/revocation.
    // Clear is idempotent when absent; another owner's entry is OwnerConflict.
    // Existing Grab 1.0/1.1 prefixes and Core 1.0 requirements are unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor, minorVersion = kMinor;
        v1_1::Api v1_1;
        Status(ROCK_CALL* setDefaultGrabPoses)(OwnerToken, const DefaultGrabPoses*) noexcept;
        Status(ROCK_CALL* clearDefaultGrabPoses)(OwnerToken, std::uint32_t baseFormId) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api>);
    static_assert(offsetof(Api, v1_1) == 0);
    static_assert(offsetof(Api, setDefaultGrabPoses) == sizeof(v1_1::Api));
    static_assert(sizeof(Api) == sizeof(v1_1::Api) + 2 * sizeof(void*));
    static_assert(sizeof(DefaultHandPose) == 132 && sizeof(DefaultGrabPoses) == 272);
}
#pragma pack(pop)
