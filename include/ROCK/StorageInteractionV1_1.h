#pragma once
#include "StorageInteraction.h"

#pragma pack(push,8)
namespace rock::api::storage_interaction::v1_1
{
    inline constexpr char kExport[]="ROCK_StorageInteraction_QueryV1_1";
    inline constexpr std::uint32_t kMajor=1,kMinor=1;
    enum class GripRole : std::uint32_t { Nearest, Firing, Support };
    struct GripRequest {
        std::uint32_t size{sizeof(GripRequest)},version{1};
        SampleV1 sample{};
        std::uint64_t displayId{};
        Hand hand{Hand::None};
        GripRole role{GripRole::Nearest};
    };
    struct GripPose {
        std::uint32_t size{sizeof(GripPose)},version{1};
        SampleV1 sample{};
        std::uint64_t displayId{},poseSequence{};
        std::uint32_t storedReference{},valid{};
        Hand hand{Hand::None};
        GripRole role{GripRole::Nearest};
        Transform displayWorld{},handInDisplay{};
        Transform fingerLocals[15]{};
        Point3 controllerInDisplay{},controllerInTracking{},gripInDisplay{};
        float distanceGame{},acquisitionRadiusGame{};
        std::uint32_t fingerMask{},reserved{};
    };
    // Additive, opt-in 1.1. Negotiate major=1, minimumMinor=1 and sizeof(Api)
    // at this export before using getAuthoredGrip. The complete 1.0 prefix,
    // its size/version fields and ROCK_StorageInteraction_QueryV1 stay intact.
    // This export also accepts the unchanged 1.0 prefix size; no old consumer
    // is required to negotiate or bind additional interfaces.
    //
    // getAuthoredGrip is a read on the owner-frame thread, using Grab Read and
    // the caller's live display/committed Storage claim. Request a current
    // sample and physical hand. Nearest prefers an in-range authored firing
    // or support seat, otherwise the nearest seat; a concrete role keeps that
    // same seat during a pull.
    // valid=0 means no complete authored pose exists yet. No guessed grip is
    // returned. Geometry/collider readiness is not a requirement for this read.
    // Pose and 15 finger transforms are copied values; controllerInDisplay
    // comes from the tracked wand, never the posed/rendered wrist. Its local
    // coordinates scale with displayWorld.scale. controllerInTracking is the
    // same wand in tracking-room coordinates, expressed in game units. Require
    // both controller travel and travel away from the mount: moving only the
    // mount or moving controller and mount together must not synthesize a pull.
    // Reproject handInDisplay through each fresh displayWorld for a latched
    // visual hand; preserve the latched pose until release, identity/sequence
    // change or lifecycle loss. The consumer owns pull thresholds and delivery.
    // Animation hand authority and Touch 1.1 can present/reserve the latch;
    // each must have its own negotiated permissions and clear/expiry path.
    struct Api {
        ApiV1 v1;
        Status(ROCK_CALL* getAuthoredGrip)(OwnerToken,const GripRequest*,GripPose*) noexcept;
    };
    using Query=const Api*(ROCK_CALL*)(std::uint32_t major,std::uint32_t minimumMinor,std::uint32_t bytes) noexcept;
    static_assert(offsetof(Api,v1)==0 && offsetof(Api,getAuthoredGrip)==sizeof(ApiV1));
    static_assert(sizeof(Api)==56 && sizeof(GripRequest)==56);
}
#pragma pack(pop)
