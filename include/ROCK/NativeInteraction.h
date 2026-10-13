#pragma once
#include "Abi.h"

// Opt-in FO4VR-native interaction extension. Existing interface majors, table
// prefixes, records and permissions are unchanged. Native addresses below are
// borrowed only inside the provider callback that supplied them. They are not
// save identities and may never be dereferenced by a worker/render thread.
#pragma pack(push, 8)
namespace rock::api::native_interaction {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::NativeInteraction;
    inline constexpr std::uint32_t kMajor = 1, kMinor = 0;
    inline constexpr std::uint32_t kSupportedPermissions = 3;
    inline constexpr std::uint32_t kInvalidBody = 0x7FFFFFFF;
    inline constexpr std::uint32_t kMaxExtensions = 8, kMaxBodies = 1024;
    inline constexpr std::uint32_t kMaxVisualBodies = 1024;
    inline constexpr std::uint32_t kMaxWeaponSources = 128;
    inline constexpr std::uint32_t kPassiveObstacleLayer = 55;
    using Resource = std::uint64_t;

    // Value-only marshalling: real NiMatrix3 rows contain a fourth padding lane.
    // Never memcpy nine floats from/to that native matrix storage.
    template<class NativeTransform> Transform packTransform(const NativeTransform& native) {
        Transform value{};
        for(int row=0;row<3;++row)for(int column=0;column<3;++column)value.rotate[row*3+column]=native.rotate.entry[row][column];
        value.translate[0]=native.translate.x;value.translate[1]=native.translate.y;value.translate[2]=native.translate.z;
        value.scale=native.scale;return value;
    }
    template<class NativeTransform> NativeTransform unpackTransform(const Transform& value) {
        NativeTransform native{};
        for(int row=0;row<3;++row)for(int column=0;column<3;++column)native.rotate.entry[row][column]=value.rotate[row*3+column];
        if constexpr(std::extent_v<decltype(native.rotate.entry),1> > 3)for(int row=0;row<3;++row)native.rotate.entry[row][3]=0;
        native.translate={value.translate[0],value.translate[1],value.translate[2]};native.scale=value.scale;return native;
    }

    enum class Phase : std::uint32_t { Update = 1, PreCollide, Render, Retire, WeaponUpdate, WeaponContact, WeaponRetire, Selection, Extraction, Gameplay };
    struct Frame {
        std::uint32_t size{sizeof(Frame)};
        Phase phase{Phase::Update};
        SampleV1 sample{};
        std::uintptr_t world{}, bhkWorld{};
        Point3 hmdPosition{};
        float deltaSeconds{}, gameToHavok{}, havokToGame{};
        std::uint32_t worldReady{}, menuBlocked{}, hmdValid{};
        float substepSeconds{};
        std::uint32_t timingValid{}, timingFallback{};
    };
    struct BodyIdentity {
        std::uint32_t body{kInvalidBody}, motion{kInvalidBody};
        std::uintptr_t collisionObject{}, ownerNode{}, shape{};
        std::uint32_t group{}, reserved{};
        bool operator==(const BodyIdentity&) const = default;
    };
    enum class BodyReadStage : std::uint32_t { BodySlot, BodyIdentity, OwnerWorld, BodyPose, Complete };
    struct BodySnapshot {
        std::uint32_t size{sizeof(BodySnapshot)}, valid{};
        BodyIdentity identity{};
        Transform world{};
        std::uint32_t filter{}, flags{};
        // Native FO4VR body+6C membership witness; -1 is absent unless native
        // pending-registration flag 0x40000 is set. Raw flags are read-only.
        std::int32_t membership{-1};
        std::uint32_t material{0xFFFF};
        BodyReadStage stage{BodyReadStage::BodySlot};
        std::uint32_t motionProperties{};
    };
    enum class PoseBasis : std::uint32_t { NiRows = 0, BodyStoredAxes = 1 };
    struct BodyCreate {
        std::uint32_t size{sizeof(BodyCreate)}, filter{};
        // Nonzero privateAnchor builds ROCK's established non-contact guide;
        // otherwise source must identify the exact live shape to retain.
        BodyIdentity source{};
        std::uint32_t privateAnchor{};
        PoseBasis poseBasis{PoseBasis::NiRows};
        Transform pose{};
        char name[48]{};
    };
    struct SliderCreate {
        std::uint32_t size{sizeof(SliderCreate)}, bodyA{kInvalidBody};
        Resource anchor{};
        Transform bodyWorld{}, anchorWorld{};
        Point3 pivotWorld{}, axisWorld{};
        float minimumGame{}, maximumGame{};
    };
    struct WeaponSource {
        std::uint32_t size{sizeof(WeaponSource)}, body{kInvalidBody};
        std::uint64_t generation{};
        char name[64]{};
    };
    struct SourceGeometry {
        std::uint32_t size{sizeof(SourceGeometry)}, valid{};
        std::uint64_t generation{};
        std::uint32_t triangleCount{}, reserved{};
        Transform sourceInWeapon{};
        Bounds3 boundsLocal{};
        Point3 supportPointLocal{};
    };
    struct WeaponFrame {
        std::uint32_t size{sizeof(WeaponFrame)}, body{kInvalidBody};
        Frame frame{};
        std::uint64_t generation{};
        std::uint32_t form{}, supportOwnsPose{};
        Transform requestedWeapon{};
        Point3 centerWeaponLocal{}, primaryGripLocal{};
        std::uint32_t primaryGripValid{};
        float recoveryDistanceGame{};
    };
    enum class DriveResult : std::uint32_t { Unchanged = 0, Controlled = 1, RetireWeapon = 2 };
    struct WeaponCommand {
        std::uint32_t size{sizeof(WeaponCommand)};
        DriveResult result{DriveResult::Unchanged};
        Transform requestedWeapon{}, presentation{};
    };
    struct Contact {
        std::uint32_t size{sizeof(Contact)}, weapon{kInvalidBody};
        Frame frame{};
        std::uint32_t target{kInvalidBody}, targetLayer{};
        Point3 pointWorld{};
    };
    enum class SelectionKind : std::uint32_t { Unhandled = 0, Blocked, DirectConnectedBody, DirectDetachedBody, ExtractableItem };
    enum class NodeRelation : std::uint32_t { Descendant = 0, SkinnedTo = 1 };
    enum class SelectionStage : std::uint32_t { BeforeCoreTypes = 0, AfterCoreTypes = 1 };
    struct SelectionQuery {
        std::uint32_t size{sizeof(SelectionQuery)}, body{kInvalidBody};
        std::uintptr_t reference{}, world{}, hitNode{};
        Point3 hitPoint{};
        std::uint32_t hasHitPoint{}, farSelection{};
        SelectionStage stage{SelectionStage::BeforeCoreTypes};
    };
    struct SelectionReply {
        std::uint32_t size{sizeof(SelectionReply)};
        SelectionKind kind{SelectionKind::Unhandled};
        OwnerToken owner{};
        std::uintptr_t visualNode{};
        // Consumer-owned value ticket. Addresses may be opaque identity values,
        // never pointers to dereference on a later callback. Re-resolve on action.
        std::uint32_t ticket[8]{};
        char reason[64]{};
    };
    struct ExtractedReference {
        std::uint32_t size{sizeof(ExtractedReference)}, nativeHandle{};
        std::uint32_t referenceForm{}, itemForm{};
    };
    struct ProximityQuery {
        std::uint32_t size{sizeof(ProximityQuery)};
        float radiusGame{};
        std::uintptr_t reference{};
    };
    struct ProximityResult {
        std::uint32_t size{sizeof(ProximityResult)}, handled{}, targetForm{}, scanned{}, truncated{};
    };
    struct ImpactQuery {
        std::uint32_t size{sizeof(ImpactQuery)};
        float authoredDamage{};
        std::uintptr_t source{}, target{};
    };
    struct ImpactResult {
        std::uint32_t size{sizeof(ImpactResult)}, handled{}, targetForm{};
    };
    enum class PushKind : std::uint32_t { Unhandled = 0, PointOnly = 1, Blocked = 2 };
    // Every shape uses its live native BODY pose. InactiveSource prevents an
    // absent body's stale world AABB from being advertised as live geometry.
    enum class VisualFlag : std::uint32_t { None = 0, Dimensions = 1, InactiveSource = 2 };
    struct BodyVisual {
        std::uint32_t size{sizeof(BodyVisual)}, flags{};
        BodyIdentity identity{};
        float color[4]{1,1,1,1};
        float labelOffsetY{};
        char label[64]{};
    };
    using ExtractionCallback = Status(ROCK_CALL*)(const SelectionQuery*,const SelectionReply*,ExtractedReference*,void*) noexcept;
    struct Callbacks {
        std::uint32_t size{sizeof(Callbacks)}, version{1};
        void* context{};
        void(ROCK_CALL* update)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* preCollide)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* render)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* retire)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* weaponUpdate)(const WeaponFrame*,WeaponCommand*,void*) noexcept{};
        std::uint32_t(ROCK_CALL* weaponPreCollide)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* weaponContact)(const Contact*,void*) noexcept{};
        void(ROCK_CALL* weaponRetire)(const Frame*,void*) noexcept{};
        void(ROCK_CALL* classifySelection)(const SelectionQuery*,SelectionReply*,void*) noexcept{};
        ExtractionCallback extractSelection{};
        std::uint32_t(ROCK_CALL* validateSelection)(const SelectionQuery*,const SelectionReply*,void*) noexcept{};
        void(ROCK_CALL* proximityTarget)(const ProximityQuery*,ProximityResult*,void*) noexcept{};
        void(ROCK_CALL* impactEffect)(const ImpactQuery*,ImpactResult*,void*) noexcept{};
        // Retired: ROCK no longer pushes on contact and never calls this. The
        // slot remains only while the registered layout still includes it.
        PushKind(ROCK_CALL* classifyPush)(const SelectionQuery*,void*) noexcept{};
    };

    // Registration/unregistration: ROCK update-owner thread (before the first
    // frame, a Core callback can perform registration). Native operations below
    // are restricted to these callbacks; tokens cannot extend callback lifetime.
    // Structural operations are Update/WeaponUpdate/Retire only. Drive/filter
    // operations also permit PreCollide. Shape/reference results are borrowed.
    // All game mutations drain the same physics callbacks before proceeding.
    // Contact callbacks may be concurrent and must publish through a bounded
    // thread-safe channel. Unregister drains callbacks and retires all owned
    // constraints, bodies, pair suppressions, obstacle masks and visualizations.
    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        Status(ROCK_CALL* registerExtension)(OwnerToken,const Callbacks*) noexcept;
        Status(ROCK_CALL* unregisterExtension)(OwnerToken) noexcept;
        Status(ROCK_CALL* readBody)(OwnerToken,std::uint32_t,BodySnapshot*) noexcept;
        Status(ROCK_CALL* resolveCollisionBody)(OwnerToken,std::uintptr_t,std::uint32_t*) noexcept;
        Status(ROCK_CALL* resolveBodyReference)(OwnerToken,std::uint32_t,std::uintptr_t*) noexcept;
        Status(ROCK_CALL* queryNodeRelation)(OwnerToken,std::uintptr_t,std::uintptr_t,NodeRelation,std::uint32_t,std::uint32_t*) noexcept;
        Status(ROCK_CALL* createBody)(OwnerToken,const BodyCreate*,Resource*,std::uint32_t*) noexcept;
        Status(ROCK_CALL* retireBody)(OwnerToken,Resource) noexcept;
        Status(ROCK_CALL* placeBody)(OwnerToken,Resource,const Transform*) noexcept;
        Status(ROCK_CALL* driveBody)(OwnerToken,Resource,const Transform*,float) noexcept;
        Status(ROCK_CALL* setBodyFilter)(OwnerToken,Resource,std::uint32_t) noexcept;
        Status(ROCK_CALL* activateWeapon)(OwnerToken) noexcept;
        Status(ROCK_CALL* createSlider)(OwnerToken,const SliderCreate*,Resource*) noexcept;
        Status(ROCK_CALL* setSliderLimits)(OwnerToken,Resource,float,float) noexcept;
        Status(ROCK_CALL* destroySlider)(OwnerToken,Resource) noexcept;
        Status(ROCK_CALL* setWeaponContactExclusion)(OwnerToken,const BodyIdentity*) noexcept;
        Status(ROCK_CALL* clearWeaponContactExclusion)(OwnerToken) noexcept;
        Status(ROCK_CALL* hasWeaponContactExclusion)(OwnerToken,const BodyIdentity*,std::uint32_t*) noexcept;
        Status(ROCK_CALL* setDynamicObstacleLayers)(OwnerToken,std::uint64_t) noexcept;
        Status(ROCK_CALL* copyWeaponSources)(OwnerToken,WeaponSource*,std::uint32_t,std::uint32_t*) noexcept;
        Status(ROCK_CALL* getSourceGeometry)(OwnerToken,std::uint32_t,const Point3*,SourceGeometry*) noexcept;
        Status(ROCK_CALL* publishVisuals)(OwnerToken,const BodyVisual*,std::uint32_t) noexcept;
    };
    static_assert(std::is_standard_layout_v<ApiV1> && sizeof(ApiV1)==22*sizeof(void*));
    static_assert(std::is_trivially_copyable_v<BodySnapshot> && std::is_trivially_copyable_v<Callbacks>);
}
#pragma pack(pop)
