#include "physics-interaction/grab/TouchGrabRuntime.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/TouchGrabMath.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Bethesda/bhkCharacterController.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpConstraintCinfo.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rock
{
    namespace
    {
        constexpr std::size_t kBallAndSocketConstraintSize = 0x70;
        constexpr std::size_t kLimitedHingeConstraintSize = 0x130;
        constexpr std::size_t kPrismaticConstraintSize = 0x120;
        constexpr std::uint32_t kInvalidConstraintId = 0x7FFF'FFFFu;

        constexpr std::ptrdiff_t kLimitedHingeLimitEnabledOffset = 0xFA;
        constexpr std::ptrdiff_t kLimitedHingeMinimumOffset = 0xFC;
        constexpr std::ptrdiff_t kLimitedHingeMaximumOffset = 0x100;
        constexpr std::ptrdiff_t kPrismaticLimitEnabledOffset = 0x10A;
        constexpr std::ptrdiff_t kPrismaticMinimumOffset = 0x10C;
        constexpr std::ptrdiff_t kPrismaticMaximumOffset = 0x110;

        using ConstraintDataCtor = void* (*)(void*);
        using BallAndSocketSetPivots = void (*)(
            void*,
            const RE::hkVector4f&,
            const RE::hkVector4f&);
        using ConstraintSetInWorldSpace = void (*)(
            void*,
            const RE::hkTransformf&,
            const RE::hkTransformf&,
            const RE::hkVector4f&,
            const RE::hkVector4f&);

        [[nodiscard]] RE::NiPoint3 toNiPoint(
            const provider::RockProviderPoint3& point)
        {
            return { point.x, point.y, point.z };
        }

        [[nodiscard]] touch_grab_math::Vector3 toMathPoint(
            const RE::NiPoint3& point)
        {
            return { point.x, point.y, point.z };
        }

        [[nodiscard]] RE::NiPoint3 toNiPoint(
            const touch_grab_math::Vector3& point)
        {
            return { point.x, point.y, point.z };
        }

        [[nodiscard]] bool finitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) &&
                   std::isfinite(point.y) &&
                   std::isfinite(point.z);
        }

        [[nodiscard]] provider::TouchGrabMotionClassV1 classifyMotion(
            const havok_runtime::BodySnapshot& snapshot)
        {
            if (!snapshot.valid || !snapshot.body) {
                return provider::TouchGrabMotionClassV1::Other;
            }
            if (snapshot.motionIndex == 0 ||
                (snapshot.body->flags & 0x1u) != 0) {
                return provider::TouchGrabMotionClassV1::Static;
            }
            switch (snapshot.body->motionPropertiesId & 0xFFu) {
            case 1:
                return provider::TouchGrabMotionClassV1::Dynamic;
            case 2:
                return provider::TouchGrabMotionClassV1::Keyframed;
            default:
                return provider::TouchGrabMotionClassV1::Other;
            }
        }

        [[nodiscard]] bool sameRuntimeContract(
            const provider::RockProviderTouchGrabTargetV1& left,
            const provider::RockProviderTouchGrabTargetV1& right)
        {
            const bool commonContractMatches =
                left.targetId == right.targetId &&
                left.targetGeneration == right.targetGeneration &&
                left.kind == right.kind &&
                left.flags == right.flags &&
                left.bodyId == right.bodyId &&
                left.referenceFormId == right.referenceFormId &&
                left.referenceNativeHandle ==
                    right.referenceNativeHandle &&
                left.allowedLayerMask == right.allowedLayerMask &&
                left.worldGeneration == right.worldGeneration &&
                left.skeletonGeneration ==
                    right.skeletonGeneration &&
                left.providerGeneration ==
                    right.providerGeneration;
            if (!commonContractMatches ||
                left.kind ==
                    provider::RockProviderTouchGrabKindV1::
                        FixedAnchor) {
                return commonContractMatches;
            }
            return left.pivotWorldGame.x == right.pivotWorldGame.x &&
                   left.pivotWorldGame.y == right.pivotWorldGame.y &&
                   left.pivotWorldGame.z == right.pivotWorldGame.z &&
                   left.axisWorldGame.x == right.axisWorldGame.x &&
                   left.axisWorldGame.y == right.axisWorldGame.y &&
                   left.axisWorldGame.z == right.axisWorldGame.z &&
                   left.minimumCoordinate == right.minimumCoordinate &&
                   left.maximumCoordinate == right.maximumCoordinate;
        }

        [[nodiscard]] RE::hkTransformf makeHavokTransform(
            const RE::NiTransform& transform)
        {
            RE::hkTransformf result{};
            result.rotation =
                niRotToHkTransformRotation(transform.rotate);
            const float scale = gameToHavokScale();
            result.translation = RE::NiPoint4{
                transform.translate.x * scale,
                transform.translate.y * scale,
                transform.translate.z * scale,
                0.0f
            };
            return result;
        }

        [[nodiscard]] RE::hkVector4f makeHavokPoint(
            const RE::NiPoint3& point)
        {
            const float scale = gameToHavokScale();
            return {
                point.x * scale,
                point.y * scale,
                point.z * scale,
                0.0f
            };
        }

        [[nodiscard]] RE::hkVector4f makeHavokDirection(
            const RE::NiPoint3& direction)
        {
            return {
                direction.x,
                direction.y,
                direction.z,
                0.0f
            };
        }

        [[nodiscard]] std::uint32_t createStockConstraint(
            RE::hknpWorld* world,
            void* constraintData,
            const std::uint32_t bodyIdA,
            const std::uint32_t bodyIdB)
        {
            if (!world || !constraintData ||
                bodyIdA == kInvalidConstraintId ||
                bodyIdB == kInvalidConstraintId) {
                havok_ref_count::release(constraintData);
                return kInvalidConstraintId;
            }

            RE::hknpConstraintCinfo cinfo{};
            cinfo.constraintData =
                static_cast<RE::hkpConstraintData*>(constraintData);
            cinfo.bodyIdA = bodyIdA;
            cinfo.bodyIdB = bodyIdB;
            std::uint32_t constraintId = kInvalidConstraintId;
            world->CreateConstraint(&constraintId, cinfo);
            /*
             * FO4VR's hknp world takes its own hkpConstraintData reference.
             * The runtime retains only the constraint ID; DestroyConstraints
             * owns the matching final release.
             */
            havok_ref_count::release(constraintData);
            return constraintId;
        }

        [[nodiscard]] std::uint32_t createBallAndSocketConstraint(
            RE::hknpWorld* world,
            const std::uint32_t bodyIdA,
            const std::uint32_t bodyIdB,
            const RE::NiTransform& bodyAWorld,
            const RE::NiTransform& bodyBWorld,
            const RE::NiPoint3& pivotWorld)
        {
            auto* data =
                havok_runtime::allocateHavok(
                    kBallAndSocketConstraintSize);
            if (!data) {
                return kInvalidConstraintId;
            }
            static REL::Relocation<ConstraintDataCtor> constructor{
                REL::Offset(
                    offsets::kFunc_BallAndSocketConstraintData_Ctor)
            };
            static REL::Relocation<BallAndSocketSetPivots> setPivots{
                REL::Offset(
                    offsets::kFunc_BallAndSocketConstraintData_SetPivots)
            };
            constructor(data);

            const auto pivotA = makeHavokPoint(
                transform_math::worldPointToLocal(
                    bodyAWorld,
                    pivotWorld));
            const auto pivotB = makeHavokPoint(
                transform_math::worldPointToLocal(
                    bodyBWorld,
                    pivotWorld));
            setPivots(data, pivotA, pivotB);
            return createStockConstraint(
                world,
                data,
                bodyIdA,
                bodyIdB);
        }

        [[nodiscard]] std::uint32_t createLimitedHingeConstraint(
            RE::hknpWorld* world,
            const std::uint32_t bodyIdA,
            const std::uint32_t bodyIdB,
            const RE::NiTransform& bodyAWorld,
            const RE::NiTransform& bodyBWorld,
            const RE::NiPoint3& pivotWorld,
            const RE::NiPoint3& axisWorld,
            const float minimumRelativeAngle,
            const float maximumRelativeAngle)
        {
            auto* data =
                havok_runtime::allocateHavok(
                    kLimitedHingeConstraintSize);
            if (!data) {
                return kInvalidConstraintId;
            }
            static REL::Relocation<ConstraintDataCtor> constructor{
                REL::Offset(
                    offsets::kFunc_LimitedHingeConstraintData_Ctor)
            };
            static REL::Relocation<ConstraintSetInWorldSpace>
                setInWorldSpace{
                    REL::Offset(
                        offsets::
                            kFunc_LimitedHingeConstraintData_SetInWorldSpace)
                };
            constructor(data);

            const auto transformA = makeHavokTransform(bodyAWorld);
            const auto transformB = makeHavokTransform(bodyBWorld);
            const auto pivot = makeHavokPoint(pivotWorld);
            const auto axis = makeHavokDirection(axisWorld);
            setInWorldSpace(
                data,
                transformA,
                transformB,
                pivot,
                axis);

            auto* bytes = static_cast<std::uint8_t*>(data);
            bytes[kLimitedHingeLimitEnabledOffset] = 1;
            *reinterpret_cast<float*>(
                bytes + kLimitedHingeMinimumOffset) =
                minimumRelativeAngle;
            *reinterpret_cast<float*>(
                bytes + kLimitedHingeMaximumOffset) =
                maximumRelativeAngle;
            return createStockConstraint(
                world,
                data,
                bodyIdA,
                bodyIdB);
        }

        [[nodiscard]] std::uint32_t createPrismaticConstraint(
            RE::hknpWorld* world,
            const std::uint32_t bodyIdA,
            const std::uint32_t bodyIdB,
            const RE::NiTransform& bodyAWorld,
            const RE::NiTransform& bodyBWorld,
            const RE::NiPoint3& pivotWorld,
            const RE::NiPoint3& axisWorld,
            const float minimumRelativeGame,
            const float maximumRelativeGame)
        {
            auto* data =
                havok_runtime::allocateHavok(
                    kPrismaticConstraintSize);
            if (!data) {
                return kInvalidConstraintId;
            }
            static REL::Relocation<ConstraintDataCtor> constructor{
                REL::Offset(
                    offsets::kFunc_PrismaticConstraintData_Ctor)
            };
            static REL::Relocation<ConstraintSetInWorldSpace>
                setInWorldSpace{
                    REL::Offset(
                        offsets::
                            kFunc_PrismaticConstraintData_SetInWorldSpace)
                };
            constructor(data);

            const auto transformA = makeHavokTransform(bodyAWorld);
            const auto transformB = makeHavokTransform(bodyBWorld);
            const auto pivot = makeHavokPoint(pivotWorld);
            const auto axis = makeHavokDirection(axisWorld);
            setInWorldSpace(
                data,
                transformA,
                transformB,
                pivot,
                axis);

            const float scale = gameToHavokScale();
            auto* bytes = static_cast<std::uint8_t*>(data);
            bytes[kPrismaticLimitEnabledOffset] = 1;
            *reinterpret_cast<float*>(
                bytes + kPrismaticMinimumOffset) =
                minimumRelativeGame * scale;
            *reinterpret_cast<float*>(
                bytes + kPrismaticMaximumOffset) =
                maximumRelativeGame * scale;
            return createStockConstraint(
                world,
                data,
                bodyIdA,
                bodyIdB);
        }

        void destroyConstraint(
            RE::hknpWorld* world,
            std::uint32_t& constraintId)
        {
            if (constraintId == kInvalidConstraintId) {
                return;
            }
            if (world) {
                world->DestroyConstraints(&constraintId, 1);
            }
            constraintId = kInvalidConstraintId;
        }
    }

    void TouchGrabRuntime::setPhysicsCallbackGate(
        PhysicsCallbackQuiescenceGate* gate) noexcept
    {
        _physicsCallbackGate = gate;
    }

    bool TouchGrabRuntime::isHandActive(const bool isLeft) const noexcept
    {
        return findTargetForHand(isLeft) != nullptr;
    }

    TouchGrabRuntime::ActiveTarget* TouchGrabRuntime::findTarget(
        const std::uint64_t ownerToken,
        const std::uint64_t scopeToken,
        const std::uint64_t targetId,
        const std::uint32_t targetGeneration) noexcept
    {
        for (auto& active : _targets) {
            if (active.active &&
                active.ownerToken == ownerToken &&
                active.scopeToken == scopeToken &&
                active.target.targetId == targetId &&
                active.target.targetGeneration == targetGeneration) {
                return &active;
            }
        }
        return nullptr;
    }

    const TouchGrabRuntime::ActiveTarget*
    TouchGrabRuntime::findTargetForHand(const bool isLeft) const noexcept
    {
        for (const auto& active : _targets) {
            if (!active.active) {
                continue;
            }
            for (const auto& hand : active.hands) {
                if (hand.active && hand.isLeft == isLeft) {
                    return &active;
                }
            }
        }
        return nullptr;
    }

    TouchGrabRuntime::ActiveTarget*
    TouchGrabRuntime::findTargetForHand(const bool isLeft) noexcept
    {
        return const_cast<ActiveTarget*>(
            std::as_const(*this).findTargetForHand(isLeft));
    }

    TouchGrabRuntime::ActiveTarget*
    TouchGrabRuntime::firstFreeTarget() noexcept
    {
        for (auto& active : _targets) {
            if (!active.active) {
                return &active;
            }
        }
        return nullptr;
    }

    void TouchGrabRuntime::resetTarget(
        ActiveTarget& active) noexcept
    {
        active.active = false;
        active.ownerToken = 0;
        active.scopeToken = 0;
        active.target = {};
        active.bodyId = kInvalidId;
        active.originalMotionClass =
            provider::TouchGrabMotionClassV1::Other;
        active.collisionIdentity = nullptr;
        active.collisionObject = nullptr;
        active.mechanismConstraintId = kInvalidId;
        active.initialBodyWorld = {};
        active.axisWorld = {};
        active.initialHingeWitnessWorld = {};
        active.hingeWitnessBodyLocal = {};
        active.initialCoordinate = 0.0f;
        active.lastCoordinate = 0.0f;
        active.hands = {};
    }

    bool TouchGrabRuntime::tryAcquire(
        const bool isLeft,
        const hand_semantic_contact_state::SemanticContactRecord& contact,
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        const std::uint32_t collisionGeneration,
        const TargetClass targetClass)
    {
        if (!bhkWorld || !hknpWorld || !contact.valid ||
            contact.isLeft != isLeft ||
            contact.handBodyId == kInvalidId ||
            contact.otherBodyId == kInvalidId ||
            isHandActive(isLeft)) {
            return false;
        }

        const RE::hknpBodyId bodyId{ contact.otherBodyId };
        const auto snapshot =
            havok_runtime::snapshotBody(hknpWorld, bodyId);
        const auto motionClass = classifyMotion(snapshot);
        if (!snapshot.valid ||
            motionClass == provider::TouchGrabMotionClassV1::Other) {
            return false;
        }

        ActiveTarget* targetOnBody = nullptr;
        for (auto& candidate : _targets) {
            if (candidate.active &&
                candidate.bodyId == bodyId.value) {
                targetOnBody = &candidate;
                break;
            }
        }
        const auto eligibilityMotionClass =
            targetOnBody ?
            targetOnBody->originalMotionClass :
            motionClass;

        provider::TouchGrabTargetMatchV1 match{};
        const auto providerHand =
            isLeft ?
            provider::RockProviderHand::Left :
            provider::RockProviderHand::Right;
        const std::uint32_t layer =
            snapshot.collisionFilterInfo & 0x7Fu;
        if (!provider::resolveTouchGrabTargetV1(
                bodyId.value,
                layer,
                eligibilityMotionClass,
                providerHand,
                worldGeneration,
                skeletonGeneration,
                providerGeneration,
                match)) {
            return false;
        }
        if (match.yieldRequested) {
            return false;
        }
        const bool wildcardRequested =
            targetClass == TargetClass::Wildcard;
        if (match.wildcard != wildcardRequested) {
            return false;
        }

        const bool anyTargetActive = std::any_of(
            _targets.begin(),
            _targets.end(),
            [](const ActiveTarget& value) {
                return value.active;
            });
        if (anyTargetActive &&
            (bhkWorld != _activeBhkWorld ||
                hknpWorld != _activeHknpWorld ||
                worldGeneration != _worldGeneration ||
                skeletonGeneration != _skeletonGeneration ||
                providerGeneration != _providerGeneration)) {
            return false;
        }

        auto* active = findTarget(
            match.ownerToken,
            match.scopeToken,
            match.target.targetId,
            match.target.targetGeneration);
        if (targetOnBody && active != targetOnBody) {
            return false;
        }
        if (active) {
            if (active->bodyId != bodyId.value ||
                !provider::hasTouchGrabTargetFlagV1(
                    active->target.flags,
                    provider::RockProviderTouchGrabTargetFlagV1::
                        AllowTwoHands) ||
                !provider::hasTouchGrabTargetFlagV1(
                    match.target.flags,
                    provider::RockProviderTouchGrabTargetFlagV1::
                        AllowTwoHands)) {
                return false;
            }
            auto structuralMutation = _physicsCallbackGate ?
                _physicsCallbackGate->pauseForMutation() :
                PhysicsCallbackQuiescenceGate::MutationLease{};
            if (!attachHand(
                    *active,
                    isLeft,
                    contact,
                    hknpWorld)) {
                return false;
            }
            publishState(
                *active,
                provider::RockProviderTouchGrabPhaseV1::Held,
                provider::RockProviderTouchGrabReleaseReasonV1::None,
                0.0f,
                collisionGeneration);
            return true;
        }

        active = firstFreeTarget();
        if (!active) {
            return false;
        }
        resetTarget(*active);
        active->active = true;
        active->ownerToken = match.ownerToken;
        active->scopeToken = match.scopeToken;
        active->target = match.target;
        active->bodyId = bodyId.value;
        active->originalMotionClass = motionClass;
        active->collisionIdentity =
            snapshot.collisionObject;
        active->collisionObject =
            snapshot.collisionObject ?
            snapshot.collisionObject->IsbhkNPCollisionObject() :
            nullptr;
        active->initialCoordinate =
            match.target.kind ==
                provider::RockProviderTouchGrabKindV1::
                    FixedAnchor ?
            0.0f :
            match.target.currentCoordinate;
        active->lastCoordinate =
            active->initialCoordinate;
        active->axisWorld = toNiPoint(match.target.axisWorldGame);

        _activeBhkWorld = bhkWorld;
        _activeHknpWorld = hknpWorld;
        _worldGeneration = worldGeneration;
        _skeletonGeneration = skeletonGeneration;
        _providerGeneration = providerGeneration;

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (match.target.kind !=
                provider::RockProviderTouchGrabKindV1::FixedAnchor &&
            !createMechanism(*active, bhkWorld, hknpWorld)) {
            releaseTarget(
                *active,
                bhkWorld,
                hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    TargetInvalid,
                collisionGeneration,
                true);
            return false;
        }
        if (!attachHand(
                *active,
                isLeft,
                contact,
                hknpWorld)) {
            releaseTarget(
                *active,
                bhkWorld,
                hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    TargetInvalid,
                collisionGeneration,
                true);
            return false;
        }

        publishState(
            *active,
            provider::RockProviderTouchGrabPhaseV1::Held,
            provider::RockProviderTouchGrabReleaseReasonV1::None,
            0.0f,
            collisionGeneration);
        ROCK_LOG_INFO(
            Hand,
            "Touch grab acquired: hand={} owner={:016X} scope={:016X} target={} generation={} kind={} body={} motion={}",
            isLeft ? "left" : "right",
            active->ownerToken,
            active->scopeToken,
            active->target.targetId,
            active->target.targetGeneration,
            static_cast<std::uint32_t>(active->target.kind),
            active->bodyId,
            static_cast<std::uint32_t>(active->originalMotionClass));
        return true;
    }

    bool TouchGrabRuntime::createMechanism(
        ActiveTarget& active,
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world)
    {
        if (!bhkWorld || !world || !active.collisionObject ||
            active.originalMotionClass ==
                provider::TouchGrabMotionClassV1::Static ||
            active.bodyId == kInvalidId) {
            return false;
        }
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                RE::hknpBodyId{ active.bodyId },
                active.initialBodyWorld)) {
            return false;
        }

        touch_grab_math::Vector3 witness{};
        if (active.target.kind ==
                provider::RockProviderTouchGrabKindV1::LimitedHinge) {
            if (!touch_grab_math::makePerpendicularWitness(
                    toMathPoint(active.axisWorld),
                    witness)) {
                return false;
            }
            active.initialHingeWitnessWorld = toNiPoint(witness);
            active.hingeWitnessBodyLocal =
                transform_math::worldVectorToLocal(
                    active.initialBodyWorld,
                    active.initialHingeWitnessWorld);
        }

        auto* shape = grab_authority_proxy::buildProxyShape();
        if (!shape) {
            return false;
        }
        const auto material =
            havok_material_registry::registerGeneratedBodyMaterial(world);
        const bool anchorCreated =
            material.value != 0xFFFF &&
            active.anchor.create(
                world,
                bhkWorld,
                shape,
                grab_authority_proxy::noContactFilterInfo(),
                material,
                BethesdaMotionType::Keyframed,
                "ROCK_TouchGrabAnchor");
        havok_ref_count::release(shape);
        if (!anchorCreated) {
            return false;
        }

        RE::NiTransform anchorWorld =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        anchorWorld.translate = toNiPoint(
            active.target.pivotWorldGame);
        if (!active.anchor.setTransform(
                makeHavokTransform(anchorWorld))) {
            return false;
        }

        if (active.originalMotionClass ==
            provider::TouchGrabMotionClassV1::Keyframed) {
            active.collisionObject->SetMotionType(
                RE::hknpMotionPropertiesId::Preset::DYNAMIC);
        }
        const auto dynamicSnapshot = havok_runtime::snapshotBody(
            world,
            RE::hknpBodyId{ active.bodyId });
        if (!dynamicSnapshot.valid ||
            dynamicSnapshot.collisionObject !=
                active.collisionObject ||
            classifyMotion(dynamicSnapshot) !=
                provider::TouchGrabMotionClassV1::Dynamic) {
            return false;
        }

        RE::NiTransform liveBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                RE::hknpBodyId{ active.bodyId },
                liveBodyWorld)) {
            return false;
        }

        const float minimumRelative =
            active.target.minimumCoordinate -
            active.initialCoordinate;
        const float maximumRelative =
            active.target.maximumCoordinate -
            active.initialCoordinate;
        if (active.target.kind ==
            provider::RockProviderTouchGrabKindV1::LimitedHinge) {
            active.mechanismConstraintId =
                createLimitedHingeConstraint(
                    world,
                    active.anchor.getBodyId().value,
                    active.bodyId,
                    anchorWorld,
                    liveBodyWorld,
                    toNiPoint(active.target.pivotWorldGame),
                    active.axisWorld,
                    minimumRelative,
                    maximumRelative);
        } else if (active.target.kind ==
                   provider::RockProviderTouchGrabKindV1::
                       LimitedPrismatic) {
            active.mechanismConstraintId =
                createPrismaticConstraint(
                    world,
                    active.anchor.getBodyId().value,
                    active.bodyId,
                    anchorWorld,
                    liveBodyWorld,
                    toNiPoint(active.target.pivotWorldGame),
                    active.axisWorld,
                    minimumRelative,
                    maximumRelative);
        }
        if (active.mechanismConstraintId ==
            kInvalidConstraintId) {
            return false;
        }
        havok_runtime::activateBody(world, active.bodyId);
        return true;
    }

    bool TouchGrabRuntime::attachHand(
        ActiveTarget& active,
        const bool isLeft,
        const hand_semantic_contact_state::SemanticContactRecord& contact,
        RE::hknpWorld* world)
    {
        HandAttachment* attachment = nullptr;
        std::size_t activeHands = 0;
        for (auto& hand : active.hands) {
            if (hand.active) {
                ++activeHands;
                if (hand.isLeft == isLeft) {
                    return false;
                }
            } else if (!attachment) {
                attachment = &hand;
            }
        }
        if (!attachment ||
            (activeHands != 0 &&
                !provider::hasTouchGrabTargetFlagV1(
                    active.target.flags,
                    provider::RockProviderTouchGrabTargetFlagV1::
                        AllowTwoHands))) {
            return false;
        }

        RE::NiPoint3 contactPoint{};
        const bool hasContactPoint =
            contact.hasContactPointGame &&
            finitePoint(RE::NiPoint3{
                contact.contactPointGame.x,
                contact.contactPointGame.y,
                contact.contactPointGame.z
            });
        if (hasContactPoint) {
            contactPoint = {
                contact.contactPointGame.x,
                contact.contactPointGame.y,
                contact.contactPointGame.z
            };
        }

        std::uint32_t constraintId = kInvalidConstraintId;
        if (active.target.kind !=
            provider::RockProviderTouchGrabKindV1::FixedAnchor) {
            RE::NiTransform handBodyWorld{};
            RE::NiTransform targetBodyWorld{};
            if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    RE::hknpBodyId{ contact.handBodyId },
                    handBodyWorld) ||
                !havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    RE::hknpBodyId{ active.bodyId },
                    targetBodyWorld)) {
                return false;
            }
            if (!hasContactPoint) {
                contactPoint = handBodyWorld.translate;
            }
            constraintId = createBallAndSocketConstraint(
                world,
                contact.handBodyId,
                active.bodyId,
                handBodyWorld,
                targetBodyWorld,
                contactPoint);
            if (constraintId == kInvalidConstraintId) {
                return false;
            }
        }

        attachment->active = true;
        attachment->isLeft = isLeft;
        attachment->handBodyId = contact.handBodyId;
        attachment->constraintId = constraintId;
        attachment->hasContactPoint = hasContactPoint;
        attachment->contactPointGame = contactPoint;
        attachment->hasContactNormal =
            contact.hasContactNormalGame &&
            finitePoint(RE::NiPoint3{
                contact.contactNormalGame.x,
                contact.contactNormalGame.y,
                contact.contactNormalGame.z
            });
        if (attachment->hasContactNormal) {
            attachment->contactNormalGame = {
                contact.contactNormalGame.x,
                contact.contactNormalGame.y,
                contact.contactNormalGame.z
            };
        }
        return true;
    }

    void TouchGrabRuntime::service(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const float deltaSeconds,
        const std::uint32_t worldGeneration,
        const std::uint32_t skeletonGeneration,
        const std::uint32_t providerGeneration,
        const std::uint32_t collisionGeneration)
    {
        const bool anyActive = std::any_of(
            _targets.begin(),
            _targets.end(),
            [](const ActiveTarget& active) {
                return active.active;
            });
        if (!anyActive) {
            return;
        }

        if (!bhkWorld || !hknpWorld ||
            bhkWorld != _activeBhkWorld ||
            hknpWorld != _activeHknpWorld) {
            abandonAll(
                provider::RockProviderTouchGrabReleaseReasonV1::
                    WorldLost);
            return;
        }
        if (worldGeneration != _worldGeneration ||
            skeletonGeneration != _skeletonGeneration ||
            providerGeneration != _providerGeneration) {
            releaseAll(
                bhkWorld,
                hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    GenerationChanged,
                collisionGeneration);
            return;
        }

        for (auto& active : _targets) {
            if (!active.active) {
                continue;
            }
            provider::TouchGrabTargetMatchV1 current{};
            if (!provider::currentTouchGrabTargetV1(
                    active.ownerToken,
                    active.scopeToken,
                    active.target.targetId,
                    active.target.targetGeneration,
                    worldGeneration,
                    skeletonGeneration,
                    providerGeneration,
                    current)) {
                auto structuralMutation = _physicsCallbackGate ?
                    _physicsCallbackGate->pauseForMutation() :
                    PhysicsCallbackQuiescenceGate::MutationLease{};
                releaseTarget(
                    active,
                    bhkWorld,
                    hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        TargetRemoved,
                    collisionGeneration,
                    true);
                continue;
            }
            if (current.yieldRequested) {
                auto structuralMutation = _physicsCallbackGate ?
                    _physicsCallbackGate->pauseForMutation() :
                    PhysicsCallbackQuiescenceGate::MutationLease{};
                releaseTarget(
                    active,
                    bhkWorld,
                    hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        OwnerYield,
                    collisionGeneration,
                    true);
                continue;
            }
            if (!sameRuntimeContract(
                    active.target,
                    current.target)) {
                auto structuralMutation = _physicsCallbackGate ?
                    _physicsCallbackGate->pauseForMutation() :
                    PhysicsCallbackQuiescenceGate::MutationLease{};
                releaseTarget(
                    active,
                    bhkWorld,
                    hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        TargetInvalid,
                    collisionGeneration,
                    true);
                continue;
            }

            const auto bodySnapshot = havok_runtime::snapshotBody(
                hknpWorld,
                RE::hknpBodyId{ active.bodyId });
            if (!bodySnapshot.valid ||
                bodySnapshot.collisionObject !=
                    active.collisionIdentity ||
                (active.target.kind !=
                        provider::RockProviderTouchGrabKindV1::
                            FixedAnchor &&
                    (bodySnapshot.collisionObject !=
                            active.collisionObject ||
                        classifyMotion(bodySnapshot) !=
                            provider::TouchGrabMotionClassV1::
                                Dynamic))) {
                auto structuralMutation = _physicsCallbackGate ?
                    _physicsCallbackGate->pauseForMutation() :
                    PhysicsCallbackQuiescenceGate::MutationLease{};
                releaseTarget(
                    active,
                    bhkWorld,
                    hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        TargetInvalid,
                    collisionGeneration,
                    bodySnapshot.valid);
                continue;
            }

            const float coordinate =
                active.target.kind ==
                    provider::RockProviderTouchGrabKindV1::
                        FixedAnchor ?
                0.0f :
                std::clamp(
                    sampleCoordinate(active, hknpWorld),
                    active.target.minimumCoordinate,
                    active.target.maximumCoordinate);
            const float coordinateVelocity =
                std::isfinite(deltaSeconds) &&
                    deltaSeconds > 1.0e-5f ?
                (coordinate - active.lastCoordinate) /
                    deltaSeconds :
                0.0f;
            active.lastCoordinate = coordinate;
            if (active.target.kind !=
                provider::RockProviderTouchGrabKindV1::
                    FixedAnchor) {
                havok_runtime::activateBody(
                    hknpWorld,
                    active.bodyId);
            }
            publishState(
                active,
                provider::RockProviderTouchGrabPhaseV1::Held,
                provider::RockProviderTouchGrabReleaseReasonV1::None,
                coordinateVelocity,
                collisionGeneration);
        }
    }

    float TouchGrabRuntime::sampleCoordinate(
        ActiveTarget& active,
        RE::hknpWorld* world) const
    {
        if (active.target.kind ==
            provider::RockProviderTouchGrabKindV1::FixedAnchor) {
            return 0.0f;
        }
        RE::NiTransform currentBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                RE::hknpBodyId{ active.bodyId },
                currentBodyWorld)) {
            return active.lastCoordinate;
        }
        if (active.target.kind ==
            provider::RockProviderTouchGrabKindV1::LimitedPrismatic) {
            return touch_grab_math::prismaticCoordinate(
                active.initialCoordinate,
                toMathPoint(active.initialBodyWorld.translate),
                toMathPoint(currentBodyWorld.translate),
                toMathPoint(active.axisWorld));
        }
        const auto currentWitness =
            transform_math::localVectorToWorld(
                currentBodyWorld,
                active.hingeWitnessBodyLocal);
        return touch_grab_math::hingeCoordinate(
            active.initialCoordinate,
            toMathPoint(active.initialHingeWitnessWorld),
            toMathPoint(currentWitness),
            toMathPoint(active.axisWorld));
    }

    std::uint32_t TouchGrabRuntime::activeHandMask(
        const ActiveTarget& active) const noexcept
    {
        std::uint32_t mask = 0;
        for (const auto& hand : active.hands) {
            if (!hand.active) {
                continue;
            }
            mask |= static_cast<std::uint32_t>(
                hand.isLeft ?
                provider::RockProviderTouchGrabHandMaskV1::Left :
                provider::RockProviderTouchGrabHandMaskV1::Right);
        }
        return mask;
    }

    void TouchGrabRuntime::publishState(
        ActiveTarget& active,
        const provider::RockProviderTouchGrabPhaseV1 phase,
        const provider::RockProviderTouchGrabReleaseReasonV1 reason,
        const float coordinateVelocity,
        const std::uint32_t collisionGeneration)
    {
        provider::RockProviderTouchGrabStateV1 state{};
        state.targetId = active.target.targetId;
        state.targetGeneration =
            active.target.targetGeneration;
        state.kind = active.target.kind;
        state.phase = phase;
        state.releaseReason = reason;
        state.bodyId = active.bodyId;
        state.referenceFormId =
            active.target.referenceFormId;
        state.referenceNativeHandle =
            active.target.referenceNativeHandle;
        state.activeHandMask = activeHandMask(active);
        state.currentCoordinate =
            active.target.kind ==
                provider::RockProviderTouchGrabKindV1::
                    FixedAnchor ?
            0.0f :
            std::clamp(
                active.lastCoordinate,
                active.target.minimumCoordinate,
                active.target.maximumCoordinate);
        state.coordinateVelocity =
            std::isfinite(coordinateVelocity) ?
            coordinateVelocity :
            0.0f;
        state.worldGeneration = _worldGeneration;
        state.skeletonGeneration = _skeletonGeneration;
        state.providerGeneration = _providerGeneration;
        state.collisionGeneration = collisionGeneration;

        if (active.target.kind ==
            provider::RockProviderTouchGrabKindV1::FixedAnchor) {
            state.flags |= static_cast<std::uint32_t>(
                provider::RockProviderTouchGrabStateFlagV1::
                    FixedAnchor);
        } else {
            state.flags |= static_cast<std::uint32_t>(
                provider::RockProviderTouchGrabStateFlagV1::
                    CoordinateValid);
        }
        if (active.originalMotionClass ==
            provider::TouchGrabMotionClassV1::Keyframed) {
            state.flags |= static_cast<std::uint32_t>(
                provider::RockProviderTouchGrabStateFlagV1::
                    OriginalMotionKeyframed);
        } else if (active.originalMotionClass ==
                   provider::TouchGrabMotionClassV1::Dynamic) {
            state.flags |= static_cast<std::uint32_t>(
                provider::RockProviderTouchGrabStateFlagV1::
                    OriginalMotionDynamic);
        }
        for (const auto& hand : active.hands) {
            if (!hand.active) {
                continue;
            }
            if (hand.hasContactPoint) {
                state.contactPointGame = {
                    hand.contactPointGame.x,
                    hand.contactPointGame.y,
                    hand.contactPointGame.z
                };
                state.flags |= static_cast<std::uint32_t>(
                    provider::RockProviderTouchGrabStateFlagV1::
                        ContactPointValid);
            }
            if (hand.hasContactNormal) {
                state.contactNormalGame = {
                    hand.contactNormalGame.x,
                    hand.contactNormalGame.y,
                    hand.contactNormalGame.z
                };
                state.flags |= static_cast<std::uint32_t>(
                    provider::RockProviderTouchGrabStateFlagV1::
                        ContactNormalValid);
            }
            break;
        }
        provider::publishTouchGrabStateV1(
            active.ownerToken,
            active.scopeToken,
            state);
    }

    void TouchGrabRuntime::releaseHand(
        const bool isLeft,
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const provider::RockProviderTouchGrabReleaseReasonV1 reason,
        const std::uint32_t collisionGeneration)
    {
        auto* active = findTargetForHand(isLeft);
        if (!active) {
            return;
        }
        if (!bhkWorld || !hknpWorld ||
            bhkWorld != _activeBhkWorld ||
            hknpWorld != _activeHknpWorld) {
            abandonAll(
                provider::RockProviderTouchGrabReleaseReasonV1::
                    WorldLost);
            return;
        }
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        for (auto& hand : active->hands) {
            if (!hand.active || hand.isLeft != isLeft) {
                continue;
            }
            destroyConstraint(
                hknpWorld,
                hand.constraintId);
            hand = {};
            break;
        }

        if (activeHandMask(*active) != 0) {
            active->lastCoordinate =
                active->target.kind ==
                    provider::RockProviderTouchGrabKindV1::
                        FixedAnchor ?
                0.0f :
                std::clamp(
                    sampleCoordinate(*active, hknpWorld),
                    active->target.minimumCoordinate,
                    active->target.maximumCoordinate);
            publishState(
                *active,
                provider::RockProviderTouchGrabPhaseV1::Held,
                provider::RockProviderTouchGrabReleaseReasonV1::None,
                0.0f,
                collisionGeneration);
            return;
        }
        releaseTarget(
            *active,
            bhkWorld,
            hknpWorld,
            reason,
            collisionGeneration,
            true);
    }

    void TouchGrabRuntime::releaseAll(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const provider::RockProviderTouchGrabReleaseReasonV1 reason,
        const std::uint32_t collisionGeneration)
    {
        const bool anyActive = std::any_of(
            _targets.begin(),
            _targets.end(),
            [](const ActiveTarget& active) {
                return active.active;
            });
        if (!anyActive) {
            return;
        }
        if (!bhkWorld || !hknpWorld ||
            bhkWorld != _activeBhkWorld ||
            hknpWorld != _activeHknpWorld) {
            abandonAll(
                provider::RockProviderTouchGrabReleaseReasonV1::
                    WorldLost);
            return;
        }
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        for (auto& active : _targets) {
            if (active.active) {
                releaseTarget(
                    active,
                    bhkWorld,
                    hknpWorld,
                    reason,
                    collisionGeneration,
                    true);
            }
        }
    }

    void TouchGrabRuntime::releaseTarget(
        ActiveTarget& active,
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        const provider::RockProviderTouchGrabReleaseReasonV1 reason,
        const std::uint32_t collisionGeneration,
        const bool restoreTarget)
    {
        if (!active.active) {
            return;
        }
        if (world &&
            active.target.kind !=
                provider::RockProviderTouchGrabKindV1::
                    FixedAnchor) {
            active.lastCoordinate = std::clamp(
                sampleCoordinate(active, world),
                active.target.minimumCoordinate,
                active.target.maximumCoordinate);
        }
        for (auto& hand : active.hands) {
            destroyConstraint(world, hand.constraintId);
            hand = {};
        }
        destroyConstraint(
            world,
            active.mechanismConstraintId);

        if (restoreTarget && world &&
            active.target.kind !=
                provider::RockProviderTouchGrabKindV1::
                    FixedAnchor &&
            active.collisionObject &&
            active.bodyId != kInvalidId) {
            const auto snapshot = havok_runtime::snapshotBody(
                world,
                RE::hknpBodyId{ active.bodyId });
            if (snapshot.valid &&
                snapshot.collisionObject ==
                    active.collisionObject) {
                RE::hkVector4f zero{};
                active.collisionObject->SetLinearVelocity(zero);
                active.collisionObject->SetAngularVelocity(zero);
                if (active.originalMotionClass ==
                    provider::TouchGrabMotionClassV1::Keyframed) {
                    active.collisionObject->SetMotionType(
                        RE::hknpMotionPropertiesId::Preset::
                            KEYFRAMED);
                } else if (
                    active.originalMotionClass ==
                    provider::TouchGrabMotionClassV1::Dynamic) {
                    active.collisionObject->SetMotionType(
                        RE::hknpMotionPropertiesId::Preset::
                            DYNAMIC);
                }
            }
        }

        if (active.anchor.isValid()) {
            if (bhkWorld && world == _activeHknpWorld) {
                active.anchor.retireDeferred(bhkWorld);
            } else {
                active.anchor.destroy(nullptr);
            }
        }

        provider::RockProviderTouchGrabPhaseV1 phase =
            provider::RockProviderTouchGrabPhaseV1::Invalidated;
        if (reason ==
            provider::RockProviderTouchGrabReleaseReasonV1::
                OwnerYield) {
            phase =
                provider::RockProviderTouchGrabPhaseV1::Yielded;
        } else if (
            reason ==
            provider::RockProviderTouchGrabReleaseReasonV1::
                GripReleased) {
            const bool latch =
                active.target.kind !=
                    provider::RockProviderTouchGrabKindV1::
                        FixedAnchor &&
                provider::hasTouchGrabTargetFlagV1(
                    active.target.flags,
                    provider::RockProviderTouchGrabTargetFlagV1::
                        LatchOnRelease);
            phase = latch ?
                provider::RockProviderTouchGrabPhaseV1::Latched :
                provider::RockProviderTouchGrabPhaseV1::Armed;
        }
        publishState(
            active,
            phase,
            reason,
            0.0f,
            collisionGeneration);
        if (reason ==
            provider::RockProviderTouchGrabReleaseReasonV1::
                OwnerYield) {
            provider::acknowledgeTouchGrabYieldV1(
                active.ownerToken,
                active.scopeToken,
                active.target.targetId,
                active.target.targetGeneration);
        }
        ROCK_LOG_INFO(
            Hand,
            "Touch grab released: owner={:016X} scope={:016X} target={} generation={} body={} reason={} coordinate={:.4f}",
            active.ownerToken,
            active.scopeToken,
            active.target.targetId,
            active.target.targetGeneration,
            active.bodyId,
            static_cast<std::uint32_t>(reason),
            active.lastCoordinate);
        resetTarget(active);

        if (std::none_of(
                _targets.begin(),
                _targets.end(),
                [](const ActiveTarget& value) {
                    return value.active;
                })) {
            _activeBhkWorld = nullptr;
            _activeHknpWorld = nullptr;
            _worldGeneration = 0;
            _skeletonGeneration = 0;
            _providerGeneration = 0;
        }
    }

    void TouchGrabRuntime::abandonAll(
        const provider::RockProviderTouchGrabReleaseReasonV1 reason) noexcept
    {
        for (auto& active : _targets) {
            if (!active.active) {
                continue;
            }
            for (auto& hand : active.hands) {
                hand = {};
            }
            active.mechanismConstraintId =
                kInvalidConstraintId;
            if (active.anchor.isValid()) {
                active.anchor.destroy(nullptr);
            }
            publishState(
                active,
                provider::RockProviderTouchGrabPhaseV1::
                    Invalidated,
                reason,
                0.0f,
                0);
            resetTarget(active);
        }
        _activeBhkWorld = nullptr;
        _activeHknpWorld = nullptr;
        _worldGeneration = 0;
        _skeletonGeneration = 0;
        _providerGeneration = 0;
    }
}
