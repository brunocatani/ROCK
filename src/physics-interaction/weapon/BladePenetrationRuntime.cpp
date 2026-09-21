#include "physics-interaction/weapon/BladePenetrationRuntime.h"

#include "RockConfig.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/telemetry/DynamicColliderTrace.h"

#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpConstraintCinfo.h"

#include <chrono>
#include <limits>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidId = 0x7FFF'FFFFu;
        using namespace blade_penetration;

        std::uint64_t nowMilliseconds()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        }

        RE::hkTransformf nativeBodyTransform(const RE::NiTransform& value)
        {
            RE::hkTransformf result{};
            // BODY readback stores each physical axis in one padded Ni row;
            // those three blocks already have hkTransform's column layout.
            result.rotation = value.rotate;
            const float scale = physics_scale::gameToHavok();
            result.translation = { value.translate.x * scale, value.translate.y * scale,
                value.translate.z * scale, 0.0f };
            return result;
        }

        template <std::size_t N>
        bool entryMatches(std::uintptr_t address, const std::array<std::uint8_t, N>& bytes)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            std::array<std::uint8_t, N> actual{};
            return address >= text.address() && address + N <= text.address() + text.size() &&
                native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), N) &&
                actual == bytes;
        }

        void setSlideDepth(void* data, float maximumDepthGame)
        {
            // Constructor 1419B142D and validator 1419B1ABA identify the limits.
            // Keep both directions open until physical withdrawal releases the
            // pair. The controller guide supplies the shorter withdrawal stop.
            const float extent = (std::max)(maximumDepthGame, kWithdrawalClearanceGame + 1.0f) * physics_scale::gameToHavok();
            const float minimum = -extent;
            std::memcpy(static_cast<std::byte*>(data) + 0x10C, &minimum, sizeof(minimum));
            std::memcpy(static_cast<std::byte*>(data) + 0x110, &extent, sizeof(extent));
        }

        std::uint32_t createBladeSlide(RE::hknpWorld* world, std::uint32_t bladeBody,
            std::uint32_t anchorBody, const RE::NiTransform& bladeWorld,
            const RE::NiTransform& anchorWorld, const RE::NiPoint3& tip, const RE::NiPoint3& axis,
            float maximumDepthGame, void*& borrowedData)
        {
            borrowedData = nullptr;
            using Constructor = void* (*)(void*);
            using SetFrames = void (*)(void*, const RE::hkTransformf&, const RE::hkTransformf&,
                const RE::hkVector4f&, const RE::hkVector4f&);
            const auto constructor = REL::Offset(offsets::kFunc_PrismaticConstraintData_Ctor).address();
            const auto setFrames = REL::Offset(offsets::kFunc_PrismaticConstraintData_SetInWorldSpace).address();
            if (!entryMatches(constructor, std::array<std::uint8_t, 7>{ 0xC7, 0x41, 0x08, 0x01, 0x00, 0xFF, 0xFF }) ||
                !entryMatches(setFrames, std::array<std::uint8_t, 8>{ 0x4C, 0x8B, 0xDC, 0x49, 0x89, 0x5B, 0x08, 0x49 })) {
                ROCK_LOG_ERROR(Weapon, "BLADE rejected: native slider entry bytes differ");
                return kInvalidId;
            }
            // Native allocation 141843CD9 and constructor 1419B1350 agree.
            auto* data = havok_runtime::allocateHavok(0x120);
            if (!data) return kInvalidId;
            reinterpret_cast<Constructor>(constructor)(data);
            const float scale = physics_scale::gameToHavok();
            const RE::hkVector4f pivot{ tip.x * scale, tip.y * scale, tip.z * scale, 0.0f };
            const RE::hkVector4f direction{ axis.x, axis.y, axis.z, 0.0f };
            reinterpret_cast<SetFrames>(setFrames)(data, nativeBodyTransform(bladeWorld),
                nativeBodyTransform(anchorWorld), pivot, direction);
            // 141A56EDB and 141A5B0FE read atom+2 as an AXIS INDEX.
            // Preserve constructor axis 0 at +10A; it is not an enable flag.
            setSlideDepth(data, maximumDepthGame);
            RE::hknpConstraintCinfo cinfo{};
            cinfo.constraintData = static_cast<RE::hkpConstraintData*>(data);
            cinfo.bodyIdA = bladeBody;
            cinfo.bodyIdB = anchorBody;
            std::uint32_t id = kInvalidId;
            world->CreateConstraint(&id, cinfo);
            if (id != kInvalidId) borrowedData = data; // The world now owns its reference.
            havok_ref_count::release(data);
            return id;
        }
    }

    bool BladePenetrationRuntime::resolveBlade(const WeaponCollision& collision, RE::NiNode* weaponNode,
        const std::uint64_t generation, blade_penetration::Blade& blade)
    {
        blade = {};
        if (_profileGeneration != generation) {
            _profileGeneration = generation;
            _sourceBodyId = kInvalidId;
            _sourceTip = {};
            _acquisitionUnavailable = false;
            const auto composition = collision.getWeaponCompositionSnapshot();
            const auto weaponFamily = family(composition.weaponFormId);
            if (composition.weaponGenerationKey != generation || weaponFamily == Family::None) return false;
            const auto evidence = collision.getProfileEvidenceDescriptors();
            if (evidence.size() > 128) return false;
            for (const auto& source : evidence) {
                if (!source.valid || source.weaponGenerationKey != generation) continue;
                const auto* profile = sourceProfile(weaponFamily, source.sourceName);
                if (!profile) continue;
                WeaponCollision::SupportGripEvidenceView view{};
                if (!collision.tryGetSupportGripEvidenceView(source.bodyId, weaponNode, view) || !view.sourceNodeCurrent ||
                    view.weaponGenerationKey != generation || view.localTriangles.empty() ||
                    view.localTriangles.size() > 10000) continue;
                constexpr float largest = (std::numeric_limits<float>::max)();
                RE::NiPoint3 minimum{ largest, largest, largest }, maximum{ -largest, -largest, -largest };
                RE::NiPoint3 tip{};
                bool finite = true;
                for (const auto& triangle : view.localTriangles) {
                    for (const auto& point : { triangle.v0, triangle.v1, triangle.v2 }) {
                        if (!dynamic_weapon_collision_policy::isFinitePoint(point)) { finite = false; break; }
                        if (point.y > maximum.y) tip = point;
                        minimum.x = (std::min)(minimum.x, point.x); minimum.y = (std::min)(minimum.y, point.y); minimum.z = (std::min)(minimum.z, point.z);
                        maximum.x = (std::max)(maximum.x, point.x); maximum.y = (std::max)(maximum.y, point.y); maximum.z = (std::max)(maximum.z, point.z);
                    }
                }
                if (!finite || !matchesSourceBounds(*profile, minimum, maximum)) continue;
                if (_sourceBodyId != kInvalidId) { _sourceBodyId = kInvalidId; break; }
                _sourceBodyId = source.bodyId;
                _sourceTip = tip;
            }
            ROCK_LOG_INFO(Weapon, "BLADE profile: form={:08X} generation={:016X} supported={} bladeBody={} tipSource=({:.4f},{:.4f},{:.4f}) axisSource=(0,1,0) handClearance={:.3f}m",
                composition.weaponFormId, generation, _sourceBodyId != kInvalidId, _sourceBodyId,
                _sourceTip.x, _sourceTip.y, _sourceTip.z, kHandClearanceMeters);
        }
        if (_sourceBodyId == kInvalidId) return false;
        WeaponCollision::SupportGripEvidenceView view{};
        if (!collision.tryGetSupportGripEvidenceView(_sourceBodyId, weaponNode, view) || !view.sourceNodeCurrent ||
            view.weaponGenerationKey != generation || !weaponNode ||
            !dynamic_weapon_collision_policy::isFiniteTransform(weaponNode->world)) return false;
        const auto sourceInWeapon = transform_math::composeTransforms(
            transform_math::invertTransform(weaponNode->world), view.localToWorld);
        blade.tipLocal = transform_math::localPointToWorld(sourceInWeapon, _sourceTip);
        blade.valid = dynamic_weapon_collision_policy::isFinitePoint(blade.tipLocal) &&
            blade_penetration::normalized(transform_math::localVectorToWorld(sourceInWeapon, RE::NiPoint3{ 0, 1, 0 }), blade.axisLocal);
        return blade.valid;
    }

    void BladePenetrationRuntime::recordContact(RE::hknpWorld* world, const std::uint32_t weaponBody,
        const std::uint32_t targetBody, const std::uint32_t targetLayer, const RE::NiPoint3& point)
    {
        if (!_blade.valid || world != _world || weaponBody != _weaponBodyId ||
            targetLayer != collision_layer_policy::FO4_LAYER_BIPED_NO_CC ||
            !dynamic_weapon_collision_policy::isFinitePoint(point)) return;
        const auto target = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ targetBody });
        RE::NiTransform bodyWorld{}, targetWorld{};
        if (!target.valid || !target.body || !target.body->shape || !target.collisionObject ||
            !havok_runtime::tryGetBodyWorldTransform(world, RE::hknpBodyId{ weaponBody }, bodyWorld) ||
            !havok_runtime::tryGetBodyWorldTransform(world, target.bodyId, targetWorld)) return;
        weapon_surface_support::Contact contact{};
        contact.world = reinterpret_cast<std::uintptr_t>(world);
        contact.shape = reinterpret_cast<std::uintptr_t>(target.body->shape);
        contact.collisionObject = reinterpret_cast<std::uintptr_t>(target.collisionObject);
        contact.generation = _profileGeneration;
        contact.sampledAtMilliseconds = nowMilliseconds();
        contact.proxyBodyId = weaponBody;
        contact.surfaceBodyId = targetBody;
        contact.weaponWorld = weaponFromBody(bodyWorld, _centerWeaponLocal, _weaponScale);
        contact.surfaceWorld = targetWorld;
        contact.weaponPointLocal = transform_math::worldPointToLocal(contact.weaponWorld, point);
        contact.surfacePointLocal = transform_math::worldPointToLocal(targetWorld, point);
        contact.valid = dynamic_weapon_collision_policy::isFiniteTransform(contact.weaponWorld) &&
            dynamic_weapon_collision_policy::isFiniteTransform(targetWorld);
        if (contact.valid) _contacts.publish(contact);
    }

    bool BladePenetrationRuntime::targetWorld(RE::hknpWorld* world, RE::NiTransform& result) const
    {
        if (!_contact.valid || world != _world) return false;
        const auto target = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ _contact.surfaceBodyId });
        return target.valid && target.body &&
            reinterpret_cast<std::uintptr_t>(target.collisionObject) == _contact.collisionObject &&
            reinterpret_cast<std::uintptr_t>(target.body->shape) == _contact.shape &&
            (target.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK) == collision_layer_policy::FO4_LAYER_BIPED_NO_CC &&
            havok_runtime::tryGetBodyWorldTransform(world, target.bodyId, result) &&
            dynamic_weapon_collision_policy::isFiniteTransform(result);
    }

    bool BladePenetrationRuntime::update(const PhysicsFrameContext& frame, const WeaponCollision& collision,
        RE::NiNode* weaponNode, BethesdaPhysicsBody& weaponBody, const RE::NiPoint3& centerWeaponLocal,
        const std::uint64_t generation, PhysicsCallbackQuiescenceGate* gate,
        RE::NiTransform& requestedWeapon, const bool surfaceSupportActive, const RE::NiPoint3* primaryGripWeaponLocal)
    {
        if (!gate) return !active();
        if (_profileGeneration == generation && _sourceBodyId == kInvalidId && !active()) return true;
        auto mutation = gate->pauseForMutation();
        blade_penetration::Blade blade{};
        const bool supported = resolveBlade(collision, weaponNode, generation, blade);
        if (!supported && !_blade.valid && !active()) return true;
        const bool wasActive = active();
        if (wasActive && (frame.hknpWorld != _world || !supported || surfaceSupportActive ||
                !_slideData || _physicsFailed.load(std::memory_order_acquire) ||
                !native_player_collision::hasBladePair(_world, _weaponBodyId, _contact.surfaceBodyId) ||
                blade_penetration::dot(blade.axisLocal, _blade.axisLocal) < 0.999f ||
                blade_penetration::dot(difference(blade.tipLocal, _blade.tipLocal), difference(blade.tipLocal, _blade.tipLocal)) > 0.01f)) {
            ROCK_LOG_WARN(Weapon, "BLADE retirement requested: source-or-target-invalid");
            return false; // Parent retires the overlapping weapon before restoring ordinary tracking.
        }
        _blade = blade;
        _world = frame.hknpWorld;
        _weaponBodyId = weaponBody.getBodyId().value;
        _weaponScale = requestedWeapon.scale;
        _centerWeaponLocal = centerWeaponLocal;
        if (!_blade.valid || surfaceSupportActive || _acquisitionUnavailable) return true;

        const float maximumDepthGame = primaryGripWeaponLocal ?
            maximumDepth(_blade, *primaryGripWeaponLocal, _weaponScale, physics_scale::havokToGame()) : 0.0f;
        if (maximumDepthGame <= 0.0f) {
            const auto now = nowMilliseconds();
            if (dynamic_collider_trace::enabled() && now - _lastContactReport >= 1000) {
                _lastContactReport = now;
                dynamic_collider_trace::writeWeapon("BLADE unavailable: reason=grip-clearance active={} gripPresent={}",
                    wasActive, primaryGripWeaponLocal != nullptr);
            }
            return !wasActive;
        }
        if (wasActive && maximumDepthGame != _maximumDepthGame) {
            setSlideDepth(_slideData, maximumDepthGame);
            havok_runtime::activateBody(_world, _weaponBodyId);
        }
        _maximumDepthGame = maximumDepthGame;

        RE::NiTransform physicalBody{}, target{};
        if (!havok_runtime::tryGetBodyWorldTransform(_world, weaponBody.getBodyId(), physicalBody)) return !wasActive;
        const auto physicalWeapon = weaponFromBody(physicalBody, centerWeaponLocal, _weaponScale);
        if (!wasActive) {
            weapon_surface_support::Contact candidate{};
            if (!_contacts.read(candidate) || !weapon_surface_support::isFresh(candidate,
                    reinterpret_cast<std::uintptr_t>(_world), generation, _weaponBodyId, nowMilliseconds())) return true;
            _contact = candidate;
            if (!targetWorld(_world, target)) return true;
            auto* ref = reference_interaction::resolveBody(_world, _contact.surfaceBodyId);
            if (!ref || ref->GetFormType() != RE::ENUM_FORM_ID::kACHR || ref == RE::PlayerCharacter::GetSingleton()) {
                const auto now = nowMilliseconds();
                if (dynamic_collider_trace::enabled() && now - _lastContactReport >= 1000) {
                    _lastContactReport = now;
                    dynamic_collider_trace::writeWeapon("BLADE rejected: target={} reason=npc-reference-unavailable", _contact.surfaceBodyId);
                }
                return true;
            }
            const auto point = transform_math::localPointToWorld(target, candidate.surfacePointLocal);
            const auto entry = evaluateEntry(_blade, physicalWeapon, requestedWeapon, point);
            const auto now = nowMilliseconds();
            if (dynamic_collider_trace::enabled() && now - _lastContactReport >= 1000) {
                _lastContactReport = now;
                dynamic_collider_trace::writeWeapon("BLADE contact: target={} tipDistance={:.3f} pressure={:.3f} sideways={:.3f} accepted={}",
                    _contact.surfaceBodyId, entry.tipDistance, entry.pressure, entry.lateralError, entry.accepted);
            }
            if (!entry.accepted) return true;
            // Fail once per generation if native setup cannot be established.
            // Ordinary contact remains solid and a re-equip permits retry.
            _acquisitionUnavailable = true;
            if (!_anchor.isValid()) {
                auto* shape = grab_authority_proxy::buildProxyShape();
                if (!shape) {
                    ROCK_LOG_WARN(Weapon, "BLADE rejected: guide-shape-unavailable");
                    return true;
                }
                const auto material = havok_material_registry::registerGeneratedBodyMaterial(_world);
                const bool created = material.value != 0xFFFF && _anchor.create(_world, frame.bhkWorld, shape,
                    grab_authority_proxy::noContactFilterInfo(), material, BethesdaMotionType::Keyframed, "ROCK_BladeGuide");
                havok_ref_count::release(shape);
                if (!created) {
                    ROCK_LOG_WARN(Weapon, "BLADE rejected: guide-body-unavailable");
                    return true;
                }
            }
            if (!_anchor.setTransform(nativeBodyTransform(target))) {
                ROCK_LOG_WARN(Weapon, "BLADE rejected: guide-placement-failed");
                return true;
            }
            RE::NiTransform placedAnchor{};
            if (!havok_runtime::tryGetBodyWorldTransform(_world, _anchor.getBodyId(), placedAnchor) ||
                dynamic_weapon_collision_policy::translationDeltaGameUnits(target, placedAnchor) > 0.05f ||
                dynamic_weapon_collision_policy::rotationDeltaDegrees(target, placedAnchor) > 0.5f) {
                ROCK_LOG_WARN(Weapon, "BLADE rejected: guide placement readback mismatch");
                return true;
            }
            RE::NiPoint3 axis{};
            if (!axisWorld(_blade, physicalWeapon, axis)) return true;
            const auto tip = transform_math::localPointToWorld(physicalWeapon, _blade.tipLocal);
            _constraintId = createBladeSlide(_world, _weaponBodyId, _anchor.getBodyId().value,
                physicalBody, target, tip, axis, _maximumDepthGame, _slideData);
            if (!active()) {
                ROCK_LOG_WARN(Weapon, "BLADE rejected: slider-creation-failed");
                return true;
            }
            if (!native_player_collision::publishBladePair(_world, _weaponBodyId, _contact.surfaceBodyId)) {
                release(_world, "simulation-pair-unavailable");
                return true;
            }
            _pairPublished = true;
            _entryWeaponInTarget = transform_math::composeTransforms(transform_math::invertTransform(target), physicalWeapon);
            _lastTargetWorld = target;
            _physicsFailed.store(false, std::memory_order_release);
            _acquisitionUnavailable = false;
            havok_runtime::activateBody(_world, _weaponBodyId);
            ROCK_LOG_INFO(Weapon, "BLADE entered: body={} target={} constraint={} tip=({:.3f},{:.3f},{:.3f}) axis=({:.3f},{:.3f},{:.3f}) maximumDepth={:.2f}gu",
                _weaponBodyId, _contact.surfaceBodyId, _constraintId, tip.x, tip.y, tip.z, axis.x, axis.y, axis.z, _maximumDepthGame);
        } else if (!targetWorld(_world, target)) {
            ROCK_LOG_WARN(Weapon, "BLADE retirement requested: target-lost");
            return false;
        }

        const auto command = guide(_blade, _entryWeaponInTarget, target, requestedWeapon, _maximumDepthGame);
        const auto actual = guide(_blade, _entryWeaponInTarget, target, physicalWeapon, _maximumDepthGame);
        if (!command.valid || !actual.valid) {
            ROCK_LOG_WARN(Weapon, "BLADE retirement requested: invalid-guide");
            return false;
        }
        if (withdrawn(command.requestedDepth, actual.requestedDepth)) {
            release(_world, "withdrawn");
            return true;
        }
        if (dynamic_weapon_collision_policy::translationDeltaGameUnits(requestedWeapon, physicalWeapon) >
                g_rockConfig.rockWeaponCollisionGripRecoveryDistanceGameUnits) {
            ROCK_LOG_WARN(Weapon, "BLADE retirement requested: tracking-discontinuity");
            return false;
        }
        requestedWeapon = command.weaponWorld;
        _presentation = physicalWeapon;
        const auto now = nowMilliseconds();
        if (dynamic_collider_trace::enabled() && now - _lastContactReport >= 500) {
            _lastContactReport = now;
            dynamic_collider_trace::writeWeapon("BLADE depth: target={} requested={:.3f} actual={:.3f} maximum={:.3f}gu filteredPairs={}",
                _contact.surfaceBodyId, command.requestedDepth, actual.requestedDepth, _maximumDepthGame,
                native_player_collision::bladePairRejectedCount());
        }
        return true;
    }

    bool BladePenetrationRuntime::prePhysics(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!active()) return true;
        RE::NiTransform target{};
        if (_physicsFailed.load(std::memory_order_acquire) || world != _world || !targetWorld(world, target) ||
            !timing.valid || timing.usedFallback || !havok_physics_timing::isUsableDelta(timing.substepDeltaSeconds) ||
            dynamic_weapon_collision_policy::translationDeltaGameUnits(_lastTargetWorld, target) >
                dynamic_weapon_collision_policy::kDivergenceTeleportDistanceGameUnits ||
            dynamic_weapon_collision_policy::rotationDeltaDegrees(_lastTargetWorld, target) > 90.0f ||
            !_anchor.driveToKeyFrame(nativeBodyTransform(target), timing.substepDeltaSeconds)) {
            _physicsFailed.store(true, std::memory_order_release);
            return false;
        }
        _lastTargetWorld = target;
        return true;
    }

    void BladePenetrationRuntime::release(RE::hknpWorld* world, const char* reason)
    {
        _slideData = nullptr;
        _maximumDepthGame = 0.0f;
        if (active()) {
            if (world && world == _world) world->DestroyConstraints(&_constraintId, 1);
            ROCK_LOG_INFO(Weapon, "BLADE released: target={} reason={}", _contact.surfaceBodyId, reason);
            _constraintId = kInvalidId;
        }
        if (_pairPublished) {
            native_player_collision::clearBladePair(world == _world ? world : nullptr);
            _pairPublished = false;
        }
        _contact = {};
        _contacts.clear();
        _physicsFailed.store(false, std::memory_order_release);
    }

    void BladePenetrationRuntime::retire(RE::hknpWorld* world, void* bhkWorld)
    {
        release(world, "weapon-retired");
        if (world && world == _world && bhkWorld && _anchor.isValid()) _anchor.retireDeferred(bhkWorld);
        else _anchor.reset();
        _blade = {};
        _world = nullptr;
        _profileGeneration = 0;
        _sourceBodyId = _weaponBodyId = kInvalidId;
        _acquisitionUnavailable = false;
    }
}
