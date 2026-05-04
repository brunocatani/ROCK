#include "PhysicsInteraction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "BodyCollisionControl.h"
#include "CollisionLayerPolicy.h"
#include "CollisionSuppressionRegistry.h"
#include "ContactPipelinePolicy.h"
#include "ContactSignalSubscriptionPolicy.h"
#include "DirectSkeletonBoneReader.h"
#include "HavokOffsets.h"
#include "DebugBodyOverlay.h"
#include "DebugOverlayPolicy.h"
#include "RootFlattenedFingerSkeletonRuntime.h"
#include "GrabInteractionPolicy.h"
#include "GrabTransformTelemetry.h"
#include "GrabTransformTelemetryOverlay.h"
#include "HeldObjectPhysicsMath.h"
#include "HeldObjectBodySetPolicy.h"
#include "HeldObjectDampingMath.h"
#include "HeldPlayerSpaceMath.h"
#include "HeldPlayerSpaceRegistry.h"
#include "HandCollisionSuppressionMath.h"
#include "HavokRuntime.h"
#include "InputRemapPolicy.h"
#include "InputRemapRuntime.h"
#include "ObjectPhysicsBodySet.h"
#include "OffhandInteractionReservation.h"
#include "PalmTransform.h"
#include "PhysicsHooks.h"
#include "PhysicsRecursiveWrappers.h"
#include "PhysicsScale.h"
#include "PhysicsUtils.h"
#include "PhysicsWorldOriginDiagnostics.h"
#include "PushAssist.h"
#include "SelectionStatePolicy.h"
#include "WeaponTwoHandedGripMath.h"
#include "WeaponAuthorityLifecyclePolicy.h"
#include "WeaponMuzzleAuthorityMath.h"
#include "WeaponSupportAuthorityPolicy.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"
#include "f4vr/MiscStructs.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"
#include "vrcf/VRControllersManager.h"
#include <windows.h>

namespace frik::rock
{
    namespace
    {
        constexpr float kRawParityWarnPosition = 0.10f;
        constexpr float kRawParityWarnRotationDegrees = 0.5f;
        constexpr float kRawParityFailPosition = 0.50f;
        constexpr float kRawParityFailRotationDegrees = 2.0f;
        constexpr int kRawParityWarnFrames = 2;
        constexpr int kRawParityFailFrames = 10;
        constexpr int kRawParitySummaryFrames = 300;
        constexpr int kRawParityLagFrames = 5;
        constexpr float kRawParityLagSlack = 0.05f;
        DirectSkeletonBoneReader s_directSkeletonBoneReader;
        std::uint32_t s_directSkeletonBoneLogCounter = 0;
        bool s_worldOriginDiagnosticsEnabledLogged = false;

        struct ContactEventCallbackInfo
        {
            void* fn = nullptr;
            std::uint64_t ctx = 0;
        };

        struct GrabButtonState
        {
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
        };

        GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return {};
            }

            const auto rawState = input_remap_runtime::consumeRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return GrabButtonState{ .held = rawState.held, .pressed = rawState.pressed, .released = rawState.released };
            }

            const auto vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            return GrabButtonState{
                .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                .released = vrcf::VRControllers.isReleased(vrHand, buttonId),
            };
        }

        bool readGrabButtonHeld(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.held;
            }

            return vrcf::VRControllers.isPressHeldDown(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

        bool readGrabButtonPressedEdge(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.pressed;
            }

            return vrcf::VRControllers.isPressed(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

        struct TransformDelta
        {
            float position = 0.0f;
            float rotationDegrees = 0.0f;
        };

        TransformDelta measureTransformDelta(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const float dx = a.translate.x - b.translate.x;
            const float dy = a.translate.y - b.translate.y;
            const float dz = a.translate.z - b.translate.z;

            const auto qa = niRotToHkQuat(a.rotate);
            const auto qb = niRotToHkQuat(b.rotate);
            const float dot = std::clamp(std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w), 0.0f, 1.0f);
            const float angleRadians = 2.0f * std::acos(dot);

            return TransformDelta{ .position = std::sqrt(dx * dx + dy * dy + dz * dz), .rotationDegrees = angleRadians * (180.0f / std::numbers::pi_v<float>)};
        }

        float measurePointDelta(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        float measureDirectionDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
            return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
        }

        bool startsWith(std::string_view value, std::string_view prefix)
        {
            return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
        }

        debug::SkeletonOverlayRole skeletonOverlayRoleForBone(std::string_view name)
        {
            if (startsWith(name, "RArm_Finger")) {
                return debug::SkeletonOverlayRole::RightFinger;
            }
            if (startsWith(name, "LArm_Finger")) {
                return debug::SkeletonOverlayRole::LeftFinger;
            }
            if (startsWith(name, "RArm_")) {
                return debug::SkeletonOverlayRole::RightArm;
            }
            if (startsWith(name, "LArm_")) {
                return debug::SkeletonOverlayRole::LeftArm;
            }
            if (startsWith(name, "RLeg_")) {
                return debug::SkeletonOverlayRole::RightLeg;
            }
            if (startsWith(name, "LLeg_")) {
                return debug::SkeletonOverlayRole::LeftLeg;
            }
            if (name == "Head" || name == "Neck") {
                return debug::SkeletonOverlayRole::Head;
            }
            return debug::SkeletonOverlayRole::Core;
        }

        std::string_view trimView(std::string_view value)
        {
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.remove_prefix(1);
            }
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.remove_suffix(1);
            }
            return value;
        }

        bool skeletonLogFilterMatches(std::string_view filter, std::string_view boneName)
        {
            filter = trimView(filter);
            if (filter.empty()) {
                return false;
            }

            while (!filter.empty()) {
                const std::size_t comma = filter.find(',');
                const std::string_view token = trimView(filter.substr(0, comma));
                if (token == boneName) {
                    return true;
                }
                if (comma == std::string_view::npos) {
                    break;
                }
                filter.remove_prefix(comma + 1);
            }
            return false;
        }

        std::uint32_t currentEquippedWeaponFormId()
        {
            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            return weaponForm ? weaponForm->formID : 0;
        }

        void fillProviderTransform(const RE::NiTransform& source, ::rock::provider::RockProviderTransform& target)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    target.rotate[static_cast<std::size_t>(row * 3 + column)] = source.rotate.entry[row][column];
                }
            }
            target.translate[0] = source.translate.x;
            target.translate[1] = source.translate.y;
            target.translate[2] = source.translate.z;
            target.scale = source.scale;
        }

        std::uint32_t providerHandStateFlags(const Hand& hand, bool isLeft)
        {
            std::uint32_t flags = 0;
            if (hand.isTouching()) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::Touching);
            }
            if (hand.isHolding()) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::Holding);
            }
            if (isLeft ? PhysicsInteraction::s_leftHandDisabled.load(std::memory_order_acquire) :
                         PhysicsInteraction::s_rightHandDisabled.load(std::memory_order_acquire)) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::PhysicsDisabled);
            }
            return flags;
        }

        void copyProviderString(char* target, std::size_t targetSize, const std::string& source)
        {
            if (!target || targetSize == 0) {
                return;
            }

            std::snprintf(target, targetSize, "%s", source.c_str());
            target[targetSize - 1] = '\0';
        }

        ::rock::provider::RockProviderPoint3 makeProviderPoint(const WeaponEvidencePoint3& point)
        {
            return ::rock::provider::RockProviderPoint3{ .x = point.x, .y = point.y, .z = point.z };
        }

        const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
        }

        const char* weaponSupportClassName(weapon_support_authority_policy::WeaponSupportWeaponClass weaponClass)
        {
            using weapon_support_authority_policy::WeaponSupportWeaponClass;

            switch (weaponClass) {
            case WeaponSupportWeaponClass::Unknown:
                return "Unknown";
            case WeaponSupportWeaponClass::Sidearm:
                return "Sidearm";
            case WeaponSupportWeaponClass::LongGun:
                return "LongGun";
            }
            return "Unknown";
        }

        const char* weaponSupportAuthorityModeName(weapon_support_authority_policy::WeaponSupportAuthorityMode mode)
        {
            using weapon_support_authority_policy::WeaponSupportAuthorityMode;

            switch (mode) {
            case WeaponSupportAuthorityMode::FullTwoHandedSolver:
                return "FullTwoHandedSolver";
            case WeaponSupportAuthorityMode::VisualOnlySupport:
                return "VisualOnlySupport";
            }
            return "Unknown";
        }

        struct WeaponSupportKeywordCache
        {
            bool initialized{ false };
            RE::BGSKeyword* pistolGrip{ nullptr };
            RE::BGSKeyword* rifleAssaultGrip{ nullptr };
            RE::BGSKeyword* rifleStraightGrip{ nullptr };
            RE::BGSKeyword* shoulderFiredGrip{ nullptr };
        };

        RE::BGSKeyword* resolveWeaponSupportKeyword(const char* editorID)
        {
            return editorID ? RE::TESForm::GetFormByEditorID<RE::BGSKeyword>(RE::BSFixedString(editorID)) : nullptr;
        }

        const WeaponSupportKeywordCache& weaponSupportKeywordCache()
        {
            static WeaponSupportKeywordCache cache{};
            if (!cache.initialized) {
                cache.initialized = true;
                cache.pistolGrip = resolveWeaponSupportKeyword("AnimsGripPistol");
                cache.rifleAssaultGrip = resolveWeaponSupportKeyword("AnimsGripRifleAssault");
                cache.rifleStraightGrip = resolveWeaponSupportKeyword("AnimsGripRifleStraight");
                cache.shoulderFiredGrip = resolveWeaponSupportKeyword("AnimsGripShoulderFired");
                if (!cache.pistolGrip || !cache.rifleAssaultGrip || !cache.rifleStraightGrip || !cache.shoulderFiredGrip) {
                    ROCK_LOG_WARN(
                        Weapon,
                        "Weapon support grip keyword lookup incomplete: AnimsGripPistol={} AnimsGripRifleAssault={} AnimsGripRifleStraight={} AnimsGripShoulderFired={}",
                        static_cast<const void*>(cache.pistolGrip),
                        static_cast<const void*>(cache.rifleAssaultGrip),
                        static_cast<const void*>(cache.rifleStraightGrip),
                        static_cast<const void*>(cache.shoulderFiredGrip));
                }
            }
            return cache;
        }

        bool keywordFormHasKeyword(const RE::BGSKeywordForm* keywordForm, const RE::BGSKeyword* keyword)
        {
            return keywordForm && keyword && keywordForm->HasKeyword(keyword, nullptr);
        }

        bool equippedWeaponHasKeyword(const RE::TESObjectWEAP* weapon, const RE::TBO_InstanceData* instanceData, const RE::BGSKeyword* keyword)
        {
            return weapon && keyword && weapon->HasKeyword(keyword, instanceData);
        }

        bool equippedInstanceHasKeyword(const RE::TBO_InstanceData* instanceData, const RE::BGSKeyword* keyword)
        {
            const auto* keywordData = instanceData ? instanceData->GetKeywordData() : nullptr;
            return keywordFormHasKeyword(keywordData, keyword);
        }

        bool equippedWeaponHasAnyKeyword(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSKeyword* keywordA,
            const RE::BGSKeyword* keywordB,
            const RE::BGSKeyword* keywordC)
        {
            return equippedWeaponHasKeyword(weapon, instanceData, keywordA) ||
                   equippedWeaponHasKeyword(weapon, instanceData, keywordB) ||
                   equippedWeaponHasKeyword(weapon, instanceData, keywordC);
        }

        bool equippedInstanceHasAnyKeyword(
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSKeyword* keywordA,
            const RE::BGSKeyword* keywordB,
            const RE::BGSKeyword* keywordC)
        {
            return equippedInstanceHasKeyword(instanceData, keywordA) ||
                   equippedInstanceHasKeyword(instanceData, keywordB) ||
                   equippedInstanceHasKeyword(instanceData, keywordC);
        }

        void populateEquippedWeaponKeywordIdentity(
            weapon_support_authority_policy::EquippedWeaponIdentity& identity,
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData)
        {
            const auto& keywords = weaponSupportKeywordCache();
            identity.hasPistolGripKeyword = equippedWeaponHasKeyword(weapon, instanceData, keywords.pistolGrip);
            identity.hasInstancePistolGripKeyword = equippedInstanceHasKeyword(instanceData, keywords.pistolGrip);
            identity.hasLongGunGripKeyword = equippedWeaponHasAnyKeyword(
                weapon,
                instanceData,
                keywords.rifleAssaultGrip,
                keywords.rifleStraightGrip,
                keywords.shoulderFiredGrip);
            identity.hasInstanceLongGunGripKeyword = equippedInstanceHasAnyKeyword(
                instanceData,
                keywords.rifleAssaultGrip,
                keywords.rifleStraightGrip,
                keywords.shoulderFiredGrip);
        }

        const RE::TESObjectWEAP* asEquippedWeaponForm(const F4SEVR::TESForm* form)
        {
            if (!form || form->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return nullptr;
            }

            const auto* reForm = reinterpret_cast<const RE::TESForm*>(form);
            return reForm->As<RE::TESObjectWEAP>();
        }

        weapon_support_authority_policy::EquippedWeaponIdentity makeEquippedWeaponSupportIdentity(RE::NiNode* weaponNode)
        {
            weapon_support_authority_policy::EquippedWeaponIdentity identity{};
            identity.nodeName = weaponDiagnosticNodeName(weaponNode);

            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (weaponForm) {
                identity.formID = weaponForm->formID;
                if (const char* fullName = weaponForm->GetFullName()) {
                    identity.displayName = fullName;
                }
                populateEquippedWeaponKeywordIdentity(identity, asEquippedWeaponForm(weaponForm), equipData ? equipData->instanceData : nullptr);
            }

            return identity;
        }

        weapon_support_authority_policy::WeaponSupportAuthorityMode resolveEquippedWeaponSupportAuthorityMode(RE::NiNode* weaponNode)
        {
            using namespace weapon_support_authority_policy;

            if (!g_rockConfig.rockVisualOnlySidearmSupportGripEnabled) {
                return WeaponSupportAuthorityMode::FullTwoHandedSolver;
            }

            const auto identity = makeEquippedWeaponSupportIdentity(weaponNode);
            const auto weaponClass = classifyEquippedWeaponForSupportGrip(identity);
            const auto authorityMode = resolveSupportAuthorityMode(true, weaponClass);
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Weapon support classification: formID={:08X} name='{}' node='{}' pistolGrip={} instancePistolGrip={} longGunGrip={} instanceLongGunGrip={} class={} mode={}",
                identity.formID,
                identity.displayName,
                identity.nodeName,
                identity.hasPistolGripKeyword,
                identity.hasInstancePistolGripKeyword,
                identity.hasLongGunGripKeyword,
                identity.hasInstanceLongGunGripKeyword,
                weaponSupportClassName(weaponClass),
                weaponSupportAuthorityModeName(authorityMode));
            return authorityMode;
        }

        const char* pushAssistSkipReasonName(push_assist::PushAssistSkipReason reason)
        {
            switch (reason) {
            case push_assist::PushAssistSkipReason::None:
                return "none";
            case push_assist::PushAssistSkipReason::Disabled:
                return "disabled";
            case push_assist::PushAssistSkipReason::Cooldown:
                return "cooldown";
            case push_assist::PushAssistSkipReason::BelowMinSpeed:
                return "below-min-speed";
            case push_assist::PushAssistSkipReason::InvalidImpulse:
                return "invalid-impulse";
            }
            return "unknown";
        }

        WeaponInteractionDebugInfo makeWeaponInteractionDebugInfo(
            const WeaponCollision& weaponCollision,
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& contact)
        {
            WeaponInteractionDebugInfo info{};
            info.weaponNodeName = weaponDiagnosticNodeName(weaponNode);

            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (weaponForm) {
                info.weaponFormId = weaponForm->formID;
                if (const char* fullName = weaponForm->GetFullName()) {
                    info.weaponName = fullName;
                }
            }

            if (contact.valid) {
                WeaponInteractionDebugInfo sourceInfo{};
                if (weaponCollision.tryGetWeaponContactDebugInfo(contact.bodyId, sourceInfo)) {
                    info.sourceName = sourceInfo.sourceName;
                    info.sourceRootName = sourceInfo.sourceRootName;
                }
            }

            return info;
        }

        f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
        {
            /*
             * Ported from FRIK's WeaponPositionAdjuster muzzle fix because ROCK
             * is now the final weapon visual owner during mesh/hand authority.
             * FRIK still performs its own fix earlier in the frame, but any
             * later ROCK weapon write must re-own the fire node from the current
             * projectile node so the origin remains at the barrel tip.
             */
            const auto equipWeaponData = f4vr::getEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto vfunc = reinterpret_cast<std::uint64_t*>(equipWeaponData);
            if ((*vfunc & 0xFFFF) != (f4vr::EquippedWeaponData_vfunc.get() & 0xFFFF)) {
                return nullptr;
            }

            const auto muzzle = reinterpret_cast<f4vr::MuzzleFlash*>(equipWeaponData->unk28);
            if (!muzzle || !muzzle->fireNode || !muzzle->projectileNode) {
                return nullptr;
            }

            return muzzle;
        }

        void applyFinalWeaponMuzzleAuthority()
        {
            auto* muzzle = getEquippedMuzzleFlashNodes();
            if (!muzzle) {
                return;
            }

            muzzle->fireNode->local = weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(muzzle->projectileNode->world);
            f4vr::updateTransformsDown(muzzle->fireNode, true);
        }

        RE::NiNode* resolveEquippedWeaponInteractionNode()
        {
            /*
             * Weapon interaction needs the visual root that ROCK generated
             * collision from. Firearms normally expose the first-person "Weapon"
             * node; melee weapons in FO4VR are attached through the primary melee
             * offset nodes. Reusing the already-mapped F4VR node fields keeps this
             * as ROCK policy glue instead of adding a new native layout claim.
             */
            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->isWeaponDrawn()) {
                return nullptr;
            }

            if (api->isMeleeWeaponDrawn()) {
                if (auto* playerNodes = f4vr::getPlayerNodes()) {
                    if (playerNodes->primaryMeleeWeaponOffsetNode) {
                        return playerNodes->primaryMeleeWeaponOffsetNode;
                    }
                    if (playerNodes->PrimaryMeleeWeaponOffsetNode) {
                        return playerNodes->PrimaryMeleeWeaponOffsetNode;
                    }
                }
            }

            return f4vr::getWeaponNode();
        }
    }

    PhysicsInteraction::PhysicsInteraction()
    {
        s_instance.store(this, std::memory_order_release);
        _generatedBodyStepDrive.setDriveCallback(&PhysicsInteraction::onGeneratedBodyPhysicsStep, this);

        installBumpHook();
        installNativeGrabHook();
        /*
         * PAPER is the reload owner. ROCK keeps hand, weapon, and contact
         * provider hooks active, but it must not install the native clip-write
         * gate because that would create two authorities for one ammo mutation
         * path. PAPER will reinstall verified native reload hooks after the
         * required Ghidra audit records the FO4VR addresses.
         */
        installRefreshManifoldHook();

        ROCK_LOG_INFO(Init, "ROCK Physics Module v0.1 — created");
    }

    PhysicsInteraction::~PhysicsInteraction()
    {
        s_instance.store(nullptr, std::memory_order_release);

        if (_initialized) {
            shutdown();
        }
        ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
    }

    bool PhysicsInteraction::tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        outTransform = {};
        if (!_handBoneCache.isReady()) {
            return false;
        }

        outTransform = _handBoneCache.getWorldTransform(isLeft);
        return true;
    }

    void PhysicsInteraction::fillProviderFrameSnapshot(::rock::provider::RockProviderFrameSnapshot& outSnapshot) const
    {
        auto* api = frik::api::FRIKApi::inst;
        outSnapshot.providerReady = _initialized.load(std::memory_order_acquire) ? 1u : 0u;
        outSnapshot.frikSkeletonReady = api && api->isSkeletonReady() ? 1u : 0u;
        outSnapshot.menuBlocking = api && api->isAnyMenuOpen() ? 1u : 0u;
        outSnapshot.configBlocking = api && (api->isConfigOpen() || api->isWristPipboyOpen()) ? 1u : 0u;
        outSnapshot.bhkWorld = reinterpret_cast<std::uintptr_t>(_cachedBhkWorld);
        outSnapshot.hknpWorld = reinterpret_cast<std::uintptr_t>(_cachedBhkWorld ? getHknpWorld(_cachedBhkWorld) : nullptr);
        outSnapshot.gameToHavokScale = physics_scale::gameToHavok();
        outSnapshot.havokToGameScale = physics_scale::havokToGame();
        outSnapshot.physicsScaleRevision = physics_scale::revision();

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        outSnapshot.weaponNode = reinterpret_cast<std::uintptr_t>(weaponNode);
        outSnapshot.weaponFormId = currentEquippedWeaponFormId();
        outSnapshot.weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        outSnapshot.weaponBodyCount = (std::min)(_weaponCollision.getWeaponBodyCount(), ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_BODIES);
        for (std::uint32_t i = 0; i < outSnapshot.weaponBodyCount; ++i) {
            outSnapshot.weaponBodyIds[i] = _weaponCollision.getWeaponBodyIdAtomic(i);
        }

        if (_handBoneCache.isReady()) {
            fillProviderTransform(_handBoneCache.getWorldTransform(false), outSnapshot.rightHandTransform);
            fillProviderTransform(_handBoneCache.getWorldTransform(true), outSnapshot.leftHandTransform);
        }

        outSnapshot.rightHandBodyId = _rightHand.getCollisionBodyId().value;
        outSnapshot.leftHandBodyId = _leftHand.getCollisionBodyId().value;
        outSnapshot.rightHandState = providerHandStateFlags(_rightHand, false);
        outSnapshot.leftHandState = providerHandStateFlags(_leftHand, true);
        outSnapshot.offhandReservation = ::rock::provider::currentOffhandReservation();
    }

    bool PhysicsInteraction::queryProviderWeaponContactAtPoint(
        const ::rock::provider::RockProviderWeaponContactQuery& query,
        ::rock::provider::RockProviderWeaponContactResult& outResult) const
    {
        outResult = {};
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        if (!weaponNode) {
            return false;
        }

        WeaponInteractionContact contact{};
        const RE::NiPoint3 point{ query.pointGame[0], query.pointGame[1], query.pointGame[2] };
        if (!_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, point, query.radiusGame, contact)) {
            return false;
        }

        outResult.valid = contact.valid ? 1u : 0u;
        outResult.bodyId = contact.bodyId;
        outResult.partKind = static_cast<std::uint32_t>(contact.partKind);
        outResult.reloadRole = static_cast<std::uint32_t>(contact.reloadRole);
        outResult.supportRole = static_cast<std::uint32_t>(contact.supportGripRole);
        outResult.socketRole = static_cast<std::uint32_t>(contact.socketRole);
        outResult.actionRole = static_cast<std::uint32_t>(contact.actionRole);
        outResult.interactionRoot = reinterpret_cast<std::uintptr_t>(contact.interactionRoot);
        outResult.sourceRoot = reinterpret_cast<std::uintptr_t>(contact.sourceRoot);
        outResult.weaponGenerationKey = contact.weaponGenerationKey;
        outResult.probeDistanceGame = contact.probeDistanceGame;
        return contact.valid;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDescriptors(
        ::rock::provider::RockProviderWeaponEvidenceDescriptor* outDescriptors,
        std::uint32_t maxDescriptors) const
    {
        if (!outDescriptors || maxDescriptors == 0) {
            return 0;
        }

        const auto descriptors = _weaponCollision.getProfileEvidenceDescriptors();
        const std::uint32_t count = (std::min)(maxDescriptors, static_cast<std::uint32_t>(descriptors.size()));
        const std::uint64_t generationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outDescriptors[i];
            out = {};
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(descriptor.semantic.partKind);
            out.reloadRole = static_cast<std::uint32_t>(descriptor.semantic.reloadRole);
            out.supportRole = static_cast<std::uint32_t>(descriptor.semantic.supportGripRole);
            out.socketRole = static_cast<std::uint32_t>(descriptor.semantic.socketRole);
            out.actionRole = static_cast<std::uint32_t>(descriptor.semantic.actionRole);
            out.fallbackGripPose = static_cast<std::uint32_t>(descriptor.semantic.fallbackGripPose);
            out.interactionRoot = descriptor.geometryRootAddress;
            out.sourceRoot = descriptor.sourceRootAddress;
            out.weaponGenerationKey = generationKey;
            copyProviderString(out.sourceName, sizeof(out.sourceName), descriptor.sourceName);
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailCountV3() const
    {
        return _weaponCollision.getWeaponBodyCount();
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailsV3(
        ::rock::provider::RockProviderWeaponEvidenceDetailV3* outDetails,
        std::uint32_t maxDetails) const
    {
        if (!outDetails || maxDetails == 0) {
            return 0;
        }

        const auto descriptors = _weaponCollision.getProfileEvidenceDescriptors();
        const std::uint32_t count = (std::min)(maxDetails, static_cast<std::uint32_t>(descriptors.size()));
        const std::uint64_t generationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outDetails[i];
            out = {};
            out.size = sizeof(::rock::provider::RockProviderWeaponEvidenceDetailV3);
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(descriptor.semantic.partKind);
            out.reloadRole = static_cast<std::uint32_t>(descriptor.semantic.reloadRole);
            out.supportRole = static_cast<std::uint32_t>(descriptor.semantic.supportGripRole);
            out.socketRole = static_cast<std::uint32_t>(descriptor.semantic.socketRole);
            out.actionRole = static_cast<std::uint32_t>(descriptor.semantic.actionRole);
            out.fallbackGripPose = static_cast<std::uint32_t>(descriptor.semantic.fallbackGripPose);
            out.interactionRoot = descriptor.geometryRootAddress;
            out.sourceRoot = descriptor.sourceRootAddress;
            out.weaponGenerationKey = generationKey;
            out.localBoundsGame.min = makeProviderPoint(descriptor.localBoundsGame.min);
            out.localBoundsGame.max = makeProviderPoint(descriptor.localBoundsGame.max);
            out.localBoundsGame.valid = descriptor.localBoundsGame.valid ? 1u : 0u;
            out.pointCount = descriptor.pointCount;
            copyProviderString(out.sourceName, sizeof(out.sourceName), descriptor.sourceName);
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEvidenceDetailPointCountV3(std::uint32_t bodyId) const
    {
        WeaponCollisionProfileEvidenceDescriptor descriptor{};
        RE::NiAVObject* sourceNode = nullptr;
        if (!_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode)) {
            return 0;
        }

        return descriptor.pointCount;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetailPointsV3(
        std::uint32_t bodyId,
        ::rock::provider::RockProviderPoint3* outPoints,
        std::uint32_t maxPoints) const
    {
        if (!outPoints || maxPoints == 0) {
            return 0;
        }

        WeaponCollisionProfileEvidenceDescriptor descriptor{};
        RE::NiAVObject* sourceNode = nullptr;
        if (!_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode)) {
            return 0;
        }

        const std::uint32_t copied = (std::min)(maxPoints, static_cast<std::uint32_t>(descriptor.localMeshPointsGame.size()));
        for (std::uint32_t i = 0; i < copied; ++i) {
            outPoints[i] = makeProviderPoint(descriptor.localMeshPointsGame[i]);
        }

        return copied;
    }

    bool PhysicsInteraction::validateCriticalOffsets() const
    {
        REL::Relocation hookSite{ REL::Offset(offsets::kHookSite_MainLoop) };
        auto* hookByte = reinterpret_cast<const std::uint8_t*>(hookSite.address());
        if (*hookByte != 0xE8 && *hookByte != 0xE9) {
            ROCK_LOG_ERROR(Init, "Hook site 0x{:X} is not a CALL/JMP instruction (found {:#x})", offsets::kHookSite_MainLoop, *hookByte);
            return false;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_SAMPLE_DEBUG(Init, g_rockConfig.rockLogSampleMilliseconds, "No bhkWorld available for offset validation (will retry)");
            return true;
        }

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_ERROR(Init, "bhkWorld -> hknpWorld is null — offset may be wrong");
            return false;
        }

        if (!havok_runtime::getBodyArray(hknp)) {
            ROCK_LOG_ERROR(Init, "hknpWorld body array pointer returned null");
            return false;
        }

        ROCK_LOG_INFO(Init, "Critical offset validation passed");
        return true;
    }

    bool PhysicsInteraction::refreshHandBoneCache()
    {
        if (_handBoneCache.resolve()) {
            _handCacheResolveLogCounter = 0;
            return true;
        }

        if (g_rockConfig.rockDebugHandTransformParity) {
            if (++_handCacheResolveLogCounter == 1 || _handCacheResolveLogCounter % 90 == 0) {
                ROCK_LOG_WARN(Hand, "HandBoneCache unresolved; raw parity sampling skipped this frame");
            }
        }

        return false;
    }

    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft) const
    {
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
    {
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
        if (frame.valid) {
            return frame.node;
        }

        return nullptr;
    }

    void PhysicsInteraction::sampleHandTransformParity()
    {
        if (!g_rockConfig.rockDebugHandTransformParity) {
            _parityEnabledLogged = false;
            _paritySummaryCounter = 0;
            return;
        }

        if (!frik::api::FRIKApi::inst || !_handBoneCache.isReady()) {
            return;
        }

        if (!_parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (root flattened cache vs FRIK API, pre-write sampling)");
            _parityEnabledLogged = true;
        }

        const bool playerMoving = frik::api::FRIKApi::inst->isPlayerMoving();
        const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _rawHandParityStates[isLeft ? 1 : 0];
            const auto handEnum = handFromBool(isLeft);
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            const auto apiTransform = frik::api::FRIKApi::inst->getHandWorldTransform(handEnum);
            const auto delta = measureTransformDelta(localTransform, apiTransform);
            const auto localPalmPosition = computePalmPositionFromHandBasis(localTransform, isLeft);
            const auto apiPalmPosition = computePalmPositionFromHandBasis(apiTransform, isLeft);
            const auto localPalmNormal = computePalmNormalFromHandBasis(localTransform, isLeft);
            const auto apiPalmNormal = computePalmNormalFromHandBasis(apiTransform, isLeft);
            const auto localPointing = computePointingVectorFromHandBasis(localTransform, isLeft);
            const auto apiPointing = computePointingVectorFromHandBasis(apiTransform, isLeft);
            state.lastPositionDelta = delta.position;
            state.lastRotationDeltaDegrees = delta.rotationDegrees;

            const bool warnExceeded = delta.position > kRawParityWarnPosition || delta.rotationDegrees > kRawParityWarnRotationDegrees;
            const bool failExceeded = delta.position > kRawParityFailPosition || delta.rotationDegrees > kRawParityFailRotationDegrees;

            state.warnFrames = warnExceeded ? state.warnFrames + 1 : 0;
            state.failFrames = failExceeded ? state.failFrames + 1 : 0;

            const char* handLabel = isLeft ? "Left" : "Right";
            if (state.warnFrames == kRawParityWarnFrames) {
                ROCK_LOG_WARN(Hand, "{} raw hand parity warning: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (state.failFrames == kRawParityFailFrames) {
                ROCK_LOG_ERROR(Hand, "{} raw hand parity failure: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (playerMoving && state.hasPreviousApiTransform) {
                const auto prevApiDelta = measureTransformDelta(localTransform, state.previousApiTransform);
                if (prevApiDelta.position + kRawParityLagSlack < delta.position) {
                    state.lagFrames++;
                    if (state.lagFrames == kRawParityLagFrames) {
                        ROCK_LOG_WARN(Hand, "{} hand parity suggests possible one-frame lag: currentDelta={:.3f} prevApiDelta={:.3f}", handLabel, delta.position,
                            prevApiDelta.position);
                    }
                } else {
                    state.lagFrames = 0;
                }
            } else {
                state.lagFrames = 0;
            }

            state.previousApiTransform = apiTransform;
            state.hasPreviousApiTransform = true;

            if (emitSummary) {
                const char* summaryHandLabel = isLeft ? "L" : "R";
                ROCK_LOG_DEBUG(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
                    delta.position, delta.rotationDegrees, measurePointDelta(localPalmPosition, apiPalmPosition), measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
                    measureDirectionDeltaDegrees(localPointing, apiPointing));
            }
        };

        sampleHand(false);
        sampleHand(true);

        if (emitSummary) {
            _paritySummaryCounter = 0;
            const auto& right = _rawHandParityStates[0];
            const auto& left = _rawHandParityStates[1];
            ROCK_LOG_DEBUG(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
                left.lastPositionDelta, left.lastRotationDeltaDegrees);
        }
    }

    void PhysicsInteraction::init()
    {
        if (_initialized) {
            ROCK_LOG_WARN(Init, "init() called but already initialized — skipping");
            return;
        }

        if (!validateCriticalOffsets()) {
            ROCK_LOG_CRITICAL(Init,
                "ROCK DISABLED: critical Havok offset validation failed. "
                "This likely means a game update changed memory layouts.");
            return;
        }

        if (!installNativeMeleeSuppressionHooks() && g_rockConfig.rockNativeMeleeSuppressionEnabled) {
            ROCK_LOG_CRITICAL(Init, "Native melee suppression requested but hook installation failed; ROCK will continue without melee suppression");
        }

        ROCK_LOG_INFO(Init, "Initializing ROCK physics module...");

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_ERROR(Init, "Failed to get bhkWorld during init — deferring");
            return;
        }

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_ERROR(Init, "Failed to get hknpWorld during init — deferring");
            return;
        }

        physics_scale::refreshAndLogIfChanged();

        _cachedBhkWorld = bhk;
        if (!refreshHandBoneCache()) {
            ROCK_LOG_WARN(Init, "HandBoneCache not ready during init; runtime remains on pre-00 transform paths");
        }

        registerCollisionLayer(hknp);
        if (!_collisionLayerRegistered) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: collision layer registration failed");
            _cachedBhkWorld = nullptr;
            return;
        }

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: hand collision body creation failed");
            _cachedBhkWorld = nullptr;
            return;
        }

        _handContactActivity.reset();
        subscribeContactEvents(hknp);

        _weaponCollision.init(hknp, bhk);

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", true);
            ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
        }

        {
            _rightHand.updateCollisionTransform(hknp, 0.011f);
            _leftHand.updateCollisionTransform(hknp, 0.011f);
            ROCK_LOG_INFO(Init, "Initial bone-derived hand collider transforms updated");
        }

        _hasPrevPositions = false;
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _deltaLogCounter = 0;
        _contactLogCounter = 0;
        _dynamicPushElapsedSeconds = 0.0f;
        _dynamicPushCooldownUntil.clear();
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _handContactActivity.reset();

        _initialized = true;

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsInit, false);

        ROCK_LOG_INFO(Init, "ROCK physics module initialized — bhkWorld={}, hknpWorld={}, R_body={}, L_body={}", static_cast<const void*>(bhk), static_cast<const void*>(hknp),
            _rightHand.getCollisionBodyId().value, _leftHand.getCollisionBodyId().value);
    }

    PhysicsFrameContext PhysicsInteraction::buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds)
    {
        PhysicsFrameContext frame{};
        frame.bhkWorld = bhk;
        frame.hknpWorld = hknp;
        frame.deltaSeconds = (deltaSeconds > 0.0f && deltaSeconds <= 0.1f) ? deltaSeconds : (1.0f / 90.0f);
        frame.worldReady = bhk && hknp;
        frame.menuBlocked = frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->isAnyMenuOpen();
        frame.reloadBoundaryActive = false;

        auto buildHandInput = [&](bool isLeft, Hand& hand) {
            HandFrameInput input{};
            input.isLeft = isLeft;
            const bool rootHandReady = _handBoneCache.isReady();
            input.disabled = (isLeft ? s_leftHandDisabled.load(std::memory_order_acquire) : s_rightHandDisabled.load(std::memory_order_acquire)) || !rootHandReady;
            if (!rootHandReady) {
                return input;
            }

            input.rawHandWorld = getInteractionHandTransform(isLeft);
            input.handNode = getInteractionHandNode(isLeft);
            if (frame.worldReady) {
                input.grabAnchorWorld = hand.computeGrabPivotAWorld(hknp, input.rawHandWorld);
            }
            input.palmNormalWorld = computePalmNormalFromHandBasis(input.rawHandWorld, isLeft);
            input.pointingWorld = computePointingVectorFromHandBasis(input.rawHandWorld, isLeft);
            return input;
        };

        frame.right = buildHandInput(false, _rightHand);
        frame.left = buildHandInput(true, _leftHand);
        return frame;
    }

    void PhysicsInteraction::update()
    {
        if (!frik::api::FRIKApi::inst) {
            return;
        }

        vrcf::VRControllers.update(f4vr::isLeftHandedMode());

        _deltaTime = frik::api::FRIKApi::inst->getFrameTime();

        if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
            _deltaTime = 1.0f / 90.0f;
        }
        _dynamicPushElapsedSeconds += _deltaTime;
        if (_dynamicPushCooldownUntil.size() > 512) {
            for (auto it = _dynamicPushCooldownUntil.begin(); it != _dynamicPushCooldownUntil.end();) {
                if (it->second <= _dynamicPushElapsedSeconds) {
                    it = _dynamicPushCooldownUntil.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (!frik::api::FRIKApi::inst->isSkeletonReady()) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "Skeleton no longer ready — shutting down");
                shutdown();
            }
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                frik::api::FRIKApi::inst->isAnyMenuOpen(),
                false,
                false)) {
            if (_initialized) {
                _twoHandedGrip.reset();
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        restoreLeftHandCollisionAfterWeaponSupport(hknpMenu);
                        if (_rightHand.isHolding()) {
                            auto* r = _rightHand.getHeldRef();
                            _rightHand.releaseGrabbedObject(hknpMenu);
                            if (r)
                                releaseObject(r);
                        }
                        if (_leftHand.isHolding()) {
                            auto* r = _leftHand.getHeldRef();
                            _leftHand.releaseGrabbedObject(hknpMenu);
                            if (r)
                                releaseObject(r);
                        }
                    }
                } else {
                    hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
                }
            }
            debug::ClearFrame();
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                false,
                !g_rockConfig.rockEnabled,
                false)) {
            if (_initialized) {
                shutdown();
            }
            debug::ClearFrame();
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return;
        }

        if (_initialized && bhk != _cachedBhkWorld) {
            ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");

            shutdown();
        }

        if (!_initialized) {
            init();
            if (!_initialized) {
                return;
            }
        }

        _cachedBhkWorld = bhk;

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            debug::ClearFrame();
            return;
        }

        if (physics_scale::refreshAndLogIfChanged()) {
            ROCK_LOG_WARN(Config, "Authoritative Havok scale changed; invalidating ROCK-generated collision bodies");
            /*
             * A live scale change is a physics-frame convention change, not a
             * cosmetic setting reload. Existing constraints and ROCK-owned shapes
             * were authored with the previous conversion, so active interactions
             * must yield before generated bodies are destroyed and rebuilt.
             */
            auto releaseHeldForScaleChange = [&](Hand& hand) {
                if (!hand.isHolding()) {
                    return;
                }

                auto* heldRef = hand.getHeldRef();
                hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate);
                if (heldRef) {
                    releaseObject(heldRef);
                }
            };
            releaseHeldForScaleChange(_rightHand);
            releaseHeldForScaleChange(_leftHand);

            restoreLeftHandCollisionAfterWeaponSupport(hknp);
            _twoHandedGrip.reset();
            clearLeftWeaponContact();

            destroyHandCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        const auto frame = buildFrameContext(bhk, hknp, _deltaTime);
        _generatedBodyStepDrive.registerForNextStep(bhk, hknp);

        if (_collisionLayerRegistered && (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0 || _expectedReloadLayerMask != 0)) {
            if (auto* filterPtr = getQueryFilterRef(hknp)) {
                    auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);
                    const auto currentHandMask = matrix[ROCK_HAND_LAYER];
                    const auto currentWeaponMask = matrix[ROCK_WEAPON_LAYER];
                    const auto currentReloadMask = matrix[collision_layer_policy::ROCK_LAYER_RELOAD];
                    const bool handMaskDrifted =
                        _expectedHandLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                    const bool weaponMaskDrifted =
                        _expectedWeaponLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                    const bool reloadMaskDrifted =
                        _expectedReloadLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
                    if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted) {
                        ROCK_LOG_WARN(Config,
                            "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}; re-registering",
                            collision_layer_policy::configuredMask(_expectedHandLayerMask),
                            collision_layer_policy::configuredMask(currentHandMask),
                            collision_layer_policy::configuredMask(_expectedWeaponLayerMask),
                            collision_layer_policy::configuredMask(currentWeaponMask),
                            collision_layer_policy::configuredMask(_expectedReloadLayerMask),
                            collision_layer_policy::configuredMask(currentReloadMask));
                        _collisionLayerRegistered = false;
                        registerCollisionLayer(hknp);
                    }
            }
        }

        updateHandCollisions(frame);

        {
            RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();

            auto dominantHandBodyId = _rightHand.getCollisionBodyId();

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_wpnNodeLogCounter >= 90) {
                    _wpnNodeLogCounter = 0;
                    if (weaponNode) {
                        ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponNode->name.c_str(), weaponNode->world.translate.x,
                            weaponNode->world.translate.y, weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(hknp, weaponNode, dominantHandBodyId, frame.deltaSeconds);
        }

        {
            WeaponInteractionContact leftWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;
            RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();

            auto publishLeftWeaponProbeContact = [&](WeaponInteractionContact& contact) {
                _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(contact.partKind), std::memory_order_release);
                _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(contact.reloadRole), std::memory_order_release);
                _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(contact.supportGripRole), std::memory_order_release);
                _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(contact.socketRole), std::memory_order_release);
                _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(contact.actionRole), std::memory_order_release);
                _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(contact.fallbackGripPose), std::memory_order_release);
                contact.sequence = _leftWeaponContactSequence.fetch_add(1, std::memory_order_acq_rel) + 1;
                _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
            };

            const std::uint32_t leftWeaponBodyId = _leftWeaponContactBodyId.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);
            if (leftWeaponBodyId != INVALID_CONTACT_BODY_ID) {
                _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
                if (!_weaponCollision.tryGetWeaponContactAtomic(leftWeaponBodyId, leftWeaponContact)) {
                    leftWeaponContact.valid = true;
                    leftWeaponContact.bodyId = leftWeaponBodyId;
                    leftWeaponContact.partKind = static_cast<WeaponPartKind>(_leftWeaponContactPartKind.load(std::memory_order_acquire));
                    leftWeaponContact.reloadRole = static_cast<WeaponReloadRole>(_leftWeaponContactReloadRole.load(std::memory_order_acquire));
                    leftWeaponContact.supportGripRole = static_cast<WeaponSupportGripRole>(_leftWeaponContactSupportRole.load(std::memory_order_acquire));
                    leftWeaponContact.socketRole = static_cast<WeaponSocketRole>(_leftWeaponContactSocketRole.load(std::memory_order_acquire));
                    leftWeaponContact.actionRole = static_cast<WeaponActionRole>(_leftWeaponContactActionRole.load(std::memory_order_acquire));
                    leftWeaponContact.fallbackGripPose = static_cast<WeaponGripPoseId>(_leftWeaponContactGripPose.load(std::memory_order_acquire));
                }
                leftWeaponContact.sequence = _leftWeaponContactSequence.load(std::memory_order_acquire);
                leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::Contact;
            } else if (weaponNode) {
                const RE::NiPoint3 leftProbePoint = frame.left.grabAnchorWorld;
                if (_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, leftProbePoint, g_rockConfig.rockWeaponInteractionProbeRadius, leftWeaponContact)) {
                    publishLeftWeaponProbeContact(leftWeaponContact);
                    leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::Probe;
                    if (g_rockConfig.rockDebugVerboseLogging && ++_weaponInteractionProbeLogCounter >= 90) {
                        _weaponInteractionProbeLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "WeaponInteractionProbe: bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                            leftWeaponContact.bodyId,
                            static_cast<int>(leftWeaponContact.partKind),
                            static_cast<int>(leftWeaponContact.supportGripRole),
                            static_cast<int>(leftWeaponContact.reloadRole),
                            static_cast<int>(leftWeaponContact.actionRole),
                            g_rockConfig.rockWeaponInteractionProbeRadius);
                    }
                } else {
                    const auto missedFrames = _leftWeaponContactMissedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                    if (missedFrames > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                        clearLeftWeaponContact();
                    }
                }
            } else {
                const auto missedFrames = _leftWeaponContactMissedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (missedFrames > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                    clearLeftWeaponContact();
                }
            }

            const bool gripPressed = readGrabButtonHeld(true, g_rockConfig.rockGrabButtonID);
            const bool gripConfirmPressed = readGrabButtonPressedEdge(true, g_rockConfig.rockGrabButtonID);
            (void)gripConfirmPressed;

            WeaponInteractionRuntimeState providerInteractionState{};
            const auto offhandReservation = offhand_interaction_reservation::fromProvider(::rock::provider::currentOffhandReservation());
            if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
                providerInteractionState.supportGripAllowed = false;
            }

            const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, providerInteractionState);
            const std::uint64_t currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
            const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
                leftWeaponContact,
                leftWeaponDecision,
                leftWeaponContactSource);

            const bool leftHandHoldingObject = _leftHand.isHolding();
            const auto supportAuthorityMode = resolveEquippedWeaponSupportAuthorityMode(weaponNode);

            _twoHandedGrip.update(
                weaponNode,
                leftWeaponContact,
                gripPressed,
                leftHandHoldingObject,
                frame.deltaSeconds,
                currentWeaponGenerationKey,
                providerInteractionState,
                supportAuthorityMode);
            const bool weaponSupportGripActive = _twoHandedGrip.isGripping();
            if (g_rockConfig.rockDebugShowWeaponNotifications) {
                const auto gripNotificationEvent =
                    weapon_debug_notification_policy::observeWeaponSupportGrip(_weaponDebugNotificationState, weaponSupportGripActive);
                if (gripNotificationEvent != weapon_debug_notification_policy::WeaponGripNotificationEvent::None) {
                    if (gripNotificationEvent == weapon_debug_notification_policy::WeaponGripNotificationEvent::Started) {
                        const auto weaponDebugInfo = makeWeaponInteractionDebugInfo(_weaponCollision, weaponNode, leftWeaponContact);
                        f4vr::showNotification(
                            weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey, weaponDebugInfo));
                        ROCK_LOG_INFO(Weapon,
                            "WeaponGripDiagnostics: weapon='{}' formID={:08X} node='{}' root='{}' nif='{}' part={} route={} pose={} body={} source={}",
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponName),
                            weaponDebugInfo.weaponFormId,
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponNodeName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceName),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.partKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.interactionKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.gripPose),
                            weaponNotificationKey.bodyId,
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.source));
                    } else {
                        f4vr::showNotification(weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey));
                    }
                }
            } else {
                _weaponDebugNotificationState.supportGripActive = weaponSupportGripActive;
            }

            if (weaponSupportGripActive) {
                suppressLeftHandCollisionForWeaponSupport(hknp);
            } else {
                restoreLeftHandCollisionAfterWeaponSupport(hknp);
            }

            _weaponCollision.updateBodiesFromCurrentSourceTransforms(hknp, weaponNode);
            if (f4vr::isNodeVisible(weaponNode)) {
                applyFinalWeaponMuzzleAuthority();
            }
        }

        updateSelection(frame);

        _heldObjectPlayerSpaceFrame = sampleHeldObjectPlayerSpaceFrame(frame.deltaSeconds);
        /*
         * HIGGS applies player/room-space compensation before held-object grab
         * constraints are updated. Doing the same in ROCK keeps the constraint
         * target from solving against a stale body velocity and removes the
         * apparent held-object teleport/stutter caused by compensating after the
         * grab loop has already written the frame target.
         */
        applyHeldPlayerSpaceVelocity(hknp);

        updateGrabInput(frame);

        publishDebugBodyOverlay(frame);

        resolveContacts(frame);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        _rightHand.tickTouchState();
        _leftHand.tickTouchState();
        _rightHand.tickSemanticContactState();
        _leftHand.tickSemanticContactState();
        _handContactActivity.advanceFrame();
        if (wasTouchingR && !_rightHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false, _rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(), _rightHand.getLastTouchedLayer());
        }
        if (wasTouchingL && !_leftHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true, _leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(), _leftHand.getLastTouchedLayer());
        }

        _deltaLogCounter++;
        if (g_rockConfig.rockDebugVerboseLogging && _deltaLogCounter >= 90) {
            _deltaLogCounter = 0;

            if (frik::api::FRIKApi::inst) {
                const auto smoothPos = frik::api::FRIKApi::inst->getSmoothedPlayerPosition();
                const bool moving = frik::api::FRIKApi::inst->isPlayerMoving();

                if (_hasPrevPositions && moving) {
                    const auto smoothDelta = smoothPos - _prevSmoothedPos;

                    ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}", smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
                }

                _prevSmoothedPos = smoothPos;
                _hasPrevPositions = true;
            }
        }

        ::rock::provider::dispatchFrameCallbacks(*this);
    }

    void PhysicsInteraction::clearLeftWeaponContact()
    {
        _leftWeaponContactBodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _leftWeaponContactMissedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
    }

    void PhysicsInteraction::suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world)
    {
        /*
         * Layer 43 vs 44 is intentionally allowed so the offhand can physically
         * touch the equipped weapon before support grip starts. Once support
         * grip owns the transform, the left hand body becomes a driver and must
         * stop solving against the weapon package, matching the HIGGS pattern
         * used for held-object hand collision suppression.
         */
        if (!world || !_leftHand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }

            const auto suppression = hand_collision_suppression_math::beginSuppression(_leftWeaponSupportCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left hand support suppression set full; bodyId={} left active", bodyId);
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand,
                "weapon-support-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: left hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _leftHand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_leftHand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(_leftHand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_leftWeaponSupportCollisionSuppression)) {
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: cannot restore left hand support collision yet (world=null); preserving suppression leases");
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : _leftWeaponSupportCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_CONTACT_BODY_ID) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand,
                "weapon-support-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }

            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: left hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }

        if (restoreDeferred) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left hand support collision restore deferred; suppression leases preserved");
            return;
        }

        hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
    }

    void PhysicsInteraction::shutdown()
    {
        if (!_initialized) {
            return;
        }

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsShutdown, false);

        ROCK_LOG_INFO(Init, "Shutting down ROCK physics module...");

        auto* currentBhk = getPlayerBhkWorld();
        const bool worldValid = _cachedBhkWorld && currentBhk == _cachedBhkWorld;

        if (worldValid) {
            auto* hknp = getHknpWorld(_cachedBhkWorld);
            unsubscribeContactEvents(hknp);
            restoreLeftHandCollisionAfterWeaponSupport(hknp);
            if (_rightHand.isHolding()) {
                auto* r = _rightHand.getHeldRef();
                _rightHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate);
                if (r)
                    releaseObject(r);
            }
            if (_leftHand.isHolding()) {
                auto* r = _leftHand.getHeldRef();
                _leftHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate);
                if (r)
                    releaseObject(r);
            }
            _weaponCollision.destroyWeaponBody(hknp);
            destroyHandCollisions(_cachedBhkWorld);
        } else {
            unsubscribeContactEvents(nullptr);
            ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        debug::ClearFrame();
        _twoHandedGrip.reset();
        _weaponCollision.shutdown();
        _generatedBodyStepDrive.reset();
        collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        ::rock::provider::clearExternalBodiesForProviderLoss();
        clearLeftWeaponContact();
        releaseAllObjects();
        _rightHand.reset();
        _leftHand.reset();

        _cachedBhkWorld = nullptr;
        _collisionLayerRegistered = false;
        _initialized = false;
        _hasPrevPositions = false;
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _handBoneCache.reset();
        _handCacheResolveLogCounter = 0;
        _paritySummaryCounter = 0;
        _parityEnabledLogged = false;
        _runtimeScaleLogged = false;
        _rawHandParityStates = {};
        _dynamicPushCooldownUntil.clear();
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _handContactActivity.reset();

        cleanupGrabConstraintVtable();

        ROCK_LOG_INFO(Init, "ROCK physics module shut down");
    }

    void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
    {
        PhysicsEventData data{ isLeft, refr, formID, layer };

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(msgType, &data, sizeof(data), nullptr);
        }
    }

    void PhysicsInteraction::registerCollisionLayer(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Config, "registerCollisionLayer: world is null");
            return;
        }

        auto* filterPtr = getQueryFilterRef(world);
        if (!filterPtr) {
            static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
            filterPtr = *filterSingleton;
            if (!filterPtr) {
                ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
                return;
            }
            ROCK_LOG_WARN(Config, "World filter null, fell back to global singleton at {:p}", filterPtr);
        }

        {
            static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
            auto* globalFilter = *filterSingleton;
            ROCK_LOG_DEBUG(Config, "Filter source: world={:p}, global={:p}, same={}", filterPtr, (void*)globalFilter, filterPtr == globalFilter);
        }

        auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);

        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_RELOAD, matrix[collision_layer_policy::ROCK_LAYER_RELOAD]);

        collision_layer_policy::applyRockHandLayerPolicy(matrix, true);
        collision_layer_policy::applyRockWeaponLayerPolicy(
            matrix, g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);
        collision_layer_policy::applyRockReloadLayerPolicy(
            matrix, g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);

        _expectedHandLayerMask = collision_layer_policy::buildRockHandExpectedMask(true);
        _expectedWeaponLayerMask =
            collision_layer_policy::buildRockWeaponExpectedMask(g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);
        _expectedReloadLayerMask =
            collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);
        _collisionLayerRegistered = true;

        ROCK_LOG_INFO(Config, "Registered ROCK collision layers: hand={} mask=0x{:016X}, weapon={} mask=0x{:016X}, reload={} mask=0x{:016X}, projectiles={}, spells={}",
            ROCK_HAND_LAYER,
            matrix[ROCK_HAND_LAYER],
            ROCK_WEAPON_LAYER,
            matrix[ROCK_WEAPON_LAYER],
            collision_layer_policy::ROCK_LAYER_RELOAD,
            matrix[collision_layer_policy::ROCK_LAYER_RELOAD],
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled");
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const bool rightOk = _rightHand.createCollision(world, bhkWorld);

        const bool leftOk = _leftHand.createCollision(world, bhkWorld);

        if (!rightOk || !leftOk) {
            ROCK_LOG_ERROR(Hand, "Hand collision creation failed (rightOk={}, leftOk={})", rightOk, leftOk);
            if (rightOk)
                _rightHand.destroyCollision(bhkWorld);
            if (leftOk)
                _leftHand.destroyCollision(bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(Hand,
            "Bone-derived hand collision created: rightBodies={} leftBodies={} mode={} requireAnchor={} requireAllFingerBones={}",
            _rightHand.getHandColliderBodyCount(),
            _leftHand.getHandColliderBodyCount(),
            g_rockConfig.rockHandColliderRuntimeMode,
            g_rockConfig.rockHandBoneCollidersRequirePalmAnchor ? "true" : "false",
            g_rockConfig.rockHandBoneCollidersRequireAllFingerBones ? "true" : "false");

        return true;
    }

    void PhysicsInteraction::destroyHandCollisions(void* bhkWorld)
    {
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
    }

    void PhysicsInteraction::updateHandCollisions(const PhysicsFrameContext& frame)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
            return;
        }

        auto* world = frame.hknpWorld;

        _rightHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        _leftHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);

        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.deltaSeconds);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.deltaSeconds);
        }
    }

    void PhysicsInteraction::onGeneratedBodyPhysicsStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveGeneratedBodiesFromPhysicsStep(world, timing);
    }

    void PhysicsInteraction::driveGeneratedBodiesFromPhysicsStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire)) {
            return;
        }

        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
    }

    void PhysicsInteraction::publishDebugBodyOverlay(const PhysicsFrameContext& context)
    {
        auto* hknp = context.hknpWorld;
        const bool drawRockColliderBodies = g_rockConfig.rockDebugShowColliders;
        const bool drawGrabPivots = g_rockConfig.rockDebugShowGrabPivots;
        const bool drawFingerProbes = g_rockConfig.rockDebugShowGrabFingerProbes;
        const bool drawPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        const bool drawRootFlattenedFingerSkeleton = g_rockConfig.rockDebugShowRootFlattenedFingerSkeletonMarkers;
        const auto skeletonBoneMode = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneMode(g_rockConfig.rockDebugSkeletonBoneMode);
        const auto skeletonBoneSource = skeleton_bone_debug_math::sanitizeDebugSkeletonBoneSource(g_rockConfig.rockDebugSkeletonBoneSource);
        const bool drawSkeletonBones =
            g_rockConfig.rockDebugShowSkeletonBoneVisualizer && skeletonBoneMode != skeleton_bone_debug_math::DebugSkeletonBoneMode::Off;
        const bool drawGrabSurfaceFrame = g_rockConfig.rockDebugShowGrabSurfaceFrame;
        const bool drawGrabContactPatch = g_rockConfig.rockDebugDrawGrabContactPatch;
        const bool drawHandBoneContacts = g_rockConfig.rockDebugDrawHandBoneContacts;
        const bool drawGrabTransformTelemetry = g_rockConfig.rockDebugGrabTransformTelemetry;
        const bool drawGrabTransformTelemetryAxes = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryAxes;
        const bool drawGrabTransformTelemetryText = drawGrabTransformTelemetry && g_rockConfig.rockDebugGrabTransformTelemetryText;
        const bool drawWeaponAuthorityDebug = _twoHandedGrip.isGripping() && (g_rockConfig.rockDebugShowHandAxes || drawGrabPivots);
        const bool drawWorldOriginDiagnostics = g_rockConfig.rockDebugWorldObjectOriginDiagnostics;
        if (drawWorldOriginDiagnostics && !s_worldOriginDiagnosticsEnabledLogged) {
            ROCK_LOG_INFO(Hand,
                "World object origin diagnostics enabled: intervalFrames={} warnThresholdGameUnits={:.2f} visualSourceOrder=bodyOwnerNode>hitNode>visualNode>referenceRoot",
                g_rockConfig.rockDebugWorldObjectOriginLogIntervalFrames,
                g_rockConfig.rockDebugWorldObjectOriginMismatchWarnGameUnits);
            s_worldOriginDiagnosticsEnabledLogged = true;
        } else if (!drawWorldOriginDiagnostics) {
            s_worldOriginDiagnosticsEnabledLogged = false;
        }
        if (!drawRockColliderBodies && !g_rockConfig.rockDebugShowTargetColliders && !g_rockConfig.rockDebugShowHandAxes && !drawGrabPivots && !drawFingerProbes &&
            !drawPalmVectors && !drawRootFlattenedFingerSkeleton && !drawSkeletonBones && !drawGrabSurfaceFrame && !drawGrabContactPatch && !drawHandBoneContacts &&
            !drawGrabTransformTelemetry && !drawWeaponAuthorityDebug && !drawWorldOriginDiagnostics) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.drawRockBodies = drawRockColliderBodies;
        frame.drawTargetBodies = g_rockConfig.rockDebugShowTargetColliders;
        frame.drawAxes = g_rockConfig.rockDebugShowHandAxes || drawGrabTransformTelemetryAxes;
        frame.drawMarkers =
            drawGrabPivots || drawFingerProbes || drawPalmVectors || drawRootFlattenedFingerSkeleton || drawGrabSurfaceFrame || drawGrabContactPatch ||
            drawHandBoneContacts || drawGrabTransformTelemetryAxes || drawWeaponAuthorityDebug || drawWorldOriginDiagnostics;
        frame.drawSkeleton = drawSkeletonBones;
        frame.drawText = drawGrabTransformTelemetryText;
        RE::bhkWorld* originDiagnosticBhk = drawWorldOriginDiagnostics ? context.bhkWorld : nullptr;
        const bool rightDisabled = context.right.disabled;
        const bool leftDisabled = context.left.disabled;

        auto addBody = [&](RE::hknpBodyId bodyId, debug::BodyOverlayRole role) {
            if (bodyId.value == INVALID_BODY_ID || frame.count >= frame.entries.size()) {
                return;
            }

            for (std::uint32_t i = 0; i < frame.count; i++) {
                if (frame.entries[i].bodyId.value == bodyId.value && frame.entries[i].role == role) {
                    return;
                }
            }

            frame.entries[frame.count++] = debug::BodyOverlayEntry{ bodyId, role };
        };

        auto addAxisTransform = [&](const RE::NiTransform& transform, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Transform;
            entry.role = role;
            entry.transform = transform;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addAxisBody = [&](RE::hknpBodyId bodyId, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || bodyId.value == INVALID_BODY_ID || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Body;
            entry.role = role;
            entry.bodyId = bodyId;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addMarker = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, const RE::NiPoint3& lineEnd, float size, bool drawPoint, bool drawLine) {
            if (!frame.drawMarkers || frame.markerCount >= frame.markerEntries.size()) {
                return;
            }

            auto& entry = frame.markerEntries[frame.markerCount++];
            entry.role = role;
            entry.position = position;
            entry.lineEnd = lineEnd;
            entry.size = size;
            entry.drawPoint = drawPoint;
            entry.drawLine = drawLine;
        };

        auto addMarkerPoint = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, float size) {
            addMarker(role, position, position, size, true, false);
        };

        auto addMarkerRay = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end, float startSize) {
            addMarker(role, start, end, startSize, true, true);
        };

        auto addMarkerLine = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end) {
            addMarker(role, start, end, 0.0f, false, true);
        };

        auto addTextLine = [&](const RE::NiPoint3& worldAnchor, const float color[4], const char* format, auto&&... args) {
            if (!frame.drawText || frame.textCount >= frame.textEntries.size()) {
                return;
            }

            auto& entry = frame.textEntries[frame.textCount++];
            entry.x = 20.0f;
            entry.y = 0.0f;
            entry.size = 3.0f;
            entry.color[0] = color[0];
            entry.color[1] = color[1];
            entry.color[2] = color[2];
            entry.color[3] = color[3];
            entry.worldAnchor = worldAnchor;
            entry.worldAnchored = true;
            std::snprintf(entry.text, sizeof(entry.text), format, std::forward<decltype(args)>(args)...);
        };

        auto tryResolveBodyPosition = [&](std::uint32_t bodyId, RE::NiPoint3& outPosition) {
            if (!hknp || bodyId == INVALID_CONTACT_BODY_ID || bodyId == INVALID_BODY_ID) {
                return false;
            }
            RE::NiTransform bodyWorld{};
            if (!tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                return false;
            }
            outPosition = bodyWorld.translate;
            return true;
        };

        auto addWorldOriginDiagnostic =
            [&](const Hand& hand, bool held, RE::hknpBodyId bodyId, RE::TESObjectREFR* refr, RE::NiAVObject* hitNode, RE::NiAVObject* visualNode) {
                if (!drawWorldOriginDiagnostics || !originDiagnosticBhk || !hknp) {
                    return;
                }

                origin_diagnostics::TargetOriginSample sample{};
                if (!origin_diagnostics::sampleTarget(originDiagnosticBhk,
                        hknp,
                        bodyId,
                        refr,
                        hitNode,
                        visualNode,
                        g_rockConfig.rockDebugWorldObjectOriginMismatchWarnGameUnits,
                        sample)) {
                    return;
                }

                origin_diagnostics::logSampleIfNeeded(hand.handName(),
                    held,
                    sample,
                    static_cast<std::uint32_t>((std::max)(g_rockConfig.rockDebugWorldObjectOriginLogIntervalFrames, 1)));
                origin_diagnostics::publishMarkers(frame, sample);
            };

        auto addSkeletonBone = [&](const DirectSkeletonBoneSnapshot& snapshot, std::size_t boneIndex, bool drawAxis) {
            if (!frame.drawSkeleton || boneIndex >= snapshot.bones.size() || frame.skeletonCount >= frame.skeletonEntries.size()) {
                return;
            }

            const auto& bone = snapshot.bones[boneIndex];
            auto& entry = frame.skeletonEntries[frame.skeletonCount++];
            entry.role = skeletonOverlayRoleForBone(bone.name);
            entry.transform = bone.world;
            entry.pointSize = g_rockConfig.rockDebugSkeletonBonePointSize;
            entry.axisLength = g_rockConfig.rockDebugSkeletonBoneAxisLength;
            entry.drawPoint = true;
            entry.drawAxis = drawAxis;
            entry.inPowerArmor = snapshot.inPowerArmor;
            if (bone.drawableParentSnapshotIndex >= 0 && static_cast<std::size_t>(bone.drawableParentSnapshotIndex) < snapshot.bones.size()) {
                entry.parentPosition = snapshot.bones[bone.drawableParentSnapshotIndex].world.translate;
                entry.hasParent = true;
            }
        };

        auto pointDistance = [](const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) {
            const RE::NiPoint3 delta = lhs - rhs;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        };

        if (frame.drawAxes) {
            if (!rightDisabled) {
                const RE::NiTransform& rawHand = context.right.rawHandWorld;
                addAxisTransform(rawHand, debug::AxisOverlayRole::RightHandRaw, rawHand.translate, false);
                addAxisBody(_rightHand.getCollisionBodyId(), debug::AxisOverlayRole::RightHandBody, rawHand.translate, true);
            }

            if (!leftDisabled) {
                const RE::NiTransform& rawHand = context.left.rawHandWorld;
                addAxisTransform(rawHand, debug::AxisOverlayRole::LeftHandRaw, rawHand.translate, false);
                addAxisBody(_leftHand.getCollisionBodyId(), debug::AxisOverlayRole::LeftHandBody, rawHand.translate, true);
            }
        }

        if (drawPalmVectors) {
            auto addPalmVectorDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                const auto& handInput = isLeft ? context.left : context.right;
                const RE::NiPoint3 grabAnchor = handInput.grabAnchorWorld;
                const RE::NiPoint3 palmNormal = handInput.palmNormalWorld;
                const RE::NiPoint3 pointing = handInput.pointingWorld;
                const float palmNormalLength = (std::max)(5.0f, g_rockConfig.rockNearDetectionRange);
                const float pointingLength = (std::min)(90.0f, (std::max)(20.0f, g_rockConfig.rockFarDetectionRange));

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabAnchor : debug::MarkerOverlayRole::RightGrabAnchor, grabAnchor, 2.0f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPalmNormal : debug::MarkerOverlayRole::RightPalmNormal, grabAnchor,
                    grabAnchor + palmNormal * palmNormalLength, 1.6f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPointing : debug::MarkerOverlayRole::RightPointing, grabAnchor,
                    grabAnchor + pointing * pointingLength, 1.2f);
            };

            addPalmVectorDebug(false);
            addPalmVectorDebug(true);
        }

        if (drawSkeletonBones) {
            DirectSkeletonBoneSnapshot snapshot{};
            if (s_directSkeletonBoneReader.capture(skeletonBoneMode, skeletonBoneSource, snapshot)) {
                const std::size_t drawCap =
                    static_cast<std::size_t>(skeleton_bone_debug_math::sanitizeMaxSkeletonBonesDrawn(g_rockConfig.rockDebugMaxSkeletonBonesDrawn));
                const std::size_t axisCap =
                    static_cast<std::size_t>(skeleton_bone_debug_math::sanitizeMaxSkeletonAxesDrawn(g_rockConfig.rockDebugMaxSkeletonBoneAxesDrawn));
                std::size_t axesDrawn = 0;
                std::size_t skippedBones = 0;
                for (std::size_t i = 0; i < snapshot.bones.size(); ++i) {
                    if (i >= drawCap || frame.skeletonCount >= frame.skeletonEntries.size()) {
                        ++skippedBones;
                        continue;
                    }

                    const bool drawAxis =
                        g_rockConfig.rockDebugDrawSkeletonBoneAxes &&
                        skeleton_bone_debug_math::shouldDrawSkeletonAxis(g_rockConfig.rockDebugSkeletonAxisBoneFilter, snapshot.bones[i].name, axesDrawn, axisCap);
                    if (drawAxis) {
                        ++axesDrawn;
                    }
                    addSkeletonBone(snapshot, i, drawAxis);
                }

                if (skippedBones > 0 && g_rockConfig.rockDebugLogSkeletonBoneTruncation) {
                    ROCK_LOG_WARN(Hand,
                        "Direct skeleton overlay truncated: source={} mode={} total={} drawn={} skipped={} drawCap={} overlayBudget={}",
                        skeleton_bone_debug_math::snapshotSourceName(snapshot.source),
                        skeleton_bone_debug_math::modeName(snapshot.mode),
                        snapshot.bones.size(),
                        frame.skeletonCount,
                        skippedBones,
                        drawCap,
                        frame.skeletonEntries.size());
                }

                if (g_rockConfig.rockDebugLogSkeletonBones) {
                    const int interval = (std::max)(1, g_rockConfig.rockDebugSkeletonBoneLogIntervalFrames);
                    if (++s_directSkeletonBoneLogCounter >= static_cast<std::uint32_t>(interval)) {
                        s_directSkeletonBoneLogCounter = 0;
                        float vrScale = 0.0f;
                        if (auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR")) {
                            vrScale = vrScaleSetting->GetFloat();
                        }
                        float rightHandScale = 0.0f;
                        float leftHandScale = 0.0f;
                        if (frik::api::FRIKApi::inst) {
                            rightHandScale = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right).scale;
                            leftHandScale = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left).scale;
                        }

                        ROCK_LOG_DEBUG(Hand,
                            "Direct skeleton snapshot: source={} mode={} powerArmor={} skeleton={} tree={} total={} drawn={} axes={} skipped={} required={} missing={} vrScale={:.3f} rightHandScale={:.3f} leftHandScale={:.3f}",
                            skeleton_bone_debug_math::snapshotSourceName(snapshot.source),
                            skeleton_bone_debug_math::modeName(snapshot.mode),
                            snapshot.inPowerArmor,
                            reinterpret_cast<std::uintptr_t>(snapshot.skeleton),
                            reinterpret_cast<std::uintptr_t>(snapshot.boneTree),
                            snapshot.totalBoneCount,
                            frame.skeletonCount,
                            axesDrawn,
                            skippedBones,
                            snapshot.requiredResolvedCount,
                            snapshot.missingRequiredBones.size(),
                            vrScale,
                            rightHandScale,
                            leftHandScale);

                        for (const auto& bone : snapshot.bones) {
                            if (!skeletonLogFilterMatches(g_rockConfig.rockDebugSkeletonBoneLogFilter, bone.name)) {
                                continue;
                            }

                            const auto axes = skeleton_bone_debug_math::computeAxisEndpoints(bone.world, 1.0f);
                            ROCK_LOG_DEBUG(Hand,
                                "Direct skeleton bone {} parentTree={} parentDraw={} pos=({:.3f},{:.3f},{:.3f}) scale={:.3f} xAxisEnd=({:.3f},{:.3f},{:.3f}) yAxisEnd=({:.3f},{:.3f},{:.3f}) zAxisEnd=({:.3f},{:.3f},{:.3f})",
                                bone.name,
                                bone.parentTreeIndex,
                                bone.drawableParentSnapshotIndex,
                                bone.world.translate.x,
                                bone.world.translate.y,
                                bone.world.translate.z,
                                bone.world.scale,
                                axes.xEnd.x,
                                axes.xEnd.y,
                                axes.xEnd.z,
                                axes.yEnd.x,
                                axes.yEnd.y,
                                axes.yEnd.z,
                                axes.zEnd.x,
                                axes.zEnd.y,
                                axes.zEnd.z);
                        }
                    }
                } else {
                    s_directSkeletonBoneLogCounter = 0;
                }
            }
        } else {
            s_directSkeletonBoneReader.resetCache();
            s_directSkeletonBoneLogCounter = 0;
        }

        if (drawRootFlattenedFingerSkeleton) {
            auto addRootFlattenedFingerSkeletonDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
                if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot)) {
                    return;
                }

                const auto role = isLeft ? debug::MarkerOverlayRole::LeftRootFlattenedFingerSkeleton : debug::MarkerOverlayRole::RightRootFlattenedFingerSkeleton;
                const float markerSize = g_rockConfig.rockDebugRootFlattenedFingerSkeletonMarkerSize;
                for (const auto& finger : snapshot.fingers) {
                    if (!finger.valid) {
                        continue;
                    }
                    addMarkerPoint(role, finger.points[0], markerSize);
                    addMarkerPoint(role, finger.points[1], markerSize * 0.85f);
                    addMarkerPoint(role, finger.points[2], markerSize);
                    addMarkerLine(role, finger.points[0], finger.points[1]);
                    addMarkerLine(role, finger.points[1], finger.points[2]);
                }
            };

            addRootFlattenedFingerSkeletonDebug(false);
            addRootFlattenedFingerSkeletonDebug(true);
        }

        if (drawGrabPivots || drawGrabContactPatch) {
            auto addGrabPivotDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabPivotDebugSnapshot snapshot{};
                if (!hand.getGrabPivotDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotA : debug::MarkerOverlayRole::RightGrabPivotA, snapshot.handPivotWorld, 3.0f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotB : debug::MarkerOverlayRole::RightGrabPivotB, snapshot.objectPivotWorld, 3.0f);
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotError : debug::MarkerOverlayRole::RightGrabPivotError, snapshot.handPivotWorld,
                    snapshot.objectPivotWorld);
            };

            addGrabPivotDebug(_rightHand);
            addGrabPivotDebug(_leftHand);
        }

        if (drawGrabSurfaceFrame || drawGrabContactPatch) {
            auto addGrabSurfaceDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabSurfaceFrameDebugSnapshot snapshot{};
                if (!hand.getGrabSurfaceFrameDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfacePoint : debug::MarkerOverlayRole::RightGrabSurfacePoint, snapshot.contactPointWorld, 2.4f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceNormal : debug::MarkerOverlayRole::RightGrabSurfaceNormal, snapshot.contactPointWorld,
                    snapshot.normalEndWorld, 1.4f);
                if (snapshot.hasTangent) {
                    addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceTangent : debug::MarkerOverlayRole::RightGrabSurfaceTangent, snapshot.contactPointWorld,
                        snapshot.tangentEndWorld, 1.0f);
                }
                if (snapshot.hasBitangent) {
                    addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftGrabSurfaceBitangent : debug::MarkerOverlayRole::RightGrabSurfaceBitangent, snapshot.contactPointWorld,
                        snapshot.bitangentEndWorld, 1.0f);
                }
            };

            addGrabSurfaceDebug(_rightHand);
            addGrabSurfaceDebug(_leftHand);
        }

        if (drawGrabContactPatch) {
            auto addGrabContactPatchDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabContactPatchDebugSnapshot snapshot{};
                if (!hand.getGrabContactPatchDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                const auto role = isLeft ? debug::MarkerOverlayRole::LeftGrabContactPatchSample : debug::MarkerOverlayRole::RightGrabContactPatchSample;
                for (std::uint32_t i = 0; i < snapshot.sampleCount; ++i) {
                    addMarkerPoint(role, snapshot.samplePointsWorld[i], 1.8f);
                }
            };

            addGrabContactPatchDebug(_rightHand);
            addGrabContactPatchDebug(_leftHand);
        }

        if (drawFingerProbes) {
            auto addFingerProbeDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                std::array<RE::NiPoint3, 5> starts{};
                std::array<RE::NiPoint3, 5> ends{};
                if (!hand.getGrabFingerProbeDebug(starts, ends)) {
                    return;
                }

                const auto role = hand.isLeft() ? debug::MarkerOverlayRole::LeftGrabFingerProbe : debug::MarkerOverlayRole::RightGrabFingerProbe;
                for (std::size_t i = 0; i < starts.size(); ++i) {
                    addMarkerLine(role, starts[i], ends[i]);
                }
            };

            addFingerProbeDebug(_rightHand);
            addFingerProbeDebug(_leftHand);
        }

        if (drawHandBoneContacts) {
            auto addSemanticContactDebug = [&](const Hand& hand) {
                hand_semantic_contact_state::SemanticContactRecord contact{};
                if (!hand.getLastSemanticContact(contact) || contact.framesSinceContact >= 5) {
                    return;
                }

                RE::NiTransform sourceWorld{};
                if (!tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ contact.handBodyId }, sourceWorld)) {
                    return;
                }

                RE::NiTransform targetWorld{};
                const bool hasTarget = tryResolveLiveBodyWorldTransform(hknp, RE::hknpBodyId{ contact.otherBodyId }, targetWorld);
                const auto role = hand.isLeft() ? debug::MarkerOverlayRole::LeftHandBoneContact : debug::MarkerOverlayRole::RightHandBoneContact;
                addMarkerPoint(role, sourceWorld.translate, 2.8f);
                if (hasTarget) {
                    addMarkerLine(role, sourceWorld.translate, targetWorld.translate);
                }
            };

            addSemanticContactDebug(_rightHand);
            addSemanticContactDebug(_leftHand);
        }

        if (drawWeaponAuthorityDebug) {
            TwoHandedGripDebugSnapshot snapshot{};
            if (_twoHandedGrip.getDebugAuthoritySnapshot(snapshot)) {
                addAxisTransform(snapshot.weaponWorld, debug::AxisOverlayRole::WeaponAuthority, snapshot.weaponWorld.translate, false);
                addAxisTransform(snapshot.rightRequestedHandWorld, debug::AxisOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, true);
                addAxisTransform(snapshot.leftRequestedHandWorld, debug::AxisOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, true);
                addMarkerPoint(debug::MarkerOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, 3.0f);
                addMarkerPoint(debug::MarkerOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, 3.0f);

                if (frik::api::FRIKApi::inst) {
                    const RE::NiTransform appliedRight = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
                    const RE::NiTransform appliedLeft = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
                    addAxisTransform(appliedRight, debug::AxisOverlayRole::RightFrikAppliedHand, snapshot.rightRequestedHandWorld.translate, true);
                    addAxisTransform(appliedLeft, debug::AxisOverlayRole::LeftFrikAppliedHand, snapshot.leftRequestedHandWorld.translate, true);
                    addMarkerLine(debug::MarkerOverlayRole::RightWeaponAuthorityMismatch, snapshot.rightRequestedHandWorld.translate, appliedRight.translate);
                    addMarkerLine(debug::MarkerOverlayRole::LeftWeaponAuthorityMismatch, snapshot.leftRequestedHandWorld.translate, appliedLeft.translate);

                    static std::uint32_t authorityMismatchLogCounter = 0;
                    if (++authorityMismatchLogCounter >= 120) {
                        authorityMismatchLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip authority mismatch: right={:.2f}gu left={:.2f}gu",
                            pointDistance(snapshot.rightRequestedHandWorld.translate, appliedRight.translate),
                            pointDistance(snapshot.leftRequestedHandWorld.translate, appliedLeft.translate));
                    }
                }
            }
        }

        if (drawGrabTransformTelemetry) {
            auto publishGrabTelemetry = [&](Hand& hand, bool isLeft) {
                auto& telemetryState = _grabTransformTelemetryStates[isLeft ? 1 : 0];
                if (!hand.isHolding()) {
                    telemetryState.active = false;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                    return;
                }

                if (!telemetryState.active) {
                    telemetryState.active = true;
                    telemetryState.session = _grabTransformTelemetryNextSession++;
                    telemetryState.frame = 0;
                    telemetryState.logFrameCounter = 0;
                }

                ++telemetryState.frame;
                const grab_transform_telemetry::FrameStamp stamp{
                    .session = telemetryState.session,
                    .frame = telemetryState.frame,
                    .tickMs = GetTickCount64(),
                };

                const RE::NiTransform rawHandWorld = isLeft ? context.left.rawHandWorld : context.right.rawHandWorld;

                grab_transform_telemetry::RuntimeSample sample{};
                if (!hand.getGrabTransformTelemetrySnapshot(
                        hknp,
                        rawHandWorld,
                        g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled,
                        sample)) {
                    return;
                }

                if (drawGrabTransformTelemetryAxes) {
                    if (sample.hasGrabStartFrames) {
                        addAxisTransform(sample.currentConstraintDesiredObjectWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabDesiredObject : debug::AxisOverlayRole::RightGrabDesiredObject,
                            sample.rawHandWorld.translate,
                            true);
                        addAxisTransform(sample.heldNodeWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHeldNode : debug::AxisOverlayRole::RightGrabHeldNode,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHeldDesiredError : debug::MarkerOverlayRole::RightGrabHeldDesiredError,
                            sample.currentConstraintDesiredObjectWorld.translate,
                            sample.heldNodeWorld.translate);
                    }
                    if (sample.hasHiggsReverseTarget) {
                        addAxisTransform(sample.higgsReverseTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabHiggsReverse : debug::AxisOverlayRole::RightGrabHiggsReverse,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabHiggsReverseError : debug::MarkerOverlayRole::RightGrabHiggsReverseError,
                            sample.rawHandWorld.translate,
                            sample.higgsReverseTargetWorld.translate);
                    }
                    if (sample.hasConstraintReverseTarget) {
                        addAxisTransform(sample.constraintReverseTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabConstraintReverse : debug::AxisOverlayRole::RightGrabConstraintReverse,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabConstraintReverseError : debug::MarkerOverlayRole::RightGrabConstraintReverseError,
                            sample.handBodyWorld.translate,
                            sample.constraintReverseTargetWorld.translate);
                    }
                    if (sample.hasRockVisualTarget) {
                        addAxisTransform(sample.rockVisualTargetWorld,
                            isLeft ? debug::AxisOverlayRole::LeftGrabRockVisualTarget : debug::AxisOverlayRole::RightGrabRockVisualTarget,
                            sample.rawHandWorld.translate,
                            true);
                        addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabRockVisualError : debug::MarkerOverlayRole::RightGrabRockVisualError,
                            sample.rawHandWorld.translate,
                            sample.rockVisualTargetWorld.translate);
                    }
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotA : debug::MarkerOverlayRole::RightGrabPivotA, sample.pivotAWorld, 3.0f);
                    addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotB : debug::MarkerOverlayRole::RightGrabPivotB, sample.pivotBWorld, 3.0f);
                    addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotError : debug::MarkerOverlayRole::RightGrabPivotError, sample.pivotAWorld, sample.pivotBWorld);
                }

                if (drawGrabTransformTelemetryText) {
                    const float color[4] = { isLeft ? 1.0f : 0.55f, isLeft ? 0.55f : 1.0f, isLeft ? 0.95f : 1.0f, 0.94f };
                    const auto textBasis = grab_transform_telemetry_overlay::buildHandAttachedTextBasis(sample.rawHandWorld, isLeft);
                    const auto labelRole = isLeft ? debug::MarkerOverlayRole::LeftGrabTelemetryLabelAnchor : debug::MarkerOverlayRole::RightGrabTelemetryLabelAnchor;
                    addMarkerPoint(labelRole, textBasis.anchor, 2.6f);
                    addMarkerLine(labelRole, textBasis.hand, textBasis.anchor);

                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 0),
                        color,
                        "GTEL S%u F%llu MS%llu H%s AUTH%d BODY%u HB%s OB%s",
                        stamp.session,
                        static_cast<unsigned long long>(stamp.frame),
                        static_cast<unsigned long long>(stamp.tickMs),
                        grab_transform_telemetry::handLabel(isLeft),
                        sample.visualAuthorityEnabled ? 1 : 0,
                        sample.heldBodyId,
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        body_frame::bodyFrameSourceCode(sample.heldBodySource));
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 1),
                        color,
                        "LIVE_HAND %.1f %.1f %.1f",
                        sample.rawHandWorld.translate.x,
                        sample.rawHandWorld.translate.y,
                        sample.rawHandWorld.translate.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 2),
                        color,
                        "PALM_BODY %s %.1f %.1f %.1f D %.2f %.2f",
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.handBodyWorld.translate.x,
                        sample.handBodyWorld.translate.y,
                        sample.handBodyWorld.translate.z,
                        sample.rawToHandBody.positionGameUnits,
                        sample.rawToHandBody.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 3),
                        color,
                        "VIS_NODE %.1f %.1f %.1f BODYNODE %.1f %.1f %.1f D %.2f %.2f",
                        sample.heldNodeWorld.translate.x,
                        sample.heldNodeWorld.translate.y,
                        sample.heldNodeWorld.translate.z,
                        sample.heldBodyDerivedNodeWorld.translate.x,
                        sample.heldBodyDerivedNodeWorld.translate.y,
                        sample.heldBodyDerivedNodeWorld.translate.z,
                        sample.bodyDerivedNodeToHeldNode.positionGameUnits,
                        sample.bodyDerivedNodeToHeldNode.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 4),
                        color,
                        "PIVERR LIVE %.2f A %.1f %.1f %.1f B %.1f %.1f %.1f",
                        sample.pivotErrorGameUnits,
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 5),
                        color,
                        "CON_DESN %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentConstraintDesiredObjectWorld.translate.x,
                        sample.currentConstraintDesiredObjectWorld.translate.y,
                        sample.currentConstraintDesiredObjectWorld.translate.z,
                        sample.heldNodeToConstraintDesiredObject.positionGameUnits,
                        sample.heldNodeToConstraintDesiredObject.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 6),
                        color,
                        "CON_DESB %.1f %.1f %.1f D %.2f %.2f",
                        sample.currentConstraintDesiredBodyWorld.translate.x,
                        sample.currentConstraintDesiredBodyWorld.translate.y,
                        sample.currentConstraintDesiredBodyWorld.translate.z,
                        sample.heldBodyToConstraintDesiredBody.positionGameUnits,
                        sample.heldBodyToConstraintDesiredBody.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 7),
                        color,
                        "RAWHS_REV %.1f %.1f %.1f D %.2f %.2f",
                        sample.higgsReverseTargetWorld.translate.x,
                        sample.higgsReverseTargetWorld.translate.y,
                        sample.higgsReverseTargetWorld.translate.z,
                        sample.rawToHiggsReverse.positionGameUnits,
                        sample.rawToHiggsReverse.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 8),
                        color,
                        "CONHS_REV %.1f %.1f %.1f D %.2f %.2f C %.2f",
                        sample.constraintReverseTargetWorld.translate.x,
                        sample.constraintReverseTargetWorld.translate.y,
                        sample.constraintReverseTargetWorld.translate.z,
                        sample.handBodyToConstraintReverse.positionGameUnits,
                        sample.handBodyToConstraintReverse.rotationDegrees,
                        sample.higgsToConstraintReverse.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 9),
                        color,
                        "AUTH_REV %.1f %.1f %.1f D %.2f %.2f H %.2f",
                        sample.rockVisualTargetWorld.translate.x,
                        sample.rockVisualTargetWorld.translate.y,
                        sample.rockVisualTargetWorld.translate.z,
                        sample.rawToRockVisual.positionGameUnits,
                        sample.rawToRockVisual.rotationDegrees,
                        sample.higgsToRockVisual.rotationDegrees);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 10),
                        color,
                        "TBLOC %.1f %.1f %.1f DES %.1f %.1f %.1f E %.2f",
                        sample.constraintTransformBLocalGame.x,
                        sample.constraintTransformBLocalGame.y,
                        sample.constraintTransformBLocalGame.z,
                        sample.desiredTransformBLocalGame.x,
                        sample.desiredTransformBLocalGame.y,
                        sample.desiredTransformBLocalGame.z,
                        sample.transformBLocalDelta.distance);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 11),
                        color,
                        "ANGT CI %.2f RI %.2f CF %.2f TB %.2f EN%d",
                        sample.targetColumnsToConstraintInverseDegrees,
                        sample.targetRowsToConstraintInverseDegrees,
                        sample.targetColumnsToConstraintForwardDegrees,
                        sample.targetColumnsToTransformBDegrees,
                        sample.ragdollMotorEnabled ? 1 : 0);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 12),
                        color,
                        "MOTOR AT %.3f AD %.2f AF %.0f LT %.3f LF %.0f M %.2f",
                        sample.angularMotorTau,
                        sample.angularMotorDamping,
                        sample.angularMotorMaxForce,
                        sample.linearMotorTau,
                        sample.linearMotorMaxForce,
                        sample.heldBodyMass);
                    addTextLine(grab_transform_telemetry_overlay::lineAnchor(textBasis, 13),
                        color,
                        "AXIS RAW %.2f %.2f %.2f CON %.2f %.2f %.2f AUTH %.2f %.2f %.2f",
                        sample.rawToHiggsReverseAxes.x,
                        sample.rawToHiggsReverseAxes.y,
                        sample.rawToHiggsReverseAxes.z,
                        sample.rawToConstraintReverseAxes.x,
                        sample.rawToConstraintReverseAxes.y,
                        sample.rawToConstraintReverseAxes.z,
                        sample.rawToRockVisualAxes.x,
                        sample.rawToRockVisualAxes.y,
                        sample.rawToRockVisualAxes.z);
                }

                ++telemetryState.logFrameCounter;
                const std::uint64_t logInterval =
                    static_cast<std::uint64_t>((std::max)(1, g_rockConfig.rockDebugGrabTransformTelemetryLogIntervalFrames));
                if (telemetryState.frame == 1 || telemetryState.logFrameCounter >= logInterval) {
                    telemetryState.logFrameCounter = 0;
                    const auto prefix = grab_transform_telemetry::formatStampPrefix(stamp, isLeft, telemetryState.frame == 1 ? "start" : "held");
                    const char* phaseLabel = telemetryState.frame == 1 ? "START" : "HELD";
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} formID={:08X} heldBody={} handBody={} heldSource={} handSource={} heldMotion={} handMotion={} auth={} surfTarget={} pivotFrame=LIVE pivotA=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) pivotErr={:.3f}",
                        prefix,
                        phaseLabel,
                        sample.heldFormId,
                        sample.heldBodyId,
                        sample.handBodyId,
                        body_frame::bodyFrameSourceCode(sample.heldBodySource),
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.heldMotionIndex,
                        sample.handMotionIndex,
                        sample.visualAuthorityEnabled ? "on" : "off",
                        sample.rockVisualTargetUsedSurfaceFrame ? "surface" : "reverse",
                        sample.pivotAWorld.x,
                        sample.pivotAWorld.y,
                        sample.pivotAWorld.z,
                        sample.pivotBWorld.x,
                        sample.pivotBWorld.y,
                        sample.pivotBWorld.z,
                        sample.pivotErrorGameUnits);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} liveHand=({:.2f},{:.2f},{:.2f}) handBody[{}]=({:.2f},{:.2f},{:.2f}) heldBody[{}]=({:.2f},{:.2f},{:.2f}) visualNode=({:.2f},{:.2f},{:.2f}) bodyDerivedNode=({:.2f},{:.2f},{:.2f}) bodyNodeToVisual={:.3f}gu/{:.3f}deg liveToHandBody={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.rawHandWorld.translate.x,
                        sample.rawHandWorld.translate.y,
                        sample.rawHandWorld.translate.z,
                        body_frame::bodyFrameSourceCode(sample.handBodySource),
                        sample.handBodyWorld.translate.x,
                        sample.handBodyWorld.translate.y,
                        sample.handBodyWorld.translate.z,
                        body_frame::bodyFrameSourceCode(sample.heldBodySource),
                        sample.heldBodyWorld.translate.x,
                        sample.heldBodyWorld.translate.y,
                        sample.heldBodyWorld.translate.z,
                        sample.heldNodeWorld.translate.x,
                        sample.heldNodeWorld.translate.y,
                        sample.heldNodeWorld.translate.z,
                        sample.heldBodyDerivedNodeWorld.translate.x,
                        sample.heldBodyDerivedNodeWorld.translate.y,
                        sample.heldBodyDerivedNodeWorld.translate.z,
                        sample.bodyDerivedNodeToHeldNode.positionGameUnits,
                        sample.bodyDerivedNodeToHeldNode.rotationDegrees,
                        sample.rawToHandBody.positionGameUnits,
                        sample.rawToHandBody.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} startLive=({:.2f},{:.2f},{:.2f}) startHandBody=({:.2f},{:.2f},{:.2f}) objectAtGrab=({:.2f},{:.2f},{:.2f}) desiredAtGrab=({:.2f},{:.2f},{:.2f}) startDrift={:.3f}gu/{:.3f}deg rawHS=({:.2f},{:.2f},{:.2f}) conHS=({:.2f},{:.2f},{:.2f}) bodyRaw=({:.2f},{:.2f},{:.2f}) bodyLocal=({:.2f},{:.2f},{:.2f})",
                        prefix,
                        phaseLabel,
                        sample.liveHandWorldAtGrab.translate.x,
                        sample.liveHandWorldAtGrab.translate.y,
                        sample.liveHandWorldAtGrab.translate.z,
                        sample.handBodyWorldAtGrab.translate.x,
                        sample.handBodyWorldAtGrab.translate.y,
                        sample.handBodyWorldAtGrab.translate.z,
                        sample.objectNodeWorldAtGrab.translate.x,
                        sample.objectNodeWorldAtGrab.translate.y,
                        sample.objectNodeWorldAtGrab.translate.z,
                        sample.desiredObjectWorldAtGrab.translate.x,
                        sample.desiredObjectWorldAtGrab.translate.y,
                        sample.desiredObjectWorldAtGrab.translate.z,
                        sample.heldNodeToDesiredObjectAtGrab.positionGameUnits,
                        sample.heldNodeToDesiredObjectAtGrab.rotationDegrees,
                        sample.rawHandSpace.translate.x,
                        sample.rawHandSpace.translate.y,
                        sample.rawHandSpace.translate.z,
                        sample.constraintHandSpace.translate.x,
                        sample.constraintHandSpace.translate.y,
                        sample.constraintHandSpace.translate.z,
                        sample.handBodyToRawHandAtGrab.translate.x,
                        sample.handBodyToRawHandAtGrab.translate.y,
                        sample.handBodyToRawHandAtGrab.translate.z,
                        sample.bodyLocal.translate.x,
                        sample.bodyLocal.translate.y,
                        sample.bodyLocal.translate.z);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawDesired=({:.2f},{:.2f},{:.2f}) heldToRawDesired={:.3f}gu/{:.3f}deg conDesiredNode=({:.2f},{:.2f},{:.2f}) heldToConDesired={:.3f}gu/{:.3f}deg conDesiredBody=({:.2f},{:.2f},{:.2f}) bodyToConDesired={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.currentRawDesiredObjectWorld.translate.x,
                        sample.currentRawDesiredObjectWorld.translate.y,
                        sample.currentRawDesiredObjectWorld.translate.z,
                        sample.heldNodeToRawDesiredObject.positionGameUnits,
                        sample.heldNodeToRawDesiredObject.rotationDegrees,
                        sample.currentConstraintDesiredObjectWorld.translate.x,
                        sample.currentConstraintDesiredObjectWorld.translate.y,
                        sample.currentConstraintDesiredObjectWorld.translate.z,
                        sample.heldNodeToConstraintDesiredObject.positionGameUnits,
                        sample.heldNodeToConstraintDesiredObject.rotationDegrees,
                        sample.currentConstraintDesiredBodyWorld.translate.x,
                        sample.currentConstraintDesiredBodyWorld.translate.y,
                        sample.currentConstraintDesiredBodyWorld.translate.z,
                        sample.heldBodyToConstraintDesiredBody.positionGameUnits,
                        sample.heldBodyToConstraintDesiredBody.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} rawHSRev=({:.2f},{:.2f},{:.2f}) liveToRawHSRev={:.3f}gu/{:.3f}deg conHSRev=({:.2f},{:.2f},{:.2f}) handBodyToConHSRev={:.3f}gu/{:.3f}deg rawToConRev={:.3f}gu/{:.3f}deg authRev=({:.2f},{:.2f},{:.2f}) liveToAuthRev={:.3f}gu/{:.3f}deg rawRevToAuth={:.3f}gu/{:.3f}deg",
                        prefix,
                        phaseLabel,
                        sample.higgsReverseTargetWorld.translate.x,
                        sample.higgsReverseTargetWorld.translate.y,
                        sample.higgsReverseTargetWorld.translate.z,
                        sample.rawToHiggsReverse.positionGameUnits,
                        sample.rawToHiggsReverse.rotationDegrees,
                        sample.constraintReverseTargetWorld.translate.x,
                        sample.constraintReverseTargetWorld.translate.y,
                        sample.constraintReverseTargetWorld.translate.z,
                        sample.handBodyToConstraintReverse.positionGameUnits,
                        sample.handBodyToConstraintReverse.rotationDegrees,
                        sample.higgsToConstraintReverse.positionGameUnits,
                        sample.higgsToConstraintReverse.rotationDegrees,
                        sample.rockVisualTargetWorld.translate.x,
                        sample.rockVisualTargetWorld.translate.y,
                        sample.rockVisualTargetWorld.translate.z,
                        sample.rawToRockVisual.positionGameUnits,
                        sample.rawToRockVisual.rotationDegrees,
                        sample.higgsToRockVisual.positionGameUnits,
                        sample.higgsToRockVisual.rotationDegrees);
                    ROCK_LOG_INFO(Hand,
                        "GRAB TELEMETRY {} {} axisDots rawHSRev=({:.3f},{:.3f},{:.3f}) conHSRev=({:.3f},{:.3f},{:.3f}) authRev=({:.3f},{:.3f},{:.3f})",
                        prefix,
                        phaseLabel,
                        sample.rawToHiggsReverseAxes.x,
                        sample.rawToHiggsReverseAxes.y,
                        sample.rawToHiggsReverseAxes.z,
                        sample.rawToConstraintReverseAxes.x,
                        sample.rawToConstraintReverseAxes.y,
                        sample.rawToConstraintReverseAxes.z,
                        sample.rawToRockVisualAxes.x,
                        sample.rawToRockVisualAxes.y,
                        sample.rawToRockVisualAxes.z);
                    if (sample.hasConstraintAngularTelemetry) {
                        ROCK_LOG_INFO(Hand,
                            "GRAB TELEMETRY {} {} transformBLocal=({:.2f},{:.2f},{:.2f}) desiredTransformBLocal=({:.2f},{:.2f},{:.2f}) transformBErr={:.3f}gu targetErr(colsInv={:.3f}deg rowsInv={:.3f}deg colsForward={:.3f}deg colsTransformB={:.3f}deg) ragEnabled={} angTau={:.3f} angDamping={:.3f} angForce={:.1f} linTau={:.3f} linForce={:.1f} mass={:.3f}",
                            prefix,
                            phaseLabel,
                            sample.constraintTransformBLocalGame.x,
                            sample.constraintTransformBLocalGame.y,
                            sample.constraintTransformBLocalGame.z,
                            sample.desiredTransformBLocalGame.x,
                            sample.desiredTransformBLocalGame.y,
                            sample.desiredTransformBLocalGame.z,
                            sample.transformBLocalDelta.distance,
                            sample.targetColumnsToConstraintInverseDegrees,
                            sample.targetRowsToConstraintInverseDegrees,
                            sample.targetColumnsToConstraintForwardDegrees,
                            sample.targetColumnsToTransformBDegrees,
                            sample.ragdollMotorEnabled ? "yes" : "no",
                            sample.angularMotorTau,
                            sample.angularMotorDamping,
                            sample.angularMotorMaxForce,
                            sample.linearMotorTau,
                            sample.linearMotorMaxForce,
                            sample.heldBodyMass);
                    }
                }
            };

            publishGrabTelemetry(_rightHand, false);
            publishGrabTelemetry(_leftHand, true);
        } else {
            for (auto& state : _grabTransformTelemetryStates) {
                state.active = false;
                state.frame = 0;
                state.logFrameCounter = 0;
            }
        }

        if (frame.drawRockBodies) {
            if (debug_overlay_policy::shouldDrawHandBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawHandColliders) &&
                !g_rockConfig.rockDebugDrawHandBoneColliders) {
                addBody(_rightHand.getCollisionBodyId(), debug::BodyOverlayRole::RightHand);
                addBody(_leftHand.getCollisionBodyId(), debug::BodyOverlayRole::LeftHand);
            }

            if (debug_overlay_policy::shouldDrawHandBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawHandBoneColliders)) {
                const std::uint32_t cap = static_cast<std::uint32_t>((std::clamp)(g_rockConfig.rockDebugMaxHandBoneBodiesDrawn, 0, 48));
                std::uint32_t drawn = 0;
                auto addHandBoneBodies = [&](const Hand& hand, debug::BodyOverlayRole anchorRole, debug::BodyOverlayRole segmentRole) {
                    const std::uint32_t count = hand.getHandColliderBodyCount();
                    for (std::uint32_t i = 0; i < count && drawn < cap; ++i) {
                        const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                        if (bodyId == INVALID_BODY_ID) {
                            continue;
                        }
                        addBody(RE::hknpBodyId{ bodyId }, i == 0 ? anchorRole : segmentRole);
                        ++drawn;
                    }
                };

                addHandBoneBodies(_rightHand, debug::BodyOverlayRole::RightHand, debug::BodyOverlayRole::RightHandSegment);
                addHandBoneBodies(_leftHand, debug::BodyOverlayRole::LeftHand, debug::BodyOverlayRole::LeftHandSegment);
            }

            const auto weaponCount = _weaponCollision.getWeaponBodyCount();
            for (std::uint32_t i = 0; i < weaponCount; i++) {
                const bool drawNormalWeaponBody =
                    debug_overlay_policy::shouldDrawWeaponBody(drawRockColliderBodies, g_rockConfig.rockDebugDrawWeaponColliders, i, g_rockConfig.rockDebugMaxWeaponBodiesDrawn);
                if (drawNormalWeaponBody) {
                    addBody(RE::hknpBodyId{ _weaponCollision.getWeaponBodyIdAtomic(i) }, debug::BodyOverlayRole::Weapon);
                }
            }
        }

        if (frame.drawTargetBodies || drawWorldOriginDiagnostics) {
            auto addHandTarget = [&](const Hand& hand) {
                const auto& handInput = hand.isLeft() ? context.left : context.right;
                if (hand.isHolding()) {
                    const auto& savedState = hand.getSavedObjectState();
                    if (frame.drawTargetBodies) {
                        addBody(savedState.bodyId, debug::BodyOverlayRole::Target);
                        addAxisBody(savedState.bodyId, debug::AxisOverlayRole::TargetBody, handInput.rawHandWorld.translate, true);
                    }
                    addWorldOriginDiagnostic(hand, true, savedState.bodyId, savedState.refr, nullptr, nullptr);
                    return;
                }
                if (hand.hasSelection()) {
                    const auto& selection = hand.getSelection();
                    if (frame.drawTargetBodies) {
                        addBody(selection.bodyId, debug::BodyOverlayRole::Target);
                        addAxisBody(selection.bodyId, debug::AxisOverlayRole::TargetBody, handInput.rawHandWorld.translate, true);
                    }
                    addWorldOriginDiagnostic(hand, false, selection.bodyId, selection.refr, selection.hitNode, selection.visualNode);
                }
            };

            addHandTarget(_rightHand);
            addHandTarget(_leftHand);
        }

        debug::PublishFrame(frame);
    }

    void PhysicsInteraction::updateSelection(const PhysicsFrameContext& frame)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady())
            return;

        auto exclusiveRefForOtherHand = [](const Hand& hand) -> RE::TESObjectREFR* {
            if (!hand.hasSelection() || !selection_state_policy::hasExclusiveObjectSelection(hand.getState())) {
                return nullptr;
            }
            return hand.getSelection().refr;
        };

        auto* rightExclusiveRef = exclusiveRefForOtherHand(_rightHand);
        auto* leftExclusiveRef = exclusiveRefForOtherHand(_leftHand);

        if (!frame.right.disabled) {
            _rightHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.right.grabAnchorWorld,
                frame.right.palmNormalWorld,
                frame.right.pointingWorld,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                leftExclusiveRef);
        }

        if (!frame.left.disabled) {
            _leftHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.left.grabAnchorWorld,
                frame.left.palmNormalWorld,
                frame.left.pointingWorld,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                rightExclusiveRef);
        }
    }

    HeldObjectPlayerSpaceFrame PhysicsInteraction::sampleHeldObjectPlayerSpaceFrame(float deltaSeconds)
    {
        HeldObjectPlayerSpaceFrame frame{};
        auto* api = frik::api::FRIKApi::inst;
        if (!api) {
            _hasHeldPlayerSpacePosition = false;
            _hasHeldPlayerSpaceTransform = false;
            return frame;
        }

        const RE::NiPoint3 smoothPos = api->getSmoothedPlayerPosition();
        RE::NiTransform playerSpaceWorld = transform_math::makeIdentityTransform<RE::NiTransform>();
        playerSpaceWorld.translate = smoothPos;
        const char* playerSpaceSource = "frikSmoothedPosition";
        if (RE::PlayerCharacter::GetSingleton()) {
            if (auto* playerNodes = f4vr::getPlayerNodes(); playerNodes && playerNodes->roomnode) {
                playerSpaceWorld = playerNodes->roomnode->world;
                playerSpaceWorld.translate = smoothPos;
                playerSpaceSource = "roomNodeRotation+frikSmoothedPosition";
            }
        }

        if (!g_rockConfig.rockGrabPlayerSpaceCompensation) {
            _prevHeldPlayerSpacePosition = smoothPos;
            _prevHeldPlayerSpaceTransform = playerSpaceWorld;
            _hasHeldPlayerSpacePosition = true;
            _hasHeldPlayerSpaceTransform = true;
            return frame;
        }

        frame.enabled = true;
        frame.currentPlayerSpaceWorld = playerSpaceWorld;
        frame.source = playerSpaceSource;
        if (_hasHeldPlayerSpacePosition) {
            frame.deltaGameUnits = smoothPos - _prevHeldPlayerSpacePosition;
            frame.velocityHavok = held_object_physics_math::gameUnitsDeltaToHavokVelocity(frame.deltaGameUnits, deltaSeconds, physics_scale::havokToGame());
            frame.warpByDistance = held_object_physics_math::shouldWarpPlayerSpaceDelta(frame.deltaGameUnits, g_rockConfig.rockGrabPlayerSpaceWarpDistance);
        }
        if (_hasHeldPlayerSpaceTransform) {
            frame.previousPlayerSpaceWorld = _prevHeldPlayerSpaceTransform;
            frame.hasWarpTransforms = true;
            frame.rotationDeltaDegrees =
                held_player_space_math::rotationDeltaDegrees(_prevHeldPlayerSpaceTransform.rotate, playerSpaceWorld.rotate);
            frame.warpByRotation = held_player_space_math::shouldWarpPlayerSpaceRotation(
                _prevHeldPlayerSpaceTransform.rotate,
                playerSpaceWorld.rotate,
                g_rockConfig.rockGrabPlayerSpaceWarpMinRotationDegrees);
        }
        frame.warp = frame.warpByDistance || frame.warpByRotation;

        _prevHeldPlayerSpacePosition = smoothPos;
        _prevHeldPlayerSpaceTransform = playerSpaceWorld;
        _hasHeldPlayerSpacePosition = true;
        _hasHeldPlayerSpaceTransform = true;

        if (g_rockConfig.rockDebugGrabFrameLogging && (_rightHand.isHolding() || _leftHand.isHolding())) {
            ++_heldPlayerSpaceLogCounter;
            if (_heldPlayerSpaceLogCounter >= 45 || frame.warp) {
                _heldPlayerSpaceLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held player-space: source={} enabled={} beforeHeld=yes warp={} distWarp={} rotWarp={} rotDelta={:.2f}deg "
                    "delta=({:.2f},{:.2f},{:.2f}) velHk=({:.3f},{:.3f},{:.3f})",
                    frame.source, frame.enabled ? "yes" : "no", frame.warp ? "yes" : "no", frame.warpByDistance ? "yes" : "no",
                    frame.warpByRotation ? "yes" : "no", frame.rotationDeltaDegrees, frame.deltaGameUnits.x, frame.deltaGameUnits.y, frame.deltaGameUnits.z,
                    frame.velocityHavok.x, frame.velocityHavok.y, frame.velocityHavok.z);
            }
        }

        return frame;
    }

    void PhysicsInteraction::applyHeldPlayerSpaceVelocity(RE::hknpWorld* hknp)
    {
        /*
         * Held object motion has one velocity authority while the grab is active:
         * the grab constraint targets plus this single player-space compensation
         * pass. Per-hand held loops only sample local velocity for throw history.
         * That mirrors the HIGGS dynamic-grab ownership model while avoiding two
         * hands or connected bodies writing the same Havok motion more than once.
         */
        if (!hknp) {
            _lastCentralHeldPlayerSpaceVelocityHavok = {};
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(_rightHand.getHeldBodyIds().size() + _leftHand.getHeldBodyIds().size() + 2);
        auto appendHandBodies = [&](const Hand& hand) {
            if (!hand.isHolding()) {
                return;
            }

            const auto& savedState = hand.getSavedObjectState();
            if (savedState.bodyId.value != INVALID_BODY_ID) {
                bodyIds.push_back(savedState.bodyId.value);
            }
            for (const auto bodyId : hand.getHeldBodyIds()) {
                if (bodyId != INVALID_BODY_ID) {
                    bodyIds.push_back(bodyId);
                }
            }
        };

        appendHandBodies(_rightHand);
        appendHandBodies(_leftHand);

        const float keep = g_rockConfig.rockGrabResidualVelocityDamping ?
                               held_object_damping_math::velocityKeepFactor(g_rockConfig.rockGrabVelocityDamping) :
                               1.0f;
        const bool runtimeTransformWarp = held_player_space_math::shouldApplyRuntimeTransformWarp(
            g_rockConfig.rockGrabPlayerSpaceTransformWarpEnabled,
            _heldObjectPlayerSpaceFrame.warp,
            _heldObjectPlayerSpaceFrame.hasWarpTransforms);

        const auto result = held_player_space_registry::applyCentralPlayerSpaceVelocity(
            hknp,
            bodyIds,
            _heldObjectPlayerSpaceFrame.velocityHavok,
            _lastCentralHeldPlayerSpaceVelocityHavok,
            keep,
            _heldObjectPlayerSpaceFrame.enabled,
            runtimeTransformWarp,
            runtimeTransformWarp ? &_heldObjectPlayerSpaceFrame.previousPlayerSpaceWorld : nullptr,
            runtimeTransformWarp ? &_heldObjectPlayerSpaceFrame.currentPlayerSpaceWorld : nullptr);

        if (held_player_space_registry::shouldCarryPreviousPlayerVelocity(
                _heldObjectPlayerSpaceFrame.enabled,
                runtimeTransformWarp,
                result.motionsWritten)) {
            _lastCentralHeldPlayerSpaceVelocityHavok = _heldObjectPlayerSpaceFrame.velocityHavok;
        } else {
            _lastCentralHeldPlayerSpaceVelocityHavok = {};
        }

        if (g_rockConfig.rockDebugGrabFrameLogging && !bodyIds.empty()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "Held player-space central writer: beforeHeld=yes diagWarp={} runtimeWarp={} distWarp={} rotWarp={} bodies={} registered={} motionsWritten={} transformsWarped={} duplicateMotions={} writerMask=0x{:02X}",
                _heldObjectPlayerSpaceFrame.warp ? "yes" : "no",
                runtimeTransformWarp ? "yes" : "no",
                _heldObjectPlayerSpaceFrame.warpByDistance ? "yes" : "no",
                _heldObjectPlayerSpaceFrame.warpByRotation ? "yes" : "no",
                bodyIds.size(),
                result.registeredBodies,
                result.motionsWritten,
                result.transformsWarped,
                result.duplicateMotionSkips,
                result.writerMask);
        }
    }

    void PhysicsInteraction::updateGrabInput(const PhysicsFrameContext& frame)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady())
            return;

        auto* hknp = frame.hknpWorld;
        int grabButton = g_rockConfig.rockGrabButtonID;
        const bool rightHandWeaponEquipped = resolveEquippedWeaponInteractionNode() != nullptr;
        const bool equippedWeaponSupportGripActive = _twoHandedGrip.isGripping();

        auto releaseSuppressedHeldObject = [&](Hand& hand, bool isLeft, const char* reason) {
            auto* heldRef = hand.getHeldRef();
            auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
            hand.releaseGrabbedObject(hknp);
            if (heldRef) {
                releaseObject(heldRef);
            }
            ROCK_LOG_DEBUG(Hand, "{} hand: released held object because normal grab input is suppressed ({})", hand.handName(), reason ? reason : "unknown");
            dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
        };

        auto processHand = [&](Hand& hand, bool isLeft) {
            const auto& handInput = isLeft ? frame.left : frame.right;
            if (handInput.disabled)
                return;
            if (!weapon_two_handed_grip_math::canProcessNormalGrabInput(isLeft, equippedWeaponSupportGripActive, rightHandWeaponEquipped)) {
                if (hand.isHolding()) {
                    releaseSuppressedHeldObject(hand, isLeft, isLeft ? "equipped weapon support grip active" : "right-hand weapon equipped");
                } else if (hand.getState() == HandState::Pulled || hand.getState() == HandState::SelectionLocked) {
                    auto* selectedRef = hand.getSelection().refr;
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef);
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull/locked selection because normal grab input is suppressed", hand.handName());
                }
                return;
            }

            const auto grabInput = readGrabButtonState(isLeft, grabButton);
            auto selectedObjectInteractionBlocked = [&]() {
                const auto& sel = hand.getSelection();
                auto* selRef = sel.refr;
                if (!selRef) {
                    return false;
                }

                auto* baseObj = selRef->GetObjectReference();
                if (!baseObj) {
                    return false;
                }

                const char* typeStr = baseObj->GetFormTypeString();
                const std::string_view formType = typeStr ? std::string_view(typeStr) : std::string_view{};

                bool hasMotionProps = false;
                std::uint16_t motionProps = 0;
                if (formType == "ACTI" && sel.bodyId.value != 0x7FFF'FFFF && hknp) {
                    if (auto* motion = havok_runtime::getBodyMotion(hknp, sel.bodyId)) {
                        motionProps = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(motion) + offsets::kMotion_PropertiesId);
                        hasMotionProps = true;
                    }
                }

                const bool isLiveNpc = formType == "NPC_" && !selRef->IsDead(false);
                const bool blocked = grab_interaction_policy::shouldBlockSelectedObjectInteraction(formType, isLiveNpc, hasMotionProps, motionProps);
                if (blocked) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: selected object interaction blocked formType={} formID={:08X} far={} motionProps={} hasMotionProps={}",
                        hand.handName(),
                        formType.empty() ? "???" : typeStr,
                        selRef->GetFormID(),
                        sel.isFarSelection ? "yes" : "no",
                        motionProps,
                        hasMotionProps ? "yes" : "no");
                }
                return blocked;
            };

            if (hand.isHolding()) {
                if (grabInput.released) {
                    auto* heldRef = hand.getHeldRef();
                    auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    hand.releaseGrabbedObject(hknp);
                    if (heldRef)
                        releaseObject(heldRef);
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                } else {
                    const auto& transform = handInput.rawHandWorld;
                    hand.updateHeldObject(hknp, transform, _heldObjectPlayerSpaceFrame, frame.deltaSeconds, g_rockConfig.rockGrabForceFadeInTime, g_rockConfig.rockGrabTauMin);
                }
            } else if (selection_state_policy::canProcessSelectedState(hand.getState()) && hand.hasSelection()) {
                if (grabInput.pressed) {
                    if (!grab_interaction_policy::canAttemptSelectedObjectGrab(
                            hand.getSelection().isFarSelection, hand.getSelection().distance, g_rockConfig.rockFarDetectionRange)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand: far grab blocked (dist={:.1f}, configuredFarRange={:.1f})",
                            hand.handName(),
                            hand.getSelection().distance,
                            g_rockConfig.rockFarDetectionRange);
                        return;
                    }

                    if (selectedObjectInteractionBlocked()) {
                        return;
                    }

                    if (hand.getSelection().isFarSelection) {
                        auto* selectedRef = hand.getSelection().refr;
                        const auto& transform = handInput.rawHandWorld;
                        if (hand.lockFarSelection() && hand.startDynamicPull(hknp, transform)) {
                            claimObject(selectedRef);
                        } else {
                            releaseObject(selectedRef);
                        }
                        return;
                    }

                    const auto& transform = handInput.rawHandWorld;

                    bool grabbed = hand.grabSelectedObject(hknp, transform, g_rockConfig.rockGrabLinearTau, g_rockConfig.rockGrabLinearDamping,
                        g_rockConfig.rockGrabConstraintMaxForce, g_rockConfig.rockGrabLinearProportionalRecovery, g_rockConfig.rockGrabLinearConstantRecovery);

                    if (grabbed) {
                        auto* heldRef = hand.getHeldRef();
                        claimObject(heldRef);
                        dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                    }
                }
            } else if (hand.getState() == HandState::SelectionLocked) {
                if (grabInput.released) {
                    auto* selectedRef = hand.getSelection().refr;
                    ROCK_LOG_DEBUG(Hand, "{} hand released locked far selection", hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef);
                }
            } else if (hand.getState() == HandState::Pulled) {
                auto* pulledRef = hand.getSelection().refr;
                if (grabInput.released) {
                    ROCK_LOG_DEBUG(Hand, "{} hand released dynamic pull", hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(pulledRef);
                    return;
                }

                const auto& transform = handInput.rawHandWorld;
                const bool readyToGrab = hand.updateDynamicPull(hknp, transform, frame.deltaSeconds);
                if (!hand.hasSelection() || hand.getState() == HandState::Idle) {
                    releaseObject(pulledRef);
                    return;
                }

                if (readyToGrab) {
                    bool grabbed = hand.grabSelectedObject(hknp, transform, g_rockConfig.rockGrabLinearTau, g_rockConfig.rockGrabLinearDamping,
                        g_rockConfig.rockGrabConstraintMaxForce, g_rockConfig.rockGrabLinearProportionalRecovery, g_rockConfig.rockGrabLinearConstantRecovery);

                    if (grabbed) {
                        auto* heldRef = hand.getHeldRef();
                        claimObject(heldRef);
                        dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                    } else {
                        hand.clearSelectionState(true);
                        releaseObject(pulledRef);
                    }
                }
            }
        };

        processHand(_rightHand, false);
        processHand(_leftHand, true);
    }

    void PhysicsInteraction::resolveContacts(const PhysicsFrameContext& frame)
    {
        auto* bhk = frame.bhkWorld;
        auto* hknp = frame.hknpWorld;
        auto rightContactBody = _lastContactBodyRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto rightSourceBody = _lastContactSourceRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (rightContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Right", bhk, hknp, RE::hknpBodyId{ rightContactBody });
            if (rightSourceBody == 0xFFFFFFFF) {
                rightSourceBody = _rightHand.getCollisionBodyId().value;
            }
            applyDynamicPushAssist("Right", bhk, hknp, rightSourceBody, rightContactBody, false, &_rightHand);
        }

        auto leftContactBody = _lastContactBodyLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto leftSourceBody = _lastContactSourceLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (leftContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Left", bhk, hknp, RE::hknpBodyId{ leftContactBody });
            if (leftSourceBody == 0xFFFFFFFF) {
                leftSourceBody = _leftHand.getCollisionBodyId().value;
            }
            applyDynamicPushAssist("Left", bhk, hknp, leftSourceBody, leftContactBody, false, &_leftHand);
        }

        auto weaponContactBody = _lastContactBodyWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto weaponSourceBody = _lastContactSourceWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (weaponContactBody != 0xFFFFFFFF && weaponSourceBody != 0xFFFFFFFF) {
            applyDynamicPushAssist("Weapon", bhk, hknp, weaponSourceBody, weaponContactBody, true);
        }
    }

    void PhysicsInteraction::applyDynamicPushAssist(const char* sourceName,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp,
        std::uint32_t sourceBodyId,
            std::uint32_t targetBodyId,
        bool sourceIsWeapon,
        const Hand* sourceHand)
    {
        if (!bhk || !hknp || sourceBodyId == 0xFFFFFFFF || targetBodyId == 0xFFFFFFFF ||
            sourceBodyId == object_physics_body_set::INVALID_BODY_ID || targetBodyId == object_physics_body_set::INVALID_BODY_ID || sourceBodyId == targetBodyId) {
            return;
        }
        if (::rock::provider::isExternalBodyDynamicPushSuppressed(targetBodyId)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} is registered as external suppressing ROCK dynamic push",
                sourceName,
                targetBodyId);
            return;
        }
        if (held_object_body_set_policy::containsAnyBody(_rightHand.getHeldBodyIds(), _leftHand.getHeldBodyIds(), targetBodyId)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} is owned by an active held grab",
                sourceName,
                targetBodyId);
            return;
        }

        auto* targetRef = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ targetBodyId });
        if (!targetRef || targetRef->IsDeleted() || targetRef->IsDisabled()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds, "{} dynamic push skipped: target body {} has no valid ref", sourceName, targetBodyId);
            return;
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
        scanOptions.rightHandBodyId = _rightHand.getCollisionBodyId().value;
        scanOptions.leftHandBodyId = _leftHand.getCollisionBodyId().value;
        scanOptions.sourceBodyId = sourceBodyId;
        scanOptions.sourceWeaponBodyId = sourceIsWeapon ? sourceBodyId : object_physics_body_set::INVALID_BODY_ID;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;
        if (!sourceIsWeapon && sourceHand) {
            scanOptions.heldBySameHand = &sourceHand->getHeldBodyIds();
        }

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhk, hknp, targetRef, scanOptions);
        const auto* targetRecord = bodySet.findRecord(targetBodyId);
        if (!targetRecord) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} not found in ref tree formID={:08X} visitedNodes={} collisionObjects={}",
                sourceName,
                targetBodyId,
                targetRef->GetFormID(),
                bodySet.diagnostics.visitedNodes,
                bodySet.diagnostics.collisionObjects);
            return;
        }
        if (!targetRecord->accepted) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} rejected reason={} layer={} motionId={} motionType={}",
                sourceName,
                targetBodyId,
                physics_body_classifier::rejectReasonName(targetRecord->rejectReason),
                targetRecord->collisionLayer,
                targetRecord->motionId,
                static_cast<int>(targetRecord->motionType));
            return;
        }
        if (!sourceIsWeapon && collision_layer_policy::isActorOrBipedLayer(targetRecord->collisionLayer)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push actor/ragdoll accepted: sourceBody={} targetBody={} layer={} motionId={} motionType={}",
                sourceName,
                sourceBodyId,
                targetBodyId,
                targetRecord->collisionLayer,
                targetRecord->motionId,
                static_cast<int>(targetRecord->motionType));
        }

        const auto uniqueMotionRecords = bodySet.uniqueAcceptedMotionRecords();
        if (uniqueMotionRecords.empty()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: accepted target body {} produced no unique motion bodies",
                sourceName,
                targetBodyId);
            return;
        }

        auto* sourceMotion = havok_runtime::getBodyMotion(hknp, RE::hknpBodyId{ sourceBodyId });
        if (!sourceMotion) {
            return;
        }

        const RE::NiPoint3 sourceVelocityHavok{ sourceMotion->linearVelocity.x, sourceMotion->linearVelocity.y, sourceMotion->linearVelocity.z };
        const std::uint64_t cooldownKey = (static_cast<std::uint64_t>(sourceBodyId) << 32) | targetBodyId;
        float cooldownRemaining = 0.0f;
        if (const auto it = _dynamicPushCooldownUntil.find(cooldownKey); it != _dynamicPushCooldownUntil.end() && it->second > _dynamicPushElapsedSeconds) {
            cooldownRemaining = it->second - _dynamicPushElapsedSeconds;
        }

        const push_assist::PushAssistInput<RE::NiPoint3> pushInput{
            .enabled = g_rockConfig.rockDynamicPushAssistEnabled,
            .sourceVelocity = sourceVelocityHavok,
            .minSpeed = g_rockConfig.rockDynamicPushMinSpeed,
            .maxImpulse = g_rockConfig.rockDynamicPushMaxImpulse,
            .layerMultiplier = 1.0f,
            .cooldownRemainingSeconds = cooldownRemaining,
        };
        const auto push = push_assist::computePushImpulse(pushInput);
        if (!push.apply) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: reason={} speed=({:.3f},{:.3f},{:.3f}) targetBody={} layer={} acceptedBodies={}",
                sourceName,
                pushAssistSkipReasonName(push.skipReason),
                sourceVelocityHavok.x,
                sourceVelocityHavok.y,
                sourceVelocityHavok.z,
                targetBodyId,
                targetRecord->collisionLayer,
                bodySet.acceptedCount());
            return;
        }

        std::uint32_t appliedCount = 0;
        for (const auto* record : uniqueMotionRecords) {
            if (!record) {
                continue;
            }
            physics_recursive_wrappers::activateBody(hknp, record->bodyId);
            if (push_assist::applyLinearImpulse(record->collisionObject, push.impulse)) {
                ++appliedCount;
            }
        }

        if (appliedCount > 0) {
            _dynamicPushCooldownUntil[cooldownKey] =
                _dynamicPushElapsedSeconds + (std::max)(0.0f, g_rockConfig.rockDynamicPushCooldownSeconds);
            auto* baseObj = targetRef->GetObjectReference();
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const std::string nameStr = objName.empty() ? std::string("(unnamed)") : std::string(objName);
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push applied: '{}' formID={:08X} targetBody={} layer={} acceptedBodies={} uniqueMotions={} impulse=({:.3f},{:.3f},{:.3f})",
                sourceName,
                nameStr,
                targetRef->GetFormID(),
                targetBodyId,
                targetRecord->collisionLayer,
                bodySet.acceptedCount(),
                appliedCount,
                push.impulse.x,
                push.impulse.y,
                push.impulse.z);
        }
    }

    void PhysicsInteraction::resolveAndLogContact(const char* handName, RE::bhkWorld* bhk, RE::hknpWorld* hknp, RE::hknpBodyId bodyId)
    {
        if (!bhk || !hknp)
            return;

        auto& body = hknp->GetBody(bodyId);
        auto layer = body.collisionFilterInfo & 0x7F;

        auto* ref = resolveBodyToRef(bhk, hknp, bodyId);
        if (ref) {
            auto* baseObj = ref->GetObjectReference();
            const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const std::string nameStr = objName.empty() ? std::string("(unnamed)") : std::string(objName);

            ROCK_LOG_DEBUG(Hand, "{} hand touched [{}] '{}' formID={:08X} body={} layer={}", handName, typeName, nameStr, ref->GetFormID(), bodyId.value, layer);

            bool isLeft = (std::string_view(handName) == "Left");
            auto& hand = isLeft ? _leftHand : _rightHand;
            hand.setTouchState(ref, ref->GetFormID(), layer);
            dispatchPhysicsMessage(kPhysMsg_OnTouch, isLeft, ref, ref->GetFormID(), layer);
        } else {
            ROCK_LOG_DEBUG(Hand, "{} hand touched body={} layer={} (unresolved)", handName, bodyId.value, layer);
        }
    }

    void PhysicsInteraction::subscribeContactEvents(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Init, "Contact event subscription skipped because world is null");
            return;
        }

        void* signal = world->GetEventSignal(RE::hknpEventType::kContact);
        if (!signal) {
            ROCK_LOG_ERROR(Init, "Failed to get contact event signal");
            return;
        }

        auto* currentWorld = _contactEventWorld.load(std::memory_order_acquire);
        auto* currentSignal = _contactEventSignal.load(std::memory_order_acquire);
        const auto currentSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(currentWorld),
            .signal = reinterpret_cast<std::uintptr_t>(currentSignal),
            .active = currentWorld != nullptr && currentSignal != nullptr,
        };
        const auto plan = contact_signal_subscription_policy::planSubscription(
            currentSnapshot,
            reinterpret_cast<std::uintptr_t>(world),
            reinterpret_cast<std::uintptr_t>(signal));

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::IgnoreNullSignal) {
            ROCK_LOG_ERROR(Init, "Contact event subscription skipped because world or signal is null");
            return;
        }

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::AlreadySubscribed) {
            ROCK_LOG_DEBUG(Init, "Contact event signal already subscribed for current world");
            return;
        }

        if (plan.unsubscribeExistingSignal) {
            unsubscribeContactEvents(world);
        } else if (plan.clearExistingWithoutUnsubscribe) {
            _contactEventWorld.store(nullptr, std::memory_order_release);
            _contactEventSignal.store(nullptr, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Cleared stale contact event subscription state before subscribing new world");
        }

        ContactEventCallbackInfo cbInfo{};
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
        static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
        subscribeExt(signal, static_cast<void*>(this), &cbInfo);

        _contactEventSignal.store(signal, std::memory_order_release);
        _contactEventWorld.store(world, std::memory_order_release);
        ROCK_LOG_INFO(Init, "Subscribed to contact events");
    }

    void PhysicsInteraction::unsubscribeContactEvents(RE::hknpWorld* liveWorld)
    {
        const auto world = _contactEventWorld.load(std::memory_order_acquire);
        const auto signal = _contactEventSignal.load(std::memory_order_acquire);
        const auto snapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(world),
            .signal = reinterpret_cast<std::uintptr_t>(signal),
            .active = world != nullptr && signal != nullptr,
        };

        if (!contact_signal_subscription_policy::isActiveSubscription(snapshot)) {
            return;
        }

        _contactEventWorld.store(nullptr, std::memory_order_release);
        _contactEventSignal.store(nullptr, std::memory_order_release);

        if (!contact_signal_subscription_policy::canUnsubscribeFromWorld(snapshot, reinterpret_cast<std::uintptr_t>(liveWorld))) {
            ROCK_LOG_INFO(Init, "Cleared contact event subscription state without native unsubscribe because the subscribed world is stale");
            return;
        }

        ContactEventCallbackInfo cbInfo{};
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        std::uint8_t removed = 0;
        typedef void* unsubscribe_ext_t(void* signal, std::uint8_t* outRemoved, void* userData, void* callbackInfo, std::uint64_t callbackInfoSize);
        static REL::Relocation<unsubscribe_ext_t> unsubscribeExt{ REL::Offset(offsets::kFunc_UnsubscribeSignalCallback) };
        unsubscribeExt(signal, &removed, static_cast<void*>(this), &cbInfo, sizeof(cbInfo));

        ROCK_LOG_INFO(Init, "Unsubscribed from contact events (removed={})", removed != 0 ? "yes" : "no");
    }

    void PhysicsInteraction::onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        __try {
            if (!s_hooksEnabled.load(std::memory_order_acquire))
                return;
            auto* self = s_instance.load(std::memory_order_acquire);
            if (self && self->_initialized.load(std::memory_order_acquire)) {
                auto* subscribedWorld = self->_contactEventWorld.load(std::memory_order_acquire);
                auto* subscribedSignal = self->_contactEventSignal.load(std::memory_order_acquire);
                const auto snapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
                    .world = reinterpret_cast<std::uintptr_t>(subscribedWorld),
                    .signal = reinterpret_cast<std::uintptr_t>(subscribedSignal),
                    .active = subscribedWorld != nullptr && subscribedSignal != nullptr,
                };
                if (userData != static_cast<void*>(self)) {
                    return;
                }

                std::uintptr_t callbackWorld = 0;
                if (worldPtrHolder) {
                    callbackWorld = reinterpret_cast<std::uintptr_t>(*worldPtrHolder);
                }

                const auto acceptance = contact_signal_subscription_policy::evaluateCallbackAcceptance(snapshot, callbackWorld);
                if (!acceptance.accept) {
                    return;
                }

                self->handleContactEvent(reinterpret_cast<RE::hknpWorld*>(acceptance.effectiveWorld), contactEventData);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehLogCounter = 0;
            if (sehLogCounter++ % 100 == 0) {
                logger::error(
                    "[ROCK::Contact] SEH exception caught on physics thread (count={}) — "
                    "likely stale world during cell transition",
                    sehLogCounter);
            }
            s_hooksEnabled.store(false, std::memory_order_release);
        }
    }

    void PhysicsInteraction::handleContactEvent(RE::hknpWorld* world, void* contactEventData)
    {
        if (!contactEventData)
            return;

        auto* data = reinterpret_cast<std::uint8_t*>(contactEventData);
        std::uint32_t bodyIdA = *reinterpret_cast<std::uint32_t*>(data + 0x08);
        std::uint32_t bodyIdB = *reinterpret_cast<std::uint32_t*>(data + 0x0C);

        if (!contact_pipeline_policy::isValidBodyId(bodyIdA) || !contact_pipeline_policy::isValidBodyId(bodyIdB) || bodyIdA == bodyIdB) {
            return;
        }

        if (!havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdA }) ||
            !havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdB })) {
            return;
        }

        havok_runtime::ContactSignalPointResult rawContactPoint{};
        bool rawContactPointEvaluated = false;
        bool hasRawContactPoint = false;
        auto ensureRawContactPoint = [&]() {
            if (!rawContactPointEvaluated) {
                hasRawContactPoint = havok_runtime::tryExtractContactSignalPoint(world, contactEventData, rawContactPoint);
                rawContactPointEvaluated = true;
            }
            return hasRawContactPoint;
        };

        const auto rightId = _rightHand.getCollisionBodyId().value;
        const auto leftId = _leftHand.getCollisionBodyId().value;

        struct HandContactSource
        {
            bool valid = false;
            bool isLeft = false;
            bool primaryAnchor = false;
            HandColliderBodyMetadata metadata{};
        };

        auto classifyHandBody = [](const Hand& hand, std::uint32_t bodyId, bool isLeft, std::uint32_t anchorId) {
            HandContactSource source{};
            HandColliderBodyMetadata metadata{};
            if (hand.tryGetHandColliderMetadata(bodyId, metadata)) {
                source.valid = true;
                source.isLeft = isLeft;
                source.primaryAnchor = metadata.primaryPalmAnchor || bodyId == anchorId;
                source.metadata = metadata;
                source.metadata.isLeft = isLeft;
                return source;
            }

            if (bodyId == anchorId && bodyId != INVALID_BODY_ID) {
                source.valid = true;
                source.isLeft = isLeft;
                source.primaryAnchor = true;
                source.metadata.valid = true;
                source.metadata.isLeft = isLeft;
                source.metadata.primaryPalmAnchor = true;
                source.metadata.bodyId = bodyId;
                source.metadata.role = hand_collider_semantics::HandColliderRole::PalmAnchor;
            }
            return source;
        };

        const auto bodyARight = classifyHandBody(_rightHand, bodyIdA, false, rightId);
        const auto bodyBRight = classifyHandBody(_rightHand, bodyIdB, false, rightId);
        const auto bodyALeft = classifyHandBody(_leftHand, bodyIdA, true, leftId);
        const auto bodyBLeft = classifyHandBody(_leftHand, bodyIdB, true, leftId);
        const bool bodyAIsRight = bodyARight.valid;
        const bool bodyBIsRight = bodyBRight.valid;
        const bool bodyAIsLeft = bodyALeft.valid;
        const bool bodyBIsLeft = bodyBLeft.valid;
        const bool bodyAIsExternal = ::rock::provider::isExternalBodyId(bodyIdA);
        const bool bodyBIsExternal = ::rock::provider::isExternalBodyId(bodyIdB);
        const bool bodyAIsRightHeld = _rightHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsRightHeld = _rightHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsLeftHeld = _leftHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsLeftHeld = _leftHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdA);
        const bool bodyBIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdB);
        const bool bodyAIsRockSource = bodyAIsRight || bodyAIsLeft || bodyAIsRightHeld || bodyAIsLeftHeld || bodyAIsWeapon;
        const bool bodyBIsRockSource = bodyBIsRight || bodyBIsLeft || bodyBIsRightHeld || bodyBIsLeftHeld || bodyBIsWeapon;

        if (contact_pipeline_policy::shouldSkipContactSignalBeforeLayerRead(contact_pipeline_policy::ContactSignalPrefilter{
                .bodyIdA = bodyIdA,
                .bodyIdB = bodyIdB,
                .bodyAIsRockSource = bodyAIsRockSource,
                .bodyBIsRockSource = bodyBIsRockSource,
            })) {
            return;
        }

        auto readBodyLayer = [world](std::uint32_t bodyId) {
            std::uint32_t filterInfo = 0;
            if (world && havok_runtime::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, filterInfo)) {
                return filterInfo & 0x7F;
            }
            return contact_pipeline_policy::kUnknownLayer;
        };

        const std::uint32_t bodyALayer = readBodyLayer(bodyIdA);
        const std::uint32_t bodyBLayer = readBodyLayer(bodyIdB);

        auto makeEndpoint = [&](std::uint32_t bodyId, std::uint32_t layer, bool isRightHand, bool isLeftHand, bool isWeapon, bool isRightHeld, bool isLeftHeld, bool isExternal) {
            using contact_pipeline_policy::ContactEndpoint;
            using contact_pipeline_policy::ContactEndpointKind;

            ContactEndpoint endpoint{};
            endpoint.bodyId = bodyId;
            endpoint.layer = layer;
            if (isRightHand) {
                endpoint.kind = ContactEndpointKind::RightHand;
            } else if (isLeftHand) {
                endpoint.kind = ContactEndpointKind::LeftHand;
            } else if (isWeapon) {
                endpoint.kind = ContactEndpointKind::Weapon;
            } else if (isRightHeld) {
                endpoint.kind = ContactEndpointKind::RightHeldObject;
            } else if (isLeftHeld) {
                endpoint.kind = ContactEndpointKind::LeftHeldObject;
            } else if (isExternal) {
                endpoint.kind = ContactEndpointKind::External;
            } else {
                endpoint.kind = contact_pipeline_policy::classifyNonRockLayer(layer);
            }
            return endpoint;
        };

        const auto endpointA = makeEndpoint(bodyIdA, bodyALayer, bodyAIsRight, bodyAIsLeft, bodyAIsWeapon, bodyAIsRightHeld, bodyAIsLeftHeld, bodyAIsExternal);
        const auto endpointB = makeEndpoint(bodyIdB, bodyBLayer, bodyBIsRight, bodyBIsLeft, bodyBIsWeapon, bodyBIsRightHeld, bodyBIsLeftHeld, bodyBIsExternal);
        const auto contactRoute = contact_pipeline_policy::classifyContact(endpointA, endpointB);

        auto handSourceFor = [&](std::uint32_t bodyId) -> const HandContactSource* {
            if (bodyARight.valid && bodyARight.metadata.bodyId == bodyId) {
                return &bodyARight;
            }
            if (bodyBRight.valid && bodyBRight.metadata.bodyId == bodyId) {
                return &bodyBRight;
            }
            if (bodyALeft.valid && bodyALeft.metadata.bodyId == bodyId) {
                return &bodyALeft;
            }
            if (bodyBLeft.valid && bodyBLeft.metadata.bodyId == bodyId) {
                return &bodyBLeft;
            }
            return nullptr;
        };

        auto fillSourceVelocity = [world](std::uint32_t sourceBodyId, ::rock::provider::RockProviderExternalContactV2& contact) {
            if (!world || sourceBodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ sourceBodyId });
            if (!motion) {
                return;
            }

            contact.sourceVelocityHavok[0] = motion->linearVelocity.x;
            contact.sourceVelocityHavok[1] = motion->linearVelocity.y;
            contact.sourceVelocityHavok[2] = motion->linearVelocity.z;
        };

        auto publishExternalContact = [&](std::uint32_t sourceBodyId,
                                          std::uint32_t externalBodyId,
                                          ::rock::provider::RockProviderExternalSourceKind sourceKind,
                                          ::rock::provider::RockProviderHand sourceHand,
                                          const HandColliderBodyMetadata* handMetadata = nullptr) {
            if (sourceBodyId == INVALID_CONTACT_BODY_ID || externalBodyId == INVALID_CONTACT_BODY_ID || sourceBodyId == externalBodyId) {
                return;
            }

            ::rock::provider::RockProviderExternalContactV2 contact{};
            contact.sourceBodyId = sourceBodyId;
            contact.targetExternalBodyId = externalBodyId;
            contact.sourceKind = sourceKind;
            contact.sourceHand = sourceHand;
            contact.quality = ::rock::provider::RockProviderExternalContactQuality::BodyPairOnly;
            fillSourceVelocity(sourceBodyId, contact);

            if (ensureRawContactPoint()) {
                contact.quality = ::rock::provider::RockProviderExternalContactQuality::RawPoint;
                contact.contactPointWeightSum = rawContactPoint.contactPointWeightSum;
                std::copy_n(rawContactPoint.contactPointHavok, 4, contact.contactPointHavok);
                std::copy_n(rawContactPoint.contactNormalHavok, 4, contact.contactNormalHavok);
            }

            if (handMetadata && handMetadata->valid) {
                contact.sourceRole = static_cast<std::uint32_t>(handMetadata->role);
                contact.sourcePartKind = static_cast<std::uint32_t>(handMetadata->finger);
                contact.sourceSubRole = static_cast<std::uint32_t>(handMetadata->segment);
            } else if (sourceKind == ::rock::provider::RockProviderExternalSourceKind::Weapon) {
                WeaponInteractionContact weaponContact{};
                if (_weaponCollision.tryGetWeaponContactAtomic(sourceBodyId, weaponContact)) {
                    contact.sourcePartKind = static_cast<std::uint32_t>(weaponContact.partKind);
                    contact.sourceRole = static_cast<std::uint32_t>(weaponContact.reloadRole);
                    contact.sourceSubRole = static_cast<std::uint32_t>(weaponContact.supportGripRole);
                }
            }

            ::rock::provider::recordExternalContact(contact);
        };

        if (_rightHand.isHoldingAtomic()) {
            if (bodyAIsRightHeld || bodyBIsRightHeld) {
                std::uint32_t heldId = bodyAIsRightHeld ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (!_rightHand.isHandColliderBodyId(other) && !_leftHand.isHandColliderBodyId(other) && other != rightId && other != leftId &&
                    !::rock::provider::isExternalBodyId(other)) {
                    _rightHand.notifyHeldBodyContact();
                }
            }
        }
        if (_leftHand.isHoldingAtomic()) {
            if (bodyAIsLeftHeld || bodyBIsLeftHeld) {
                std::uint32_t heldId = bodyAIsLeftHeld ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (!_rightHand.isHandColliderBodyId(other) && !_leftHand.isHandColliderBodyId(other) && other != rightId && other != leftId &&
                    !::rock::provider::isExternalBodyId(other)) {
                    _leftHand.notifyHeldBodyContact();
                }
            }
        }

        bool isRight = bodyAIsRight || bodyBIsRight;
        bool isLeft = bodyAIsLeft || bodyBIsLeft;

        if (contactRoute.publishExternalContact) {
            const HandColliderBodyMetadata* handMetadata = nullptr;
            if (contactRoute.providerSourceKind == ::rock::provider::RockProviderExternalSourceKind::Hand) {
                const auto* handSource = handSourceFor(contactRoute.sourceBodyId);
                handMetadata = handSource && handSource->valid ? &handSource->metadata : nullptr;
            }

            publishExternalContact(contactRoute.sourceBodyId, contactRoute.targetBodyId, contactRoute.providerSourceKind, contactRoute.providerSourceHand, handMetadata);
        }

        if (contactRoute.driveWeaponDynamicPush) {
            _lastContactSourceWeapon.store(contactRoute.sourceBodyId, std::memory_order_release);
            _lastContactBodyWeapon.store(contactRoute.targetBodyId, std::memory_order_release);
        }

        if (contactRoute.recordWorldSurfaceEvidence) {
            int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
            if (logCount % 60 == 0) {
                ROCK_LOG_DEBUG(Hand,
                    "Surface contact evidence: route={} sourceBody={} targetBody={} targetLayer={}",
                    contact_pipeline_policy::routeName(contactRoute.route),
                    contactRoute.sourceBodyId,
                    contactRoute.targetBodyId,
                    contactRoute.target.layer == contact_pipeline_policy::kUnknownLayer ? 0xFFFFFFFFu : contactRoute.target.layer);
            }
        }

        auto publishLeftWeaponContactFromPhysics = [&](const WeaponInteractionContact& weaponContact, std::uint32_t bodyId) {
            _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(weaponContact.partKind), std::memory_order_release);
            _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(weaponContact.reloadRole), std::memory_order_release);
            _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(weaponContact.supportGripRole), std::memory_order_release);
            _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(weaponContact.socketRole), std::memory_order_release);
            _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(weaponContact.actionRole), std::memory_order_release);
            _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(weaponContact.fallbackGripPose), std::memory_order_release);
            _leftWeaponContactSequence.fetch_add(1, std::memory_order_acq_rel);
            _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
            _leftWeaponContactBodyId.store(bodyId, std::memory_order_release);
        };

        if (!isRight && !isLeft) {
            return;
        }

        if (contactRoute.route == contact_pipeline_policy::ContactRoute::RockInternal) {
            return;
        }

        if (contactRoute.drivesWeaponSupportContact && contact_pipeline_policy::isLeftHand(contactRoute.source.kind)) {
            WeaponInteractionContact weaponContact{};
            if (_weaponCollision.tryGetWeaponContactAtomic(contactRoute.targetBodyId, weaponContact)) {
                publishLeftWeaponContactFromPhysics(weaponContact, contactRoute.targetBodyId);
            }
        }

        const auto* handSource = contactRoute.recordHandSemanticContact ? handSourceFor(contactRoute.sourceBodyId) : nullptr;
        if (!handSource || !handSource->valid) {
            return;
        }

        const auto contactActivity = _handContactActivity.registerHandContact(handSource->isLeft, handSource->metadata.bodyId, contactRoute.targetBodyId);
        if (contactActivity.newlyActive && g_rockConfig.rockDebugVerboseLogging) {
            ROCK_LOG_DEBUG(Hand,
                "ContactActivity: {} {} body={} target={} frame={} inserted={} evictedStale={}",
                handSource->isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(handSource->metadata.role),
                handSource->metadata.bodyId,
                contactRoute.targetBodyId,
                contactActivity.frame,
                contactActivity.inserted ? "yes" : "no",
                contactActivity.evictedStale ? "yes" : "no");
        }

        if (handSource->isLeft) {
            _leftHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId);
            if (contactRoute.driveHandDynamicPush) {
                _lastContactSourceLeft.store(handSource->metadata.bodyId, std::memory_order_release);
                _lastContactBodyLeft.store(contactRoute.targetBodyId, std::memory_order_release);
            }
        } else {
            _rightHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId);
            if (contactRoute.driveHandDynamicPush) {
                _lastContactSourceRight.store(handSource->metadata.bodyId, std::memory_order_release);
                _lastContactBodyRight.store(contactRoute.targetBodyId, std::memory_order_release);
            }
        }

        int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
        if (logCount % 30 == 0) {
            ROCK_LOG_DEBUG(Hand,
                "Contact: {} {} body={} hit body {} route={}",
                handSource->isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(handSource->metadata.role),
                handSource->metadata.bodyId,
                contactRoute.targetBodyId,
                contact_pipeline_policy::routeName(contactRoute.route));
        }
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        return _ownedObjects.contains(ref->GetFormID());
    }

    void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        _ownedObjects.insert(formID);
        ROCK_LOG_DEBUG(Hand, "Claimed object: formID={:08X}", formID);
    }

    void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        if (_ownedObjects.erase(formID)) {
            ROCK_LOG_DEBUG(Hand, "Released object: formID={:08X}", formID);
        }
    }

    void PhysicsInteraction::releaseAllObjects()
    {
        std::scoped_lock lock(_ownedObjectsMutex);
        if (!_ownedObjects.empty()) {
            ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _ownedObjects.size());
            _ownedObjects.clear();
        }

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", false);
        }
    }

    void PhysicsInteraction::forceDropHeldObject(bool isLeft)
    {
        auto& hand = isLeft ? _leftHand : _rightHand;
        if (!hand.isHolding())
            return;

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no bhkWorld available");
            return;
        }
        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no hknpWorld available");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        ROCK_LOG_INFO(Hand, "forceDropHeldObject: {} hand dropping {}", isLeft ? "Left" : "Right", heldRef ? heldRef->GetFormID() : 0);

        hand.releaseGrabbedObject(hknp);
        if (heldRef)
            releaseObject(heldRef);
    }

    RE::bhkWorld* PhysicsInteraction::getPlayerBhkWorld() const
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player)
            return nullptr;

        auto* cell = player->GetParentCell();
        if (!cell)
            return nullptr;

        return cell->GetbhkWorld();
    }

    RE::hknpWorld* PhysicsInteraction::getHknpWorld(RE::bhkWorld* bhk)
    {
        if (!bhk)
            return nullptr;

        constexpr std::uintptr_t kBhkWorld_HknpWorldPtr = 0x60;
        return *reinterpret_cast<RE::hknpWorld**>(reinterpret_cast<uintptr_t>(bhk) + kBhkWorld_HknpWorldPtr);
    }
}
