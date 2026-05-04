#include "physics-interaction/core/PhysicsInteraction.h"

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

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/HeldPlayerSpaceRegistry.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

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

#include "physics-interaction/core/PhysicsInteractionProvider.inl"
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
            if (auto* filterPtr = havok_runtime::getQueryFilterRefWithFallback(hknp)) {
                auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);
                const auto currentHandMask = matrix[ROCK_HAND_LAYER];
                const auto currentWeaponMask = matrix[ROCK_WEAPON_LAYER];
                const auto currentReloadMask = matrix[collision_layer_policy::ROCK_LAYER_RELOAD];
                const bool handMaskDrifted = _expectedHandLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                const bool weaponMaskDrifted = _expectedWeaponLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                const bool reloadMaskDrifted = _expectedReloadLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
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

        auto* worldFilter = havok_runtime::getQueryFilterRef(world);
        auto* filterPtr = worldFilter ? worldFilter : havok_runtime::getQueryFilterRefWithFallback(world);
        if (!filterPtr) {
            ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
            return;
        }
        ROCK_LOG_DEBUG(Config, "Filter source: selected={:p}, usedFallback={}", filterPtr, worldFilter ? "no" : "yes");

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

#include "physics-interaction/core/PhysicsInteractionDebugOverlay.inl"
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

#include "physics-interaction/core/PhysicsInteractionContacts.inl"
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

        return havok_runtime::getHknpWorldFromBhk(bhk);
    }
}
