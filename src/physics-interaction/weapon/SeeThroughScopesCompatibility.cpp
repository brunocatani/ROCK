#include "physics-interaction/weapon/SeeThroughScopesCompatibility.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "common/CommonUtils.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESDataHandler.h"
#include "RE/Bethesda/TESFile.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/SeeThroughScopesPolicy.h"
#include "RockConfig.h"

namespace
{
    using PropertyMod = RE::BGSMod::Property::Mod;
    using EquippedScopeRoute = rock::see_through_scopes_policy::EquippedScopeRoute;

    constexpr std::uintptr_t kLateCullingHookCallSite = 0xD84EE4;
    constexpr std::uint64_t kNodeHiddenFlag = 0x1;
    constexpr float kVectorEpsilon = 1.0e-4f;

    using LateCullingFunc = void (*)(std::uint64_t rcx);
    LateCullingFunc s_originalLateCulling = nullptr;

    struct OverlayPatchRecord
    {
        PropertyMod* property = nullptr;
        std::array<std::byte, sizeof(PropertyMod::DATATYPE)> originalData{};
    };

    struct EquippedScopeRouteSnapshot
    {
        EquippedScopeRoute route = EquippedScopeRoute::None;
        std::uint32_t weaponFormID = 0;
        std::uint32_t activeStsScopeMods = 0;
        std::uint32_t activeNativeScopeMods = 0;
        std::uint32_t unresolvedActiveMods = 0;
        bool weaponDrawn = false;
    };

    struct RuntimeState
    {
        bool initialized = false;
        bool detected = false;
        std::string detectedPlugin;
        std::vector<const RE::TESFile*> detectedFiles;
        bool legacyDllWarningLogged = false;
        bool overlayPatchApplied = false;
        std::vector<OverlayPatchRecord> overlayPatchRecords;
        RE::NiPointer<RE::NiNode> reticleNode;
        RE::NiPoint3 reticleBaselineLocal = RE::NiPoint3::ZERO;
        bool reticleBaselineValid = false;
        RE::NiPointer<RE::NiAVObject> scopeNormalNode;
        RE::NiPointer<RE::NiAVObject> scopeAimingNode;
        std::uint64_t scopeNormalBaselineFlags = 0;
        std::uint64_t scopeAimingBaselineFlags = 0;
        bool scopeVisibilityBaselineValid = false;
        EquippedScopeRouteSnapshot scopeRoute;
        EquippedScopeRouteSnapshot loggedScopeRoute;
        bool hasLoggedScopeRoute = false;
    };

    RuntimeState s_state;

    [[nodiscard]] bool compatibilityConfigEnabled()
    {
        return rock::g_rockConfig.rockEnabled && rock::g_rockConfig.rockSeeThroughScopesCompatibilityEnabled;
    }

    [[nodiscard]] bool reticleAlignmentConfigEnabled()
    {
        return compatibilityConfigEnabled() && rock::g_rockConfig.rockSeeThroughScopesReticleAlignmentEnabled;
    }

    [[nodiscard]] bool runtimeActive()
    {
        return s_state.initialized && compatibilityConfigEnabled() && s_state.detected;
    }

    void logInvalidNativeNode(const char* context, const void* ptr, const char* reason)
    {
        ROCK_LOG_SAMPLE_WARN(
            Scope,
            rock::g_rockConfig.rockLogSampleMilliseconds,
            "See-Through Scopes skipped {}: {} node=0x{:X}",
            context,
            reason,
            reinterpret_cast<std::uintptr_t>(ptr));
    }

    [[nodiscard]] bool nativeNodeStorageWritable(RE::NiAVObject* node, const char* context)
    {
        if (!node) {
            return false;
        }

        if (!rock::native_memory::pointerRangeLooksWritable(node, sizeof(RE::NiAVObject))) {
            logInvalidNativeNode(context, node, "native node storage is not writable");
            return false;
        }

        return true;
    }

    template <class T>
    [[nodiscard]] bool readNativeNodeField(const T* field, T& out, const char* context)
    {
        static_assert(std::is_trivially_copyable_v<T>, "STS native node fields must be trivially copyable");
        if (!rock::native_memory::tryReadValue(field, out)) {
            logInvalidNativeNode(context, field, "field read failed");
            return false;
        }

        return true;
    }

    template <class T>
    [[nodiscard]] bool writeNativeNodeField(T* field, const T& value, const char* context)
    {
        static_assert(std::is_trivially_copyable_v<T>, "STS native node fields must be trivially copyable");
        if (!rock::native_memory::tryWriteValue(field, value)) {
            logInvalidNativeNode(context, field, "field write failed");
            return false;
        }

        return true;
    }

    [[nodiscard]] bool readNodeLocalTranslate(RE::NiAVObject* node, RE::NiPoint3& out, const char* context)
    {
        return nativeNodeStorageWritable(node, context) &&
               readNativeNodeField(&node->local.translate, out, context);
    }

    [[nodiscard]] bool writeNodeLocalTranslate(RE::NiAVObject* node, const RE::NiPoint3& value, const char* context)
    {
        return nativeNodeStorageWritable(node, context) &&
               writeNativeNodeField(&node->local.translate, value, context);
    }

    [[nodiscard]] bool readNodeWorldTransform(RE::NiAVObject* node, RE::NiTransform& out, const char* context)
    {
        return nativeNodeStorageWritable(node, context) &&
               readNativeNodeField(&node->world, out, context);
    }

    [[nodiscard]] bool readNodeParent(RE::NiAVObject* node, RE::NiNode*& out, const char* context)
    {
        out = nullptr;
        return nativeNodeStorageWritable(node, context) &&
               readNativeNodeField(&node->parent, out, context);
    }

    [[nodiscard]] bool readNodeFlags(RE::NiAVObject* node, std::uint64_t& out, const char* context)
    {
        out = 0;
        if (!nativeNodeStorageWritable(node, context)) {
            return false;
        }

        std::remove_reference_t<decltype(node->flags.flags)> flags{};
        if (!readNativeNodeField(&node->flags.flags, flags, context)) {
            return false;
        }

        out = static_cast<std::uint64_t>(flags);
        return true;
    }

    [[nodiscard]] bool writeNodeFlags(RE::NiAVObject* node, std::uint64_t value, const char* context)
    {
        if (!nativeNodeStorageWritable(node, context)) {
            return false;
        }

        const auto flags = static_cast<std::remove_reference_t<decltype(node->flags.flags)>>(value);
        return writeNativeNodeField(&node->flags.flags, flags, context);
    }

    void warnIfLegacyBetterScopesDllLoaded()
    {
        if (s_state.legacyDllWarningLogged) {
            return;
        }

        if (!f4cf::common::isDLLModLoaded("FO4VR_better_scopes.dll")) {
            return;
        }

        ROCK_LOG_ERROR(
            Scope,
            "FO4VR_better_scopes.dll is loaded while ROCK owns See-Through-Scopes compatibility. "
            "Disable the old Better Scopes VR DLL to avoid duplicate FOV, FRIK-message, and scope-visibility hooks.");
        s_state.legacyDllWarningLogged = true;
    }

    [[nodiscard]] bool fileIsSeeThroughScopesPlugin(const RE::TESFile* file)
    {
        if (!file) {
            return false;
        }

        const auto filename = file->GetFilename();
        return rock::see_through_scopes_policy::isSeeThroughScopesPluginFilename(filename);
    }

    [[nodiscard]] std::optional<std::string> matchLoadedFile(const RE::TESFile* file)
    {
        if (fileIsSeeThroughScopesPlugin(file)) {
            return std::string{ file->GetFilename() };
        }

        return std::nullopt;
    }

    [[nodiscard]] std::vector<const RE::TESFile*> collectLoadedSeeThroughScopesFiles()
    {
        std::vector<const RE::TESFile*> result;
        auto* dataHandler = RE::TESDataHandler::GetSingleton();
        if (!dataHandler) {
            return result;
        }

        auto addFile = [&](const RE::TESFile* file) {
            if (fileIsSeeThroughScopesPlugin(file)) {
                result.push_back(file);
            }
        };

        if (const auto* compiledFiles = dataHandler->GetCompiledFileCollection()) {
            for (const auto* file : compiledFiles->files) {
                addFile(file);
            }

            for (const auto* file : compiledFiles->smallFiles) {
                addFile(file);
            }
        }

        if (const auto* vrMods = dataHandler->GetVRModData()) {
            for (std::uint32_t i = 0; i < vrMods->loadedModCount; ++i) {
                addFile(vrMods->loadedMods[i]);
            }
        }

        return result;
    }

    [[nodiscard]] std::optional<std::string> firstLoadedSeeThroughScopesPlugin(const std::vector<const RE::TESFile*>& files)
    {
        for (const auto* file : files) {
            if (auto matched = matchLoadedFile(file)) {
                return matched;
            }
        }

        return std::nullopt;
    }

    void setScopeOverlayPropertyFalse(PropertyMod& property)
    {
        std::memset(&property.data, 0, sizeof(property.data));
        property.data.mm.min.i = 0;
        property.data.mm.max.i = 0;
    }

    [[nodiscard]] std::span<PropertyMod> propertyModsFor(RE::BGSMod::Attachment::Mod& attachmentMod)
    {
        return attachmentMod.GetBuffer<PropertyMod>(rock::see_through_scopes_policy::kBgsModPropertyBlockId);
    }

    [[nodiscard]] bool formHasSeeThroughScopesSourceFile(const RE::TESForm& form)
    {
        /*
         * Mixed scope setups need vanilla/non-STS scopes to keep Fallout VR's
         * native overlay. STS OMODs may be new forms or overrides of vanilla
         * OMODs, so source-file participation is the local authority for
         * whether ROCK may suppress target 48 on this attachment mod.
         */
        const auto* sourceFiles = form.sourceFiles.array;
        if (!sourceFiles) {
            return false;
        }

        for (const auto* file : *sourceFiles) {
            if (fileIsSeeThroughScopesPlugin(file)) {
                return true;
            }
        }

        return false;
    }

    [[nodiscard]] bool formOwnedByLoadedSeeThroughScopesPlugin(const RE::TESForm& form)
    {
        /*
         * Some STS records are easier to identify by loaded-file ownership than
         * by sourceFiles participation after overrides. This is still local
         * plugin metadata, not an external reference.
         */
        for (const auto* file : s_state.detectedFiles) {
            if (file && file->IsFormInMod(form.formID)) {
                return true;
            }
        }

        return false;
    }

    [[nodiscard]] bool formIsSeeThroughScopesAttachmentMod(const RE::TESForm& form)
    {
        return formHasSeeThroughScopesSourceFile(form) || formOwnedByLoadedSeeThroughScopesPlugin(form);
    }

    [[nodiscard]] bool attachmentModHasNativeScopeOverlayTarget(RE::BGSMod::Attachment::Mod& omod)
    {
        for (const auto& property : propertyModsFor(omod)) {
            if (property.target == rock::see_through_scopes_policy::kNativeScopeOverlayTarget) {
                return true;
            }
        }

        return false;
    }

    void restoreOverlayPatch()
    {
        if (!s_state.overlayPatchApplied) {
            s_state.overlayPatchRecords.clear();
            return;
        }

        for (const auto& record : s_state.overlayPatchRecords) {
            if (record.property) {
                std::memcpy(&record.property->data, record.originalData.data(), record.originalData.size());
            }
        }

        ROCK_LOG_INFO(Scope, "Restored {} native scope overlay properties.", s_state.overlayPatchRecords.size());
        s_state.overlayPatchRecords.clear();
        s_state.overlayPatchApplied = false;
    }

    void applyOverlayPatch()
    {
        if (s_state.overlayPatchApplied) {
            return;
        }

        auto* dataHandler = RE::TESDataHandler::GetSingleton();
        if (!dataHandler) {
            return;
        }

        std::uint32_t patchedMods = 0;
        auto& omods = dataHandler->GetFormArray<RE::BGSMod::Attachment::Mod>();

        for (auto* omod : omods) {
            if (!omod) {
                continue;
            }

            if (!formIsSeeThroughScopesAttachmentMod(*omod)) {
                continue;
            }

            bool patchedThisMod = false;
            for (auto& property : propertyModsFor(*omod)) {
                if (property.target != rock::see_through_scopes_policy::kNativeScopeOverlayTarget) {
                    continue;
                }

                OverlayPatchRecord record{};
                record.property = &property;
                std::memcpy(record.originalData.data(), &property.data, record.originalData.size());
                s_state.overlayPatchRecords.push_back(record);

                setScopeOverlayPropertyFalse(property);
                patchedThisMod = true;
            }

            if (patchedThisMod) {
                ++patchedMods;
            }
        }

        s_state.overlayPatchApplied = true;
        ROCK_LOG_INFO(
            Scope,
            "Patched {} native scope overlay properties across {} See-Through Scopes OMOD records.",
            s_state.overlayPatchRecords.size(),
            patchedMods);
    }

    [[nodiscard]] RE::NiNode* findWeaponNode()
    {
        auto* skeleton = f4cf::f4vr::getFirstPersonSkeleton();
        if (!skeleton || !nativeNodeStorageWritable(skeleton, "first-person skeleton lookup")) {
            return nullptr;
        }

        static const RE::BSFixedString weaponName{ "Weapon" };
        auto* weaponObject = skeleton->GetObjectByName(weaponName);
        if (!weaponObject || !nativeNodeStorageWritable(weaponObject, "Weapon node lookup")) {
            return nullptr;
        }

        return weaponObject->IsNode();
    }

    [[nodiscard]] RE::NiNode* findReticleNode(RE::NiNode*& weaponNodeOut)
    {
        weaponNodeOut = findWeaponNode();
        if (!weaponNodeOut) {
            return nullptr;
        }

        static const RE::BSFixedString reticleName{ "ReticleNode" };
        auto* reticleObject = weaponNodeOut->GetObjectByName(reticleName);
        if (!reticleObject || !nativeNodeStorageWritable(reticleObject, "ReticleNode lookup")) {
            return nullptr;
        }

        return reticleObject->IsNode();
    }

    void clearReticleBaseline()
    {
        s_state.reticleNode.reset();
        s_state.reticleBaselineLocal = RE::NiPoint3::ZERO;
        s_state.reticleBaselineValid = false;
    }

    void restoreReticleBaselineIfPresent()
    {
        if (!s_state.reticleBaselineValid || !s_state.reticleNode) {
            clearReticleBaseline();
            return;
        }

        auto* reticleNode = s_state.reticleNode.get();
        if (!writeNodeLocalTranslate(reticleNode, s_state.reticleBaselineLocal, "ReticleNode baseline restore")) {
            clearReticleBaseline();
            return;
        }

        f4cf::f4vr::updateDown(reticleNode, true);

        clearReticleBaseline();
    }

    [[nodiscard]] bool isFinitePoint(const RE::NiPoint3& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    [[nodiscard]] std::optional<RE::NiPoint3> normalized(RE::NiPoint3 point)
    {
        if (!isFinitePoint(point)) {
            return std::nullopt;
        }

        const float length = point.Length();
        if (!std::isfinite(length) || length <= kVectorEpsilon) {
            return std::nullopt;
        }

        point /= length;
        return point;
    }

    [[nodiscard]] bool isPlayerWeaponDrawn()
    {
        const auto* player = RE::PlayerCharacter::GetSingleton();
        return player && player->GetWeaponMagicDrawn();
    }

    [[nodiscard]] const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
        const F4SEVR::PlayerCharacter* player,
        const F4SEVR::TESForm* weaponForm,
        const RE::TBO_InstanceData* instanceData)
    {
        if (!player || !weaponForm) {
            return nullptr;
        }

        const auto* reWeaponForm = reinterpret_cast<const RE::TESForm*>(weaponForm);
        auto scanEquipData = [&](const auto* equipData) -> const RE::BGSObjectInstanceExtra* {
            if (!equipData) {
                return nullptr;
            }
            for (std::uint32_t slotIndex = 0; slotIndex < F4SEVR::ActorEquipData::kMaxSlots; ++slotIndex) {
                const auto& slot = equipData->slots[slotIndex];
                if (slot.item != reWeaponForm) {
                    continue;
                }
                if (instanceData && slot.instanceData && slot.instanceData != instanceData) {
                    continue;
                }
                if (slot.extraData) {
                    return slot.extraData;
                }
            }
            return nullptr;
        };

        if (const auto* firstPersonExtra = scanEquipData(player->playerEquipData)) {
            return firstPersonExtra;
        }
        return scanEquipData(player->equipData);
    }

    [[nodiscard]] EquippedScopeRouteSnapshot resolveEquippedScopeRoute()
    {
        EquippedScopeRouteSnapshot snapshot{};
        snapshot.weaponDrawn = isPlayerWeaponDrawn();
        if (!snapshot.weaponDrawn) {
            return snapshot;
        }

        auto* player = f4vr::getPlayer();
        auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
        auto* equipData = processData ? processData->equipData : nullptr;
        auto* weaponForm = equipData ? equipData->item : nullptr;
        if (!weaponForm || weaponForm->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
            return snapshot;
        }

        snapshot.weaponFormID = weaponForm->formID;

        const auto* objectInstanceExtra = findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData);
        if (!objectInstanceExtra || !objectInstanceExtra->values) {
            return snapshot;
        }

        for (const auto& modIndex : objectInstanceExtra->GetIndexData()) {
            if (modIndex.disabled) {
                continue;
            }

            auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
            if (!omod) {
                ++snapshot.unresolvedActiveMods;
                continue;
            }

            if (!attachmentModHasNativeScopeOverlayTarget(*omod)) {
                continue;
            }

            if (formIsSeeThroughScopesAttachmentMod(*omod)) {
                ++snapshot.activeStsScopeMods;
            } else {
                ++snapshot.activeNativeScopeMods;
            }
        }

        snapshot.route = rock::see_through_scopes_policy::chooseEquippedScopeRoute({
            .activeStsScopeMods = snapshot.activeStsScopeMods,
            .activeNativeScopeMods = snapshot.activeNativeScopeMods,
        });
        return snapshot;
    }

    [[nodiscard]] bool sameScopeRouteForLogging(const EquippedScopeRouteSnapshot& left, const EquippedScopeRouteSnapshot& right)
    {
        return left.route == right.route &&
               left.weaponFormID == right.weaponFormID &&
               left.activeStsScopeMods == right.activeStsScopeMods &&
               left.activeNativeScopeMods == right.activeNativeScopeMods &&
               left.unresolvedActiveMods == right.unresolvedActiveMods &&
               left.weaponDrawn == right.weaponDrawn;
    }

    void logScopeRouteIfChanged(const EquippedScopeRouteSnapshot& snapshot)
    {
        if (s_state.hasLoggedScopeRoute && sameScopeRouteForLogging(s_state.loggedScopeRoute, snapshot)) {
            return;
        }

        ROCK_LOG_INFO(
            Scope,
            "Scope route weapon={:08X} route={} stsMods={} nativeMods={} unresolvedMods={} drawn={}",
            snapshot.weaponFormID,
            rock::see_through_scopes_policy::equippedScopeRouteName(snapshot.route),
            snapshot.activeStsScopeMods,
            snapshot.activeNativeScopeMods,
            snapshot.unresolvedActiveMods,
            snapshot.weaponDrawn ? "yes" : "no");

        s_state.loggedScopeRoute = snapshot;
        s_state.hasLoggedScopeRoute = true;
    }

    void clearScopeMeshBaseline()
    {
        s_state.scopeNormalNode.reset();
        s_state.scopeAimingNode.reset();
        s_state.scopeNormalBaselineFlags = 0;
        s_state.scopeAimingBaselineFlags = 0;
        s_state.scopeVisibilityBaselineValid = false;
    }

    void restoreScopeMeshBaselineIfPresent()
    {
        if (!s_state.scopeVisibilityBaselineValid || !s_state.scopeNormalNode || !s_state.scopeAimingNode) {
            clearScopeMeshBaseline();
            return;
        }

        auto* scopeNormal = s_state.scopeNormalNode.get();
        auto* scopeAiming = s_state.scopeAimingNode.get();
        if (!writeNodeFlags(scopeNormal, s_state.scopeNormalBaselineFlags, "ScopeNormal visibility baseline restore") ||
            !writeNodeFlags(scopeAiming, s_state.scopeAimingBaselineFlags, "ScopeAiming visibility baseline restore")) {
            clearScopeMeshBaseline();
            return;
        }

        f4cf::f4vr::updateDown(scopeNormal, true);
        f4cf::f4vr::updateDown(scopeAiming, true);
        clearScopeMeshBaseline();
    }

    void keepScopeMeshVisible(const EquippedScopeRouteSnapshot& equippedScope)
    {
        if (equippedScope.route != EquippedScopeRoute::StsPreferred || !equippedScope.weaponDrawn) {
            restoreScopeMeshBaselineIfPresent();
            return;
        }

        auto* skeleton = f4cf::f4vr::getFirstPersonSkeleton();
        if (!skeleton || !nativeNodeStorageWritable(skeleton, "scope visibility skeleton lookup")) {
            return;
        }

        static const RE::BSFixedString scopeNormalName{ "ScopeNormal" };
        static const RE::BSFixedString scopeAimingName{ "ScopeAiming" };

        auto* scopeNormal = skeleton->GetObjectByName(scopeNormalName);
        auto* scopeAiming = skeleton->GetObjectByName(scopeAimingName);
        if (!scopeNormal || !scopeAiming) {
            return;
        }

        std::uint64_t scopeNormalFlags = 0;
        std::uint64_t scopeAimingFlags = 0;
        if (!readNodeFlags(scopeNormal, scopeNormalFlags, "ScopeNormal visibility") ||
            !readNodeFlags(scopeAiming, scopeAimingFlags, "ScopeAiming visibility")) {
            return;
        }

        if (!s_state.scopeVisibilityBaselineValid ||
            s_state.scopeNormalNode.get() != scopeNormal ||
            s_state.scopeAimingNode.get() != scopeAiming) {
            restoreScopeMeshBaselineIfPresent();
            s_state.scopeNormalNode.reset(scopeNormal);
            s_state.scopeAimingNode.reset(scopeAiming);
            s_state.scopeNormalBaselineFlags = scopeNormalFlags;
            s_state.scopeAimingBaselineFlags = scopeAimingFlags;
            s_state.scopeVisibilityBaselineValid = true;
        }

        scopeNormalFlags |= kNodeHiddenFlag;
        scopeAimingFlags &= ~kNodeHiddenFlag;

        if (!writeNodeFlags(scopeNormal, scopeNormalFlags, "ScopeNormal visibility") ||
            !writeNodeFlags(scopeAiming, scopeAimingFlags, "ScopeAiming visibility")) {
            return;
        }
    }

    void alignReticle(const EquippedScopeRouteSnapshot& equippedScope)
    {
        if (!reticleAlignmentConfigEnabled() ||
            equippedScope.route != EquippedScopeRoute::StsPreferred ||
            !equippedScope.weaponDrawn) {
            restoreReticleBaselineIfPresent();
            return;
        }

        RE::NiNode* weaponNode = nullptr;
        auto* reticleNode = findReticleNode(weaponNode);
        if (!weaponNode || !reticleNode) {
            restoreReticleBaselineIfPresent();
            return;
        }

        if (!s_state.reticleBaselineValid || s_state.reticleNode.get() != reticleNode) {
            restoreReticleBaselineIfPresent();
            RE::NiPoint3 reticleBaselineLocal{};
            if (!readNodeLocalTranslate(reticleNode, reticleBaselineLocal, "ReticleNode baseline capture")) {
                clearReticleBaseline();
                return;
            }

            s_state.reticleNode.reset(reticleNode);
            s_state.reticleBaselineLocal = reticleBaselineLocal;
            s_state.reticleBaselineValid = true;
        }

        if (!writeNodeLocalTranslate(reticleNode, s_state.reticleBaselineLocal, "ReticleNode baseline reset")) {
            restoreReticleBaselineIfPresent();
            return;
        }

        f4cf::f4vr::updateDown(weaponNode, true);

        const auto* playerNodes = f4cf::f4vr::getPlayerNodes();
        const auto* camera = f4cf::f4vr::getPlayerCamera();
        auto* cameraNode = camera ? camera->cameraNode : nullptr;
        RE::NiNode* reticleParent = nullptr;
        if (!playerNodes || !cameraNode || !readNodeParent(reticleNode, reticleParent, "ReticleNode parent") || !reticleParent) {
            return;
        }

        RE::NiTransform eyeTransform{};
        if (!readNodeWorldTransform(cameraNode, eyeTransform, "camera eye transform")) {
            return;
        }

        const float eyeOffsetGameUnits = rock::g_rockConfig.rockSeeThroughScopesRightEyeDominant
            ? rock::g_rockConfig.rockSeeThroughScopesEyeOffsetGameUnits
            : -rock::g_rockConfig.rockSeeThroughScopesEyeOffsetGameUnits;
        RE::NiPoint3 eyeOffset{ eyeOffsetGameUnits, 0.0f, 0.0f };
        eyeTransform.translate += eyeTransform.rotate * eyeOffset;

        RE::NiTransform reticleWorld{};
        if (!readNodeWorldTransform(reticleNode, reticleWorld, "ReticleNode world transform")) {
            return;
        }

        const auto eyeToReticleRaw = reticleWorld.translate - eyeTransform.translate;
        const auto eyeToReticle = normalized(eyeToReticleRaw);
        const auto reticleToBarrel = normalized(s_state.reticleBaselineLocal);
        if (!eyeToReticle || !reticleToBarrel) {
            return;
        }

        auto* hmdNode = playerNodes->HmdNode ? playerNodes->HmdNode : cameraNode;
        RE::NiTransform hmdWorld{};
        if (!readNodeWorldTransform(hmdNode, hmdWorld, "HMD world transform")) {
            return;
        }

        const auto hmdForward = normalized(hmdWorld.rotate * RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
        if (!hmdForward) {
            return;
        }

        RE::NiTransform reticleParentWorld{};
        if (!readNodeWorldTransform(reticleParent, reticleParentWorld, "ReticleNode parent world transform")) {
            return;
        }

        auto localEyeToReticleRaw = reticleParentWorld.rotate.Transpose() * eyeToReticleRaw;
        auto localEyeToReticle = normalized(localEyeToReticleRaw);
        if (!localEyeToReticle) {
            return;
        }

        const float eyeDistance = eyeToReticleRaw.Length();
        const float dotScope = localEyeToReticle->Dot(*reticleToBarrel);
        const float dotHmd = hmdForward->Dot(*eyeToReticle);
        const float lookDotThreshold = rock::g_rockConfig.rockSeeThroughScopesLookDotThreshold;
        const bool lookingThroughScope =
            dotScope > lookDotThreshold &&
            dotHmd > (lookDotThreshold - 0.02f) &&
            eyeDistance <= rock::g_rockConfig.rockSeeThroughScopesDistanceThresholdGameUnits;

        if (!lookingThroughScope) {
            return;
        }

        auto reticleOffset = *reticleToBarrel - *localEyeToReticle;
        reticleOffset.y = 0.0f;
        reticleOffset *= std::max(0.0f, eyeDistance - 1.0f);
        reticleOffset.x += rock::g_rockConfig.rockSeeThroughScopesReticleOffsetXGameUnits;
        reticleOffset.z += rock::g_rockConfig.rockSeeThroughScopesReticleOffsetZGameUnits;

        if (!writeNodeLocalTranslate(reticleNode, s_state.reticleBaselineLocal + reticleOffset, "ReticleNode aligned offset")) {
            restoreReticleBaselineIfPresent();
            return;
        }

        f4cf::f4vr::updateDown(reticleNode, true);
    }

    void onLateCullingHook(const std::uint64_t rcx)
    {
        rock::see_through_scopes::updateLateCulling();

        if (s_originalLateCulling) {
            s_originalLateCulling(rcx);
        }
    }
}

namespace rock::see_through_scopes
{
    bool installLateCullingHook()
    {
        if (s_originalLateCulling) {
            return true;
        }

        REL::Relocation hookCallSite{ REL::Offset(kLateCullingHookCallSite) };
        ROCK_LOG_INFO(Scope, "Hooking late culling at 0x{:X}.", hookCallSite.address());

        auto& trampoline = F4SE::GetTrampoline();
        const auto original = trampoline.write_call<5>(hookCallSite.address(), &onLateCullingHook);
        s_originalLateCulling = reinterpret_cast<LateCullingFunc>(original);

        if (!s_originalLateCulling) {
            ROCK_LOG_CRITICAL(Scope, "Failed to hook late culling: original function pointer is null.");
            return false;
        }

        ROCK_LOG_INFO(Scope, "Late culling hook installed, original: 0x{:X}.", original);
        return true;
    }

    void refreshRuntimeState()
    {
        warnIfLegacyBetterScopesDllLoaded();

        auto detectedFiles = collectLoadedSeeThroughScopesFiles();
        const auto matchedPlugin = firstLoadedSeeThroughScopesPlugin(detectedFiles);
        const bool detected = !detectedFiles.empty();
        const bool changed = !s_state.initialized ||
            detected != s_state.detected ||
            (detected && s_state.detectedPlugin != *matchedPlugin);

        s_state.initialized = true;
        s_state.detected = detected;
        s_state.detectedPlugin = matchedPlugin.value_or(std::string{});
        s_state.detectedFiles = std::move(detectedFiles);

        if (changed) {
            if (s_state.detected) {
                ROCK_LOG_INFO(Scope, "See-Through Scopes compatibility detected '{}'.", s_state.detectedPlugin);
            } else {
                ROCK_LOG_INFO(Scope, "See-Through Scopes compatibility inactive: no loaded 3dscopes plugin found.");
            }
        }

        if (runtimeActive()) {
            applyOverlayPatch();
        } else {
            restoreOverlayPatch();
            restoreReticleBaselineIfPresent();
            restoreScopeMeshBaselineIfPresent();
            s_state.scopeRoute = {};
        }
    }

    void resetRuntimeState()
    {
        restoreOverlayPatch();
        restoreReticleBaselineIfPresent();
        restoreScopeMeshBaselineIfPresent();
        s_state.initialized = false;
        s_state.detected = false;
        s_state.detectedPlugin.clear();
        s_state.detectedFiles.clear();
        s_state.scopeRoute = {};
        s_state.loggedScopeRoute = {};
        s_state.hasLoggedScopeRoute = false;
    }

    void updateFrame()
    {
        if (!s_state.initialized) {
            refreshRuntimeState();
        }

        if (!runtimeActive()) {
            restoreOverlayPatch();
            restoreReticleBaselineIfPresent();
            restoreScopeMeshBaselineIfPresent();
            s_state.scopeRoute = {};
            return;
        }

        applyOverlayPatch();
        s_state.scopeRoute = resolveEquippedScopeRoute();
        logScopeRouteIfChanged(s_state.scopeRoute);
        alignReticle(s_state.scopeRoute);
    }

    void updateLateCulling()
    {
        if (!runtimeActive()) {
            restoreScopeMeshBaselineIfPresent();
            return;
        }

        keepScopeMeshVisible(s_state.scopeRoute);
    }
}
