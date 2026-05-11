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
#include <vector>

#include "common/CommonUtils.h"
#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESDataHandler.h"
#include "RE/Bethesda/TESFile.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "f4vr/PlayerNodes.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/SeeThroughScopesPolicy.h"
#include "RockConfig.h"

namespace
{
    using PropertyMod = RE::BGSMod::Property::Mod;

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

    struct RuntimeState
    {
        bool initialized = false;
        bool detected = false;
        std::string detectedPlugin;
        bool legacyDllWarningLogged = false;
        bool overlayPatchApplied = false;
        std::vector<OverlayPatchRecord> overlayPatchRecords;
        RE::NiPointer<RE::NiNode> reticleNode;
        RE::NiPoint3 reticleBaselineLocal = RE::NiPoint3::ZERO;
        bool reticleBaselineValid = false;
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

    [[nodiscard]] std::optional<std::string> matchLoadedFile(const RE::TESFile* file)
    {
        if (!file) {
            return std::nullopt;
        }

        const auto filename = file->GetFilename();
        if (rock::see_through_scopes_policy::isSeeThroughScopesPluginFilename(filename)) {
            return std::string{ filename };
        }

        return std::nullopt;
    }

    [[nodiscard]] std::optional<std::string> findLoadedSeeThroughScopesPlugin()
    {
        auto* dataHandler = RE::TESDataHandler::GetSingleton();
        if (!dataHandler) {
            return std::nullopt;
        }

        if (const auto* compiledFiles = dataHandler->GetCompiledFileCollection()) {
            for (const auto* file : compiledFiles->files) {
                if (auto matched = matchLoadedFile(file)) {
                    return matched;
                }
            }

            for (const auto* file : compiledFiles->smallFiles) {
                if (auto matched = matchLoadedFile(file)) {
                    return matched;
                }
            }
        }

        if (const auto* vrMods = dataHandler->GetVRModData()) {
            for (std::uint32_t i = 0; i < vrMods->loadedModCount; ++i) {
                if (auto matched = matchLoadedFile(vrMods->loadedMods[i])) {
                    return matched;
                }
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
            "Patched {} native scope overlay properties across {} OMOD records.",
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

    void keepScopeMeshVisible()
    {
        if (!isPlayerWeaponDrawn()) {
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

        scopeNormalFlags |= kNodeHiddenFlag;
        scopeAimingFlags &= ~kNodeHiddenFlag;

        if (!writeNodeFlags(scopeNormal, scopeNormalFlags, "ScopeNormal visibility") ||
            !writeNodeFlags(scopeAiming, scopeAimingFlags, "ScopeAiming visibility")) {
            return;
        }
    }

    void alignReticle()
    {
        if (!reticleAlignmentConfigEnabled() || !isPlayerWeaponDrawn()) {
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

        const auto matchedPlugin = findLoadedSeeThroughScopesPlugin();
        const bool detected = matchedPlugin.has_value();
        const bool changed = !s_state.initialized ||
            detected != s_state.detected ||
            (detected && s_state.detectedPlugin != *matchedPlugin);

        s_state.initialized = true;
        s_state.detected = detected;
        s_state.detectedPlugin = matchedPlugin.value_or(std::string{});

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
        }
    }

    void resetRuntimeState()
    {
        restoreOverlayPatch();
        restoreReticleBaselineIfPresent();
        s_state.initialized = false;
        s_state.detected = false;
        s_state.detectedPlugin.clear();
    }

    void updateFrame()
    {
        if (!s_state.initialized) {
            refreshRuntimeState();
        }

        if (!runtimeActive()) {
            restoreOverlayPatch();
            restoreReticleBaselineIfPresent();
            return;
        }

        applyOverlayPatch();
        alignReticle();
    }

    void updateLateCulling()
    {
        if (!runtimeActive()) {
            return;
        }

        keepScopeMeshVisible();
    }
}
