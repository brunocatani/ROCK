#include "physics-interaction/weapon/WeaponCollision.h"

/*
 * "Which weapon is this, and has anything about it changed?"
 *
 * Two independent identities are computed here, and they answer different
 * questions on purpose:
 *
 *  - the EQUIPPED identity, hashed from the weapon form, its instance data, its
 *    keywords and its installed OMOD list. It changes when the player equips a
 *    different weapon or the workbench changes what is bolted to it, and it is what
 *    gates a collider rebuild.
 *  - the VISUAL composition key, accumulated over the rendered node tree. It
 *    changes when the drawn 3D actually changes, which can happen with no change to
 *    the equipped identity at all - late model streaming, an OMOD subtree the
 *    engine attaches after the build window, an animation swapping a branch.
 *
 * Mixing the two would make ROCK either rebuild every frame or never notice
 * geometry that arrived late, so nothing here folds one into the other.
 *
 * Also here: manual-scope target resolution, which is an equipped-OMOD question
 * even though its answer is published with the collider evidence.
 */

#include "physics-interaction/weapon/WeaponCollisionInternal.h"

#include "RockConfig.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/MagicItems.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/NetImmerse/NiNode.h"

#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rock
{
    using namespace weapon_collision_detail;

    namespace
    {
        void mixFormPointer(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, reinterpret_cast<std::uintptr_t>(form));
            if (form) {
                weapon_visual_composition_policy::mixValue(key, form->formID);
            }
        }

        void mixFormStableContent(std::uint64_t& key, const RE::TESForm* form)
        {
            weapon_visual_composition_policy::mixValue(key, form ? form->formID : 0);
        }

        void mixFloatBits(std::uint64_t& key, float value)
        {
            weapon_visual_composition_policy::mixValue(key, std::bit_cast<std::uint32_t>(value));
        }

        void mixKeywordFormContent(std::uint64_t& key, const RE::BGSKeywordForm* keywords)
        {
            if (!keywords) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, keywords->GetNumKeywords());
            keywords->ForEachKeyword([&](RE::BGSKeyword* keyword) {
                mixFormStableContent(key, keyword);
                return RE::BSContainer::ForEachResult::kContinue;
            });
        }

        void mixBlockBashDataContent(std::uint64_t& key, const RE::BGSBlockBashData* blockBashData)
        {
            if (!blockBashData) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, 1u);
            mixFormStableContent(key, blockBashData->blockBashImpactDataSet);
            mixFormStableContent(key, blockBashData->altBlockMaterialType);
        }

        template <class Form>
        void mixFormPointerArray(std::uint64_t& key, const RE::BSTArray<Form*>* forms)
        {
            if (!forms) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            weapon_visual_composition_policy::mixValue(key, forms->size());
            for (std::uint32_t index = 0; index < forms->size(); ++index) {
                mixFormStableContent(key, (*forms)[index]);
            }
        }

        void mixObjectInstanceExtraContent(std::uint64_t& key, const RE::BGSObjectInstanceExtra* extra)
        {
            if (!extra || !extra->values) {
                weapon_visual_composition_policy::mixValue(key, 0u);
                return;
            }

            const auto indexData = extra->GetIndexData();
            weapon_visual_composition_policy::mixValue(key, 1u);
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);
            }
        }

        struct ObjectInstanceExtraWitness
        {
            std::uint64_t signature{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t activeCount{ 0 };
            std::uint32_t disabledCount{ 0 };
        };

        ObjectInstanceExtraWitness makeObjectInstanceExtraWitness(const RE::BGSObjectInstanceExtra* extra)
        {
            ObjectInstanceExtraWitness witness{};
            if (!extra || !extra->values) {
                return witness;
            }

            const auto indexData = extra->GetIndexData();
            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKObjectInstanceExtraIndexWitnessV1");
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            witness.count = static_cast<std::uint32_t>(
                (std::min)(indexData.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);
                if (modIndex.disabled) {
                    ++witness.disabledCount;
                } else {
                    ++witness.activeCount;
                }
            }
            witness.signature = key;
            return witness;
        }

        // Name-only scan for the structural markers that prove a magnified optic.
        // Stops as soon as the evidence is conclusive: nothing later can change it.
        void collectManualScopeStructuralMarkers(
            RE::NiAVObject* node,
            manual_scope_target_policy::StructuralMarkerEvidence& evidence,
            std::size_t& visited)
        {
            BoundedTreeWalkState state{ .visited = visited };
            auto visitor = [&](RE::NiAVObject* current, int) {
                const char* name = current->name.c_str();
                manual_scope_target_policy::observeStructuralNodeName(evidence, name ? name : "");
                return manual_scope_target_policy::hasMagnifiedScopeStructure(evidence) ?
                    TreeWalkAction::Stop :
                    TreeWalkAction::Descend;
            };
            boundedTreeWalk(node, kTemplateScanMaxDepth, kTemplateScanMaxVisitedNodes, state, visitor);
            visited = state.visited;
        }

        struct EquippedManualScopeTarget
        {
            bool directTransitionRequired{ false };
            bool overlayValid{ false };
            std::uint32_t overlayIndex{ 0 };
        };


        [[nodiscard]] EquippedManualScopeTarget resolveEquippedManualScopeTarget(RE::NiAVObject* assembledWeaponRoot)
        {
            EquippedManualScopeTarget target{};
            auto* player = f4vr::getPlayer();
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            auto* equippedInstanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            const RE::BGSObjectInstanceExtra* objectInstanceExtra =
                weaponForm ? findEquippedWeaponObjectInstanceExtra(player, weaponForm, equippedInstanceData) : nullptr;
            if (!objectInstanceExtra || !objectInstanceExtra->values) {
                return target;
            }

            auto* weapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            auto* instanceData = weapon && equippedInstanceData ?
                static_cast<RE::TESObjectWEAP::InstanceData*>(equippedInstanceData) :
                nullptr;
            RE::BGSZoomData* zoomData = instanceData ? instanceData->zoomData : nullptr;
            if (!zoomData && weapon) {
                zoomData = weapon->weaponData.zoomData;
            }
            if (zoomData) {
                target.overlayIndex = zoomData->zoomData.overlay;
                target.overlayValid = manual_scope_target_policy::isValidNativeOverlayIndex(target.overlayIndex);
            }

            bool nativeScopeMetadataAuthored = instanceData && instanceData->flags.all(RE::WEAPON_FLAGS::kHasScope);
            if (!nativeScopeMetadataAuthored && weapon) {
                nativeScopeMetadataAuthored = weapon->weaponData.flags.all(RE::WEAPON_FLAGS::kHasScope);
            }
            bool explicitScopeModelInstalled = false;
            manual_scope_target_policy::StructuralMarkerEvidence structuralEvidence{};
            std::size_t structuralVisited = 0;
            collectManualScopeStructuralMarkers(assembledWeaponRoot, structuralEvidence, structuralVisited);

            visitEquippedOmodIndexData(objectInstanceExtra,
                [&](const auto& modIndex, auto* omod, const RE::BGSKeyword* attachPointKeyword) {
                // Only an installed, resolvable Mod can contribute scope evidence.
                if (modIndex.disabled || !omod) {
                    return;
                }
                nativeScopeMetadataAuthored = nativeScopeMetadataAuthored || attachmentModHasNativeScopeOverlayTarget(omod->formID);
                explicitScopeModelInstalled = explicitScopeModelInstalled ||
                    manual_scope_target_policy::hasExplicitScopeIdentity(
                        omod->fullName.c_str() ? omod->fullName.c_str() : "",
                        omod->model.c_str() ? omod->model.c_str() : "");

                if (!manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence)) {
                    const std::string_view recordName = omod->fullName.c_str() ? omod->fullName.c_str() : "";
                    const std::string modelPath = omod->model.c_str() ? omod->model.c_str() : "";
                    const bool opticalCandidate =
                        (attachPointKeyword && attachPointKeyword->formID == weapon_part_record_identity_policy::kAttachPointSight) ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "optic") ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "sight") ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(recordName, "scope") ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "optic") ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "sight") ||
                        weapon_effect_geometry_policy::containsAsciiInsensitive(modelPath, "scope");
                    if (opticalCandidate) {
                        auto templateRoot = loadCompleteOmodModelTemplate(modelPath);
                        std::size_t templateVisited = 0;
                        collectManualScopeStructuralMarkers(templateRoot.get(), structuralEvidence, templateVisited);
                    }
                }
            });

            target.directTransitionRequired = manual_scope_target_policy::requiresDirectNativeTransition(
                nativeScopeMetadataAuthored,
                explicitScopeModelInstalled,
                manual_scope_target_policy::hasMagnifiedScopeStructure(structuralEvidence),
                target.overlayValid);
            return target;
        }

        const RE::TESObjectWEAP* asEquippedWeaponForm(const RE::TESForm* form)
        {
            if (!form || form->formType != RE::ENUM_FORM_ID::kWEAP) {
                return nullptr;
            }

            return form->As<RE::TESObjectWEAP>();
        }

        /*
         * Fallout4.esm's WeaponType* keyword records, verified directly against
         * the ESM (2026-07-03) rather than assumed from general modding
         * knowledge. Stored directly on every sampled vanilla WEAP record's own
         * keyword array - no OMOD/template indirection - so a direct
         * HasKeyword() check against the equipped form is sufficient. FormIDs are
         * master-relative (Fallout4.esm is always load-order index 0), matching
         * the existing hardcoded-keyword-lookup precedent in
         * hFRIK/src/FRIK.cpp (RE::TESForm::GetFormByID<RE::BGSKeyword>(0xB34A6)).
         */
        struct WeaponKeywordFormEntry
        {
            std::uint32_t formId;
            WeaponKeywordFlag flag;
        };

        constexpr WeaponKeywordFormEntry kWeaponKeywordForms[] = {
            { 0x0004A0A0, WeaponKeywordFlag::Pistol },
            { 0x0004A0A1, WeaponKeywordFlag::Rifle },
            { 0x00226454, WeaponKeywordFlag::Shotgun },
            { 0x00226455, WeaponKeywordFlag::AssaultRifle },
            { 0x001E325D, WeaponKeywordFlag::Sniper },
            { 0x00226456, WeaponKeywordFlag::GaussRifle },
            { 0x00226452, WeaponKeywordFlag::LaserMusket },
            { 0x0004A0A3, WeaponKeywordFlag::HeavyGun },
            { 0x00226453, WeaponKeywordFlag::HandToHand },
            { 0x0004A0A4, WeaponKeywordFlag::Melee1H },
            { 0x0004A0A5, WeaponKeywordFlag::Melee2H },
            { 0x0005240E, WeaponKeywordFlag::Unarmed },
            { 0x0022575D, WeaponKeywordFlag::Minigun },
            { 0x0022575C, WeaponKeywordFlag::Fatman },
            { 0x0022575B, WeaponKeywordFlag::MissileLauncher },
            { 0x0022575E, WeaponKeywordFlag::GatlingLaser },
            { 0x00225760, WeaponKeywordFlag::Flamer },
            { 0x0022575F, WeaponKeywordFlag::Cryolater },
            { 0x00225763, WeaponKeywordFlag::JunkJet },
            { 0x00225764, WeaponKeywordFlag::RailwayRifle },
            { 0x00225766, WeaponKeywordFlag::Broadsider },
            { 0x00225765, WeaponKeywordFlag::Syringer },
            { 0x00225761, WeaponKeywordFlag::FlareGun },
            { 0x00225762, WeaponKeywordFlag::GammaGun },
            { 0x0016968B, WeaponKeywordFlag::AlienBlaster },
            { 0x00225767, WeaponKeywordFlag::Ripper },
            { 0x00225768, WeaponKeywordFlag::Shishkebab },
            { 0x00092A84, WeaponKeywordFlag::Laser },
            { 0x00092A85, WeaponKeywordFlag::Plasma },
            { 0x00092A86, WeaponKeywordFlag::Ballistic },
            { 0x0004A0A6, WeaponKeywordFlag::Thrown },
            { 0x0010C415, WeaponKeywordFlag::Grenade },
            { 0x0010C414, WeaponKeywordFlag::Mine },
            { 0x0004C922, WeaponKeywordFlag::Explosive },
            { 0x0004A0A2, WeaponKeywordFlag::Automatic },
        };

        struct ResolvedWeaponKeywordEntry
        {
            const RE::BGSKeyword* keyword{ nullptr };
            WeaponKeywordFlag flag{ WeaponKeywordFlag::None };
        };

        const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)>& resolvedWeaponKeywordForms()
        {
            /*
             * Lazily resolved on first use (function-local static, thread-safe
             * magic-static init) because RE::TESForm::GetFormByID requires the
             * game's form table to be populated, which is not guaranteed at
             * static-initialization time. No static initialization-order
             * dependency: this runs on first equipped-weapon identity read,
             * well after data load.
             */
            static const std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> resolved = [] {
                std::array<ResolvedWeaponKeywordEntry, std::size(kWeaponKeywordForms)> table{};
                for (std::size_t i = 0; i < std::size(kWeaponKeywordForms); ++i) {
                    table[i].keyword = RE::TESForm::GetFormByID<RE::BGSKeyword>(kWeaponKeywordForms[i].formId);
                    table[i].flag = kWeaponKeywordForms[i].flag;
                }
                return table;
            }();
            return resolved;
        }

        std::uint64_t computeWeaponKeywordFlags(const RE::TESObjectWEAP* weapon)
        {
            std::uint64_t flags = 0;
            if (!weapon) {
                return flags;
            }
            for (const auto& entry : resolvedWeaponKeywordForms()) {
                if (entry.keyword && weapon->HasKeyword(entry.keyword)) {
                    flags |= static_cast<std::uint64_t>(entry.flag);
                }
            }
            return flags;
        }

        struct WeaponClassificationResult
        {
            WeaponSizeClass sizeClass{ WeaponSizeClass::Rifle };
            WeaponClassificationSource source{ WeaponClassificationSource::Default };
            std::uint64_t keywordFlags{ 0 };
        };

        /*
         * Keyword-primary, weight-fallback: vanilla Fallout4.esm tags every
         * sampled weapon with exactly one (occasionally two, e.g. CombatShotgun
         * carries both Rifle and Shotgun) bucket keyword, but tagging on
         * player-installed weapon mods is author-discretion and unreliable
         * (verified directly: of two installed Glock pistol mods, one tags every
         * weapon with WeaponTypePistol, the other tags none). So a bucket
         * keyword is trusted when present; when absent, this falls back to the
         * existing weight heuristic rather than defaulting blindly.
         */
        WeaponClassificationResult classifyEquippedWeapon(const RE::TESObjectWEAP* weapon, float weightGame)
        {
            WeaponClassificationResult result{};
            if (!weapon) {
                return result;
            }

            result.keywordFlags = computeWeaponKeywordFlags(weapon);
            const auto has = [&](WeaponKeywordFlag flag) { return hasWeaponKeywordFlag(result.keywordFlags, flag); };

            if (has(WeaponKeywordFlag::Melee1H) || has(WeaponKeywordFlag::Melee2H) ||
                has(WeaponKeywordFlag::Unarmed) || has(WeaponKeywordFlag::HandToHand)) {
                result.sizeClass = WeaponSizeClass::Melee;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::HeavyGun)) {
                result.sizeClass = WeaponSizeClass::Heavy;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::Pistol)) {
                result.sizeClass = WeaponSizeClass::Pistol;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }
            if (has(WeaponKeywordFlag::Rifle) || has(WeaponKeywordFlag::Shotgun) ||
                has(WeaponKeywordFlag::AssaultRifle) || has(WeaponKeywordFlag::Sniper) ||
                has(WeaponKeywordFlag::GaussRifle) || has(WeaponKeywordFlag::LaserMusket)) {
                result.sizeClass = WeaponSizeClass::Rifle;
                result.source = WeaponClassificationSource::Keyword;
                return result;
            }

            if (weapon_type_policy::isMelee(weapon->weaponData.type.get())) {
                result.sizeClass = WeaponSizeClass::Melee;
                result.source = WeaponClassificationSource::WeightFallback;
                return result;
            }
            result.source = WeaponClassificationSource::WeightFallback;
            if (weightGame <= g_rockConfig.rockWeaponSizeClassPistolMaxWeight) {
                result.sizeClass = WeaponSizeClass::Pistol;
            } else if (weightGame <= g_rockConfig.rockWeaponSizeClassRifleMaxWeight) {
                result.sizeClass = WeaponSizeClass::Rifle;
            } else {
                result.sizeClass = WeaponSizeClass::Heavy;
            }
            return result;
        }

        std::uint64_t makeEquippedWeaponInstanceContentKey(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra)
        {
            if (!instanceData && !objectInstanceExtra) {
                return 0;
            }

            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKEquippedInstanceContentV3");
            mixFormStableContent(key, weapon);
            if (instanceData) {
                mixKeywordFormContent(key, instanceData->GetKeywordData());
                mixBlockBashDataContent(key, instanceData->GetBlockBashData());
                mixFormPointerArray(key, instanceData->GetEnchantmentArray());
                mixFormPointerArray(key, instanceData->GetMaterialSwapArray());
            }
            mixObjectInstanceExtraContent(key, objectInstanceExtra);
            if (instanceData) {
                mixFloatBits(key, instanceData->GetWeight());
                weapon_visual_composition_policy::mixValue(key, static_cast<std::uint32_t>(instanceData->GetValue()));
                weapon_visual_composition_policy::mixValue(key, instanceData->GetHealth());
                mixFloatBits(key, instanceData->GetColorRemappingIndex());
            }
            return key;
        }

        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity readEquippedWeaponGenerationIdentity()
        {
            weapon_generation_identity_policy::EquippedWeaponGenerationIdentity identity{};

            auto* player = f4vr::getPlayer();
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            auto* instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            if (!weaponForm || weaponForm->formType != RE::ENUM_FORM_ID::kWEAP) {
                return identity;
            }

            identity.hasEquippedWeapon = true;
            identity.formID = weaponForm->formID;
            identity.formAddress = reinterpret_cast<std::uintptr_t>(weaponForm);
            identity.instanceDataAddress = reinterpret_cast<std::uintptr_t>(instanceData);
            identity.instanceKeywordDataAddress = reinterpret_cast<std::uintptr_t>(
                instanceData ? instanceData->GetKeywordData() : nullptr);
            auto* equippedWeaponData = equipData->data ? static_cast<RE::EquippedWeaponData*>(equipData->data.get()) : nullptr;
            identity.equippedDataAddress = reinterpret_cast<std::uintptr_t>(equippedWeaponData);
            identity.equippedObjectAddress = reinterpret_cast<std::uintptr_t>(
                equippedWeaponData ? equippedWeaponData->fireNode : nullptr);
            const auto* objectInstanceExtra = findEquippedWeaponObjectInstanceExtra(player, weaponForm, instanceData);
            const auto objectInstanceWitness = makeObjectInstanceExtraWitness(objectInstanceExtra);
            identity.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(objectInstanceExtra);
            identity.objectIndexDataSignature = objectInstanceWitness.signature;
            identity.objectIndexDataCount = objectInstanceWitness.count;
            identity.activeModCount = objectInstanceWitness.activeCount;
            identity.disabledModCount = objectInstanceWitness.disabledCount;
            if (const auto* weapon = asEquippedWeaponForm(weaponForm)) {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(weapon, instanceData, objectInstanceExtra);
                /*
                 * FO4VR binary verification (2026-08-05): the TESObjectWEAP
                 * BGSEquipType-subobject override at 0x14033FD70 returns
                 * InstanceData::equipSlot (+0x70) whenever instanceData is
                 * non-null, otherwise BGSEquipType::equipSlot (+0x08). Keep
                 * both values for diagnostic provenance and classify authored
                 * grip behavior from the effective runtime value.
                 */
                const auto* baseEquipSlot = weapon->GetEquipSlot(nullptr);
                const auto* effectiveEquipSlot = weapon->GetEquipSlot(instanceData);
                identity.baseEquipSlotFormID =
                    baseEquipSlot ? baseEquipSlot->formID : 0;
                identity.effectiveEquipSlotFormID =
                    effectiveEquipSlot ? effectiveEquipSlot->formID : 0;
                identity.effectiveEquipSlotUsesInstanceData =
                    instanceData != nullptr;
                float weightGame = instanceData ? instanceData->GetWeight() : -1.0f;
                if (weightGame < 0.0f) {
                    weightGame = weapon->weaponData.weight;
                }
                identity.weightGame = std::isfinite(weightGame) && weightGame > 0.0f ? weightGame : 0.0f;
                const auto classification = classifyEquippedWeapon(weapon, weightGame);
                identity.sizeClass = classification.sizeClass;
                identity.classificationSource = classification.source;
                identity.keywordFlags = classification.keywordFlags;
            } else {
                identity.instanceContentKey = makeEquippedWeaponInstanceContentKey(nullptr, instanceData, objectInstanceExtra);
            }
            const auto fullName = RE::TESFullName::GetFullName(*weaponForm);
            if (!fullName.empty()) {
                identity.displayName = fullName;
            }
            return identity;
        }
        std::uintptr_t readRendererChildPointer(void* rendererData, std::ptrdiff_t rendererChildOffset)
        {
            if (!rendererData) {
                return 0;
            }
            auto* child = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + rendererChildOffset);
            if (!child) {
                return 0;
            }
            return reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(reinterpret_cast<char*>(child) + 0x08));
        }

        weapon_visual_composition_policy::VisualRecord makeWeaponVisualRecord(
            RE::NiAVObject* node,
            RE::NiAVObject* parent,
            std::uint32_t childIndex,
            std::uint32_t childCount,
            std::uint32_t depth,
            bool visible)
        {
            weapon_visual_composition_policy::VisualRecord record{
                .nodeAddress = reinterpret_cast<std::uintptr_t>(node),
                .parentAddress = reinterpret_cast<std::uintptr_t>(parent ? parent : node ? node->parent : nullptr),
                .name = safeNodeName(node),
                .depth = depth,
                .childIndex = childIndex,
                .childCount = childCount,
                .visible = visible,
                .triShape = node && node->IsTriShape(),
            };

            if (auto* triShape = node ? node->IsTriShape() : nullptr) {
                auto* base = reinterpret_cast<char*>(triShape);
                auto* rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
                record.rendererData = reinterpret_cast<std::uintptr_t>(rendererData);
                record.skinInstance = reinterpret_cast<std::uintptr_t>(*reinterpret_cast<void**>(base + VROffset::skinInstance));
                record.vertexDesc = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
                record.numTriangles = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
                record.numVertices = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
                record.geometryType = *reinterpret_cast<std::uint8_t*>(base + VROffset::geometryType);
                record.vertexBlock = readRendererChildPointer(rendererData, 0x08);
                record.triangleBlock = readRendererChildPointer(rendererData, 0x10);
            }

            return record;
        }

        void accumulateWeaponVisualKey(RE::NiAVObject* node, RE::NiAVObject* parent, std::uint32_t childIndex, int depth, std::uint64_t& key, WeaponVisualKeyStats& stats)
        {
            if (!node || depth > 15 || stats.nodeCount > 512) {
                return;
            }

            /*
             * Effect-only geometry cannot create collision and therefore must
             * not make flashlight/laser/reticle visibility toggle the weapon's
             * collision generation key. A billboard subtree is effect-only by
             * construction; other effect shapes are filtered by shader/name.
             */
            if (niObjectRttiChainContains(node, "NiBillboardNode")) {
                return;
            }
            if (auto* triShape = node->IsTriShape();
                triShape && classifyGeneratedWeaponEffectGeometry(triShape) != weapon_effect_geometry_policy::ExclusionReason::None) {
                return;
            }

            const bool visible = weaponVisualNodeVisible(node);
            ++stats.nodeCount;
            if (!visible) {
                ++stats.invisibleNodeCount;
            }

            std::uint32_t childCount = 0;
            if (auto* niNode = node->IsNode()) {
                childCount = static_cast<std::uint32_t>(niNode->GetRuntimeData().children.size());
            }
            const auto record = makeWeaponVisualRecord(node, parent, childIndex, childCount, static_cast<std::uint32_t>(depth), visible);
            weapon_visual_composition_policy::mixVisualRecord(key, record);

            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                if (visible) {
                    if (record.rendererData == 0 || record.vertexBlock == 0 || record.triangleBlock == 0) {
                        ++stats.missingRendererCount;
                    } else if (record.numTriangles == 0 || record.numVertices == 0) {
                        ++stats.emptyGeometryCount;
                    } else {
                        ++stats.visibleTriShapeCount;
                    }
                }
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            auto& kids = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size(); ++i) {
                auto* kid = kids[i].get();
                accumulateWeaponVisualKey(kid, node, i, depth + 1, key, stats);
            }
        }
    }

    std::uint64_t WeaponCollision::getEquippedWeaponIdentityKey(
        std::uint64_t* outIdentityKey,
        std::uint64_t* outOwnershipKey,
        WeaponSizeClass* outSizeClass,
        std::uint32_t* outFormID,
        std::uint64_t* outInstanceContentKey) const
    {
        const auto identity = readEquippedWeaponGenerationIdentity();
        const auto identityKey = weapon_generation_identity_policy::makeEquippedWeaponIdentityKey(identity);
        if (outIdentityKey) {
            *outIdentityKey = identityKey;
        }
        if (outOwnershipKey) {
            *outOwnershipKey = weapon_generation_identity_policy::makeEquippedWeaponOwnershipKey(identity);
        }
        if (outSizeClass) {
            *outSizeClass = identity.sizeClass;
        }
        if (outFormID) {
            *outFormID = identity.formID;
        }
        if (outInstanceContentKey) {
            *outInstanceContentKey = identity.instanceContentKey;
        }

        return identityKey;
    }

    weapon_generation_identity_policy::EquippedWeaponGenerationIdentity WeaponCollision::getEquippedWeaponClassification() const
    {
        return readEquippedWeaponGenerationIdentity();
    }

    std::uint64_t WeaponCollision::getWeaponVisualCompositionKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const
    {
        std::uint64_t visualKey = 0;
        if (weaponNode) {
            visualKey = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            const auto candidates = makeGeneratedWeaponMeshRootCandidates(weaponNode);
            for (const auto& candidate : candidates) {
                if (!candidate.root) {
                    continue;
                }

                ++stats.rootCount;
                mixWeaponVisualString(visualKey, candidate.label);
                mixWeaponVisualKey(visualKey, reinterpret_cast<std::uintptr_t>(candidate.root));
                accumulateWeaponVisualKey(candidate.root, nullptr, 0, 0, visualKey, stats);
            }

            if (visualKey == weapon_visual_composition_policy::kWeaponVisualCompositionOffset) {
                visualKey = reinterpret_cast<std::uint64_t>(weaponNode);
            }
        }
        return visualKey;
    }

    /*
     * Fill the scope-overlay fields of a sight anchor snapshot.
     *
     * The anchor itself is built from published collider evidence, but whether a
     * magnified optic is installed - and which native overlay index it uses - is an
     * equipped-OMOD question, so it is answered here rather than in the publication
     * path. Kept as a named seam so the body publisher never has to know how a
     * scope is recognized.
     */
    /*
     * Does this attachment mod drive the native scope overlay?
     *
     * BGSMod property block 1, target 48 is the overlay slot. Reading the property
     * buffer is the only reliable signal - names and model paths lie, because plenty
     * of non-magnifying sights are called "scope".
     */
    bool weapon_collision_detail::attachmentModHasNativeScopeOverlayTarget(std::uint32_t omodFormId)
    {
        constexpr std::uint8_t kBgsModPropertyBlockId = 1;
        constexpr std::uint32_t kNativeScopeOverlayTarget = 48;
        using PropertyMod = RE::BGSMod::Property::Mod;

        auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(omodFormId);
        if (!omod) {
            return false;
        }
        for (const auto& property : omod->GetBuffer<PropertyMod>(kBgsModPropertyBlockId)) {
            if (property.target == kNativeScopeOverlayTarget) {
                return true;
            }
        }
        return false;
    }

    void WeaponCollision::applyEquippedManualScopeTarget(
        RE::NiAVObject* packageDriveNode,
        NativeScopeSightAnchorSnapshot& outSnapshot)
    {
        const auto manualScopeTarget = resolveEquippedManualScopeTarget(packageDriveNode);
        outSnapshot.nativeScopeOverlayValid = manualScopeTarget.overlayValid;
        outSnapshot.nativeScopeOverlayIndex = manualScopeTarget.overlayIndex;
        outSnapshot.manualDirectTransitionRequired = manualScopeTarget.directTransitionRequired;
    }

}
