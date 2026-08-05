#include "physics-interaction/melee/MeleeTargetResolver.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundAnimObjects.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/bhkCharacterController.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <excpt.h>
#include <iterator>
#include <limits>

namespace rock::physical_melee
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        constexpr std::uint32_t anatomyFlag(provider::RockProviderTargetAnatomyFlagV1 flag)
        {
            return static_cast<std::uint32_t>(flag);
        }

        char foldAscii(char value)
        {
            return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
        }

        std::size_t normalizeNodeName(const char* source, char* destination, std::size_t capacity)
        {
            if (!source || !destination || capacity == 0) {
                return 0;
            }
            std::size_t written = 0;
            for (std::size_t i = 0; i < 255 && source[i] != '\0'; ++i) {
                const char folded = foldAscii(source[i]);
                if ((folded >= 'a' && folded <= 'z') || (folded >= '0' && folded <= '9')) {
                    if (written + 1 >= capacity) {
                        break;
                    }
                    destination[written++] = folded;
                }
            }
            destination[written] = '\0';
            return written;
        }

        std::uint64_t hashNormalizedNodeName(const char* normalizedName)
        {
            constexpr std::uint64_t offsetBasis = 14695981039346656037ull;
            constexpr std::uint64_t prime = 1099511628211ull;
            std::uint64_t hash = offsetBasis;
            if (!normalizedName) {
                return 0;
            }
            for (std::size_t i = 0; normalizedName[i] != '\0'; ++i) {
                hash ^= static_cast<std::uint8_t>(normalizedName[i]);
                hash *= prime;
            }
            return hash;
        }

        provider::RockProviderBodyZoneSide bodyPartSide(std::uint32_t partIndex)
        {
            using PartType = RE::BGSBodyPartData::PartType;
            switch (partIndex) {
            case PartType::LeftArm1:
            case PartType::LeftArm2:
            case PartType::LeftLeg1:
            case PartType::LeftLeg2:
            case PartType::LeftLeg3:
            case PartType::LeftFoot:
                return provider::RockProviderBodyZoneSide::Left;
            case PartType::RightArm1:
            case PartType::RightArm2:
            case PartType::RightLeg1:
            case PartType::RightLeg2:
            case PartType::RightLeg3:
            case PartType::RightFoot:
                return provider::RockProviderBodyZoneSide::Right;
            default:
                return provider::RockProviderBodyZoneSide::Center;
            }
        }

        provider::RockProviderBodyZoneKind bodyPartZone(std::uint32_t partIndex)
        {
            using PartType = RE::BGSBodyPartData::PartType;
            using Zone = provider::RockProviderBodyZoneKind;
            switch (partIndex) {
            case PartType::Torso: return Zone::Chest;
            case PartType::Head1:
            case PartType::Eye:
            case PartType::LookAt:
            case PartType::Head2:
            case PartType::Brain:
            case PartType::Camera:
            case PartType::FaceTargetSource:
                return Zone::NeckHead;
            case PartType::LeftArm1: return Zone::LeftUpperArm;
            case PartType::LeftArm2: return Zone::LeftForearmUpper;
            case PartType::RightArm1: return Zone::RightUpperArm;
            case PartType::RightArm2: return Zone::RightForearmUpper;
            case PartType::LeftLeg1: return Zone::LeftThigh;
            case PartType::LeftLeg2: return Zone::LeftCalf;
            case PartType::LeftLeg3:
            case PartType::LeftFoot:
                return Zone::LeftFoot;
            case PartType::RightLeg1: return Zone::RightThigh;
            case PartType::RightLeg2: return Zone::RightCalf;
            case PartType::RightLeg3:
            case PartType::RightFoot:
                return Zone::RightFoot;
            case PartType::Root:
            case PartType::COM:
            case PartType::Pelvis:
            case PartType::OffsetRoot:
                return Zone::Pelvis;
            default:
                return Zone::Unknown;
            }
        }

        bool eligibleForPointFallback(std::uint32_t partIndex)
        {
            using PartType = RE::BGSBodyPartData::PartType;
            switch (partIndex) {
            case PartType::Torso:
            case PartType::Head1:
            case PartType::Head2:
            case PartType::LeftArm1:
            case PartType::LeftArm2:
            case PartType::RightArm1:
            case PartType::RightArm2:
            case PartType::LeftLeg1:
            case PartType::LeftLeg2:
            case PartType::LeftLeg3:
            case PartType::LeftFoot:
            case PartType::RightLeg1:
            case PartType::RightLeg2:
            case PartType::RightLeg3:
            case PartType::RightFoot:
            case PartType::Pelvis:
                return true;
            default:
                return false;
            }
        }

        RE::bhkNPCollisionObject* resolveCollisionObject(RE::bhkWorld* bhkWorld, std::uint32_t bodyId) noexcept
        {
            if (!bhkWorld || bodyId == kInvalidBodyId) {
                return nullptr;
            }
#if defined(_MSC_VER)
            __try {
                return RE::bhkNPCollisionObject::Getbhk(
                    bhkWorld,
                    *reinterpret_cast<RE::hknpBodyId*>(&bodyId));
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
#else
            return RE::bhkNPCollisionObject::Getbhk(
                bhkWorld,
                *reinterpret_cast<RE::hknpBodyId*>(&bodyId));
#endif
        }

        RE::BGSBodyPartData* resolveBodyPartData(RE::Actor* actor, RE::TESNPC* baseNpc) noexcept
        {
            // The base NPC's TESRaceForm is the engine-owned form authority for
            // its BPTD.  Prefer it over Actor::race, which is mutable live
            // process state.  Keep the live race only as a bounded fallback for
            // actors whose base form does not expose a BPTD.
            auto* baseRace = baseNpc ? baseNpc->GetFormRace() : nullptr;
            auto* bodyPartData = baseRace ? baseRace->bodyPartData : nullptr;
            if (bodyPartData) {
                return bodyPartData;
            }
            auto* liveRace = actor ? actor->race : nullptr;
            return liveRace ? liveRace->bodyPartData : nullptr;
        }

        void clearAnatomy(TargetResolution& result) noexcept
        {
            result.anatomyFlags = 0;
            result.bodyPartIndex = 0xFFFF'FFFFu;
            result.bodyPart = nullptr;
            result.nativeDamageLimb = 0xFFFF'FFFFu;
            result.zone = provider::RockProviderBodyZoneKind::Unknown;
            result.side = provider::RockProviderBodyZoneSide::Center;
            result.bodyPartDamageMultiplier = 1.0f;
            result.limbActorValueFormId = 0;
            result.nodeNameHash = 0;
            result.nodeName[0] = '\0';
        }
    }

    bool TargetResolution::anatomyValid() const noexcept
    {
        return (anatomyFlags & anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartValid)) != 0 &&
               (anatomyFlags & anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartAmbiguous)) == 0 &&
               bodyPartIndex < 26;
    }

    const char* targetResolutionStageName(TargetResolutionStage stage) noexcept
    {
        switch (stage) {
        case TargetResolutionStage::NotStarted: return "NotStarted";
        case TargetResolutionStage::CollisionObjectResolved: return "CollisionObjectResolved";
        case TargetResolutionStage::SceneObjectResolved: return "SceneObjectResolved";
        case TargetResolutionStage::ReferenceResolved: return "ReferenceResolved";
        case TargetResolutionStage::LiveActorResolved: return "LiveActorResolved";
        case TargetResolutionStage::BodyPartDataResolved: return "BodyPartDataResolved";
        case TargetResolutionStage::AnatomyResolved: return "AnatomyResolved";
        case TargetResolutionStage::RegisteredBodyResolved: return "RegisteredBodyResolved";
        default: return "Unknown";
        }
    }

    TargetResolution resolveTarget(
        RE::bhkWorld* bhkWorld,
        std::uint32_t targetBodyId,
        const float* measuredContactPointHavok,
        float havokToGameScale) noexcept
    {
        TargetResolution result{};
        result.bodyId = targetBodyId;
        result.measuredContactPointValid = measuredContactPointHavok &&
            std::isfinite(havokToGameScale) && havokToGameScale > 0.0f &&
            std::isfinite(measuredContactPointHavok[0]) &&
            std::isfinite(measuredContactPointHavok[1]) &&
            std::isfinite(measuredContactPointHavok[2]);
        if (result.measuredContactPointValid) {
            result.measuredContactPointGame[0] = measuredContactPointHavok[0] * havokToGameScale;
            result.measuredContactPointGame[1] = measuredContactPointHavok[1] * havokToGameScale;
            result.measuredContactPointGame[2] = measuredContactPointHavok[2] * havokToGameScale;
        }
        auto* collisionObject = resolveCollisionObject(bhkWorld, targetBodyId);
        if (!collisionObject) {
            return result;
        }
        result.stage = TargetResolutionStage::CollisionObjectResolved;

#if defined(_MSC_VER)
        __try {
#endif
            auto* sceneObject = collisionObject->sceneObject;
            if (!sceneObject) {
                return result;
            }
            result.stage = TargetResolutionStage::SceneObjectResolved;
            auto* reference = RE::TESObjectREFR::FindReferenceFor3D(sceneObject);
            auto* base = reference ? reference->GetObjectReference() : nullptr;
            result.referenceIsPlayer = reference == RE::PlayerCharacter::GetSingleton();
            if (!reference || !base || !base->Is(RE::ENUM_FORM_ID::kNPC_) ||
                result.referenceIsPlayer || reference->IsDeleted() ||
                reference->IsDisabled()) {
                return result;
            }
            result.stage = TargetResolutionStage::ReferenceResolved;
            auto* actor = static_cast<RE::Actor*>(reference);
            result.referenceFormId = actor->GetFormID();
            result.referenceIsDead = actor->IsDead(false);
            if (result.referenceIsDead) {
                return result;
            }
            result.actor = actor;
            result.actorFormId = result.referenceFormId;
            result.stage = TargetResolutionStage::LiveActorResolved;

            const char* sourceName = sceneObject->name.c_str();
            char normalizedSource[256]{};
            if (sourceName && sourceName[0] != '\0' &&
                normalizeNodeName(sourceName, normalizedSource, std::size(normalizedSource)) != 0) {
                const auto copied = (std::min)(
                    std::strlen(sourceName),
                    static_cast<std::size_t>(provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1));
                std::memcpy(result.nodeName, sourceName, copied);
                result.nodeName[copied] = '\0';
                result.nodeNameHash = hashNormalizedNodeName(normalizedSource);
                result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::NodeNameValid);
            }

            auto* bodyPartData = resolveBodyPartData(actor, static_cast<RE::TESNPC*>(base));
            if (!bodyPartData) {
                return result;
            }
            result.stage = TargetResolutionStage::BodyPartDataResolved;
            std::uint32_t matchedIndex = 0xFFFF'FFFFu;
            std::uint32_t matchCount = 0;
            for (std::uint32_t i = 0; i < std::size(bodyPartData->partArray); ++i) {
                auto* part = bodyPartData->partArray[i];
                if (!part) {
                    continue;
                }
                char normalizedNode[256]{};
                char normalizedTarget[256]{};
                (void)normalizeNodeName(part->nodeName.c_str(), normalizedNode, std::size(normalizedNode));
                (void)normalizeNodeName(part->targetName.c_str(), normalizedTarget, std::size(normalizedTarget));
                if (normalizedSource[0] != '\0' &&
                    (std::strcmp(normalizedSource, normalizedNode) == 0 ||
                        std::strcmp(normalizedSource, normalizedTarget) == 0)) {
                    matchedIndex = i;
                    ++matchCount;
                }
            }
            result.directMatchCount = matchCount;

            const char* fallbackNodeName = nullptr;
            if (matchCount == 0 && result.measuredContactPointValid) {
                auto* actorRoot = actor->Get3D();
                const RE::NiPoint3 pointGame{
                    result.measuredContactPointGame[0],
                    result.measuredContactPointGame[1],
                    result.measuredContactPointGame[2],
                };
                float bestDistanceSquared = (std::numeric_limits<float>::max)();
                float runnerUpDistanceSquared = (std::numeric_limits<float>::max)();
                for (std::uint32_t i = 0; actorRoot && i < std::size(bodyPartData->partArray); ++i) {
                    auto* part = bodyPartData->partArray[i];
                    if (!part || !eligibleForPointFallback(i)) {
                        continue;
                    }

                    float partDistanceSquared = (std::numeric_limits<float>::max)();
                    const char* partNodeName = nullptr;
                    const char* candidateNames[]{ part->nodeName.c_str(), part->targetName.c_str() };
                    for (const auto* candidateName : candidateNames) {
                        if (!candidateName || candidateName[0] == '\0') {
                            continue;
                        }
                        auto* node = f4vr::findNode(actorRoot, candidateName);
                        if (!node) {
                            continue;
                        }
                        const float dx = node->world.translate.x - pointGame.x;
                        const float dy = node->world.translate.y - pointGame.y;
                        const float dz = node->world.translate.z - pointGame.z;
                        const float distanceSquared = dx * dx + dy * dy + dz * dz;
                        if (std::isfinite(distanceSquared) && distanceSquared < partDistanceSquared) {
                            partDistanceSquared = distanceSquared;
                            partNodeName = candidateName;
                        }
                    }
                    if (!partNodeName) {
                        continue;
                    }
                    ++result.fallbackCandidateCount;

                    if (partDistanceSquared + 1.0f < bestDistanceSquared) {
                        runnerUpDistanceSquared = bestDistanceSquared;
                        bestDistanceSquared = partDistanceSquared;
                        matchedIndex = i;
                        matchCount = 1;
                        fallbackNodeName = partNodeName;
                    } else if (std::fabs(partDistanceSquared - bestDistanceSquared) <= 1.0f) {
                        matchCount = 2;
                    }
                    if (partDistanceSquared > bestDistanceSquared &&
                        partDistanceSquared < runnerUpDistanceSquared) {
                        runnerUpDistanceSquared = partDistanceSquared;
                    }
                }
                if (std::isfinite(bestDistanceSquared) &&
                    bestDistanceSquared < (std::numeric_limits<float>::max)()) {
                    result.selectedNodeDistanceGame = std::sqrt(bestDistanceSquared);
                }
                if (std::isfinite(runnerUpDistanceSquared) &&
                    runnerUpDistanceSquared < (std::numeric_limits<float>::max)()) {
                    result.runnerUpNodeDistanceGame = std::sqrt(runnerUpDistanceSquared);
                }
            }
            if (matchCount != 1 || matchedIndex >= std::size(bodyPartData->partArray)) {
                if (matchCount > 1) {
                    result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartAmbiguous);
                }
                return result;
            }

            auto* part = bodyPartData->partArray[matchedIndex];
            if (fallbackNodeName) {
                result.pointFallbackUsed = true;
                const auto copied = (std::min)(
                    std::strlen(fallbackNodeName),
                    static_cast<std::size_t>(provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1));
                std::memcpy(result.nodeName, fallbackNodeName, copied);
                result.nodeName[copied] = '\0';
                char normalizedFallback[256]{};
                (void)normalizeNodeName(
                    fallbackNodeName,
                    normalizedFallback,
                    std::size(normalizedFallback));
                result.nodeNameHash = hashNormalizedNodeName(normalizedFallback);
                result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::NodeNameValid);
            }
            result.bodyPartIndex = matchedIndex;
            result.bodyPart = part;
            result.nativeDamageLimb = static_cast<std::uint32_t>(part->data.type);
            result.zone = bodyPartZone(matchedIndex);
            result.side = bodyPartSide(matchedIndex);
            result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::BodyPartValid);
            if (std::isfinite(part->data.damageMult) && part->data.damageMult >= 0.0f) {
                result.bodyPartDamageMultiplier = part->data.damageMult;
                result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::DamageMultiplierValid);
            }
            if (part->data.actorValue) {
                result.limbActorValueFormId = part->data.actorValue->GetFormID();
                if (result.limbActorValueFormId != 0) {
                    result.anatomyFlags |= anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::LimbActorValueValid);
                }
            }
            result.stage = TargetResolutionStage::AnatomyResolved;
#if defined(_MSC_VER)
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            // Identity is already useful evidence and may have been resolved
            // before a malformed/stale anatomy pointer faulted.  Do not erase
            // it and misreport the failure as MissingTargetActor.
            result.accessViolation = true;
            clearAnatomy(result);
        }
#endif
        return result;
    }

    TargetResolution resolveRegisteredTarget(
        const provider::RockProviderExternalBodyRegistration& registration) noexcept
    {
        TargetResolution result{};
        result.bodyId = registration.bodyId;
        result.registeredBodyEvidence = true;
        if (registration.size != sizeof(registration) ||
            registration.role != provider::RockProviderExternalBodyRole::ActorRagdollBone ||
            registration.actorFormId == 0 || registration.bodyPartIndex >= 26u) {
            return result;
        }

        const auto bodyPartValid = anatomyFlag(
            provider::RockProviderTargetAnatomyFlagV1::BodyPartValid);
        const auto bodyPartAmbiguous = anatomyFlag(
            provider::RockProviderTargetAnatomyFlagV1::BodyPartAmbiguous);
        if ((registration.anatomyFlags & bodyPartValid) == 0 ||
            (registration.anatomyFlags & bodyPartAmbiguous) != 0) {
            return result;
        }

#if defined(_MSC_VER)
        __try {
#endif
            auto* actor = RE::TESForm::GetFormByID<RE::Actor>(registration.actorFormId);
            auto* base = actor ? actor->GetObjectReference() : nullptr;
            if (!actor || !base || !base->Is(RE::ENUM_FORM_ID::kNPC_) ||
                actor == RE::PlayerCharacter::GetSingleton() || actor->IsDeleted() ||
                actor->IsDisabled()) {
                return result;
            }
            result.referenceFormId = actor->GetFormID();
            result.referenceIsDead = actor->IsDead(false);
            if (result.referenceFormId != registration.actorFormId || result.referenceIsDead) {
                return result;
            }
            result.actor = actor;
            result.actorFormId = result.referenceFormId;
            result.stage = TargetResolutionStage::LiveActorResolved;

            auto* bodyPartData = resolveBodyPartData(actor, static_cast<RE::TESNPC*>(base));
            if (!bodyPartData || registration.bodyPartIndex >= std::size(bodyPartData->partArray)) {
                return result;
            }
            result.stage = TargetResolutionStage::BodyPartDataResolved;
            auto* part = bodyPartData->partArray[registration.bodyPartIndex];
            if (!part) {
                return result;
            }

            if ((registration.anatomyFlags & anatomyFlag(
                    provider::RockProviderTargetAnatomyFlagV1::NodeNameValid)) != 0) {
                char normalizedRegistration[256]{};
                char normalizedNode[256]{};
                char normalizedTarget[256]{};
                (void)normalizeNodeName(
                    registration.nodeName,
                    normalizedRegistration,
                    std::size(normalizedRegistration));
                (void)normalizeNodeName(
                    part->nodeName.c_str(),
                    normalizedNode,
                    std::size(normalizedNode));
                (void)normalizeNodeName(
                    part->targetName.c_str(),
                    normalizedTarget,
                    std::size(normalizedTarget));
                if (normalizedRegistration[0] == '\0' ||
                    (std::strcmp(normalizedRegistration, normalizedNode) != 0 &&
                        std::strcmp(normalizedRegistration, normalizedTarget) != 0)) {
                    return result;
                }
                const auto normalizedHash = hashNormalizedNodeName(normalizedRegistration);
                if (registration.nodeNameHash != 0 &&
                    registration.nodeNameHash != normalizedHash) {
                    return result;
                }
                const auto copied = (std::min)(
                    std::strlen(registration.nodeName),
                    static_cast<std::size_t>(provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1));
                std::memcpy(result.nodeName, registration.nodeName, copied);
                result.nodeName[copied] = '\0';
                result.nodeNameHash = normalizedHash;
                result.anatomyFlags |= anatomyFlag(
                    provider::RockProviderTargetAnatomyFlagV1::NodeNameValid);
            } else {
                const auto* nodeName = part->nodeName.c_str();
                char normalizedNode[256]{};
                if (nodeName && nodeName[0] != '\0' &&
                    normalizeNodeName(nodeName, normalizedNode, std::size(normalizedNode)) != 0) {
                    const auto copied = (std::min)(
                        std::strlen(nodeName),
                        static_cast<std::size_t>(provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1));
                    std::memcpy(result.nodeName, nodeName, copied);
                    result.nodeName[copied] = '\0';
                    result.nodeNameHash = hashNormalizedNodeName(normalizedNode);
                    result.anatomyFlags |= anatomyFlag(
                        provider::RockProviderTargetAnatomyFlagV1::NodeNameValid);
                }
            }

            result.anatomyFlags |= registration.anatomyFlags;
            result.bodyPartIndex = registration.bodyPartIndex;
            result.bodyPart = part;
            result.nativeDamageLimb = static_cast<std::uint32_t>(part->data.type);
            result.zone = bodyPartZone(result.bodyPartIndex);
            result.side = bodyPartSide(result.bodyPartIndex);
            result.bodyPartDamageMultiplier = 1.0f;
            result.limbActorValueFormId = 0;
            result.anatomyFlags &= ~(
                anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::DamageMultiplierValid) |
                anatomyFlag(provider::RockProviderTargetAnatomyFlagV1::LimbActorValueValid));
            if (std::isfinite(part->data.damageMult) && part->data.damageMult >= 0.0f) {
                result.bodyPartDamageMultiplier = part->data.damageMult;
                result.anatomyFlags |= anatomyFlag(
                    provider::RockProviderTargetAnatomyFlagV1::DamageMultiplierValid);
            }
            if (part->data.actorValue) {
                result.limbActorValueFormId = part->data.actorValue->GetFormID();
                if (result.limbActorValueFormId != 0) {
                    result.anatomyFlags |= anatomyFlag(
                        provider::RockProviderTargetAnatomyFlagV1::LimbActorValueValid);
                }
            }
            result.stage = TargetResolutionStage::RegisteredBodyResolved;
#if defined(_MSC_VER)
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            result.accessViolation = true;
            clearAnatomy(result);
        }
#endif
        return result;
    }
}
