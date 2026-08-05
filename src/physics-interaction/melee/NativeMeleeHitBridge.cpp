#include "physics-interaction/melee/NativeMeleeHitBridge.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/bhkCharacterController.h"

#include <xbyak/xbyak.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <excpt.h>
#include <mutex>

namespace rock::physical_melee
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
        constexpr std::uintptr_t kHitDataCtorOffset = 0x1042460;
        constexpr std::uintptr_t kHitDataDtorOffset = 0x1042530;
        constexpr std::uintptr_t kInitializeWeaponHitDataOffset = 0x1042950;
        constexpr std::uintptr_t kGetEquippedWeaponOffset = 0x0E5F8B0;
        constexpr std::uintptr_t kActorHitMeOffset = 0x0E51760;
        constexpr std::uintptr_t kPreLimbCalculationHookOffset = 0x1045913;
        constexpr std::uintptr_t kPreLimbCalculationReturnOffset = 0x104591B;
        constexpr std::uintptr_t kImpactScalarHookOffset = 0x1042C7B;
        constexpr std::uintptr_t kImpactScalarReturnOffset = 0x1042C82;
        constexpr std::uintptr_t kActorHitDispatcherCallsiteOffset = 0x0DB176D;
        constexpr std::uintptr_t kActorHitDispatcherFunctionOffset = 0x0E526D0;
        constexpr std::uintptr_t kActorValueMutationCallsiteOffset = 0x0E4BBA0;
        constexpr std::uintptr_t kActorValueMutationFunctionOffset = 0x0E483F0;
        constexpr std::uint64_t kUnknownOutcomeAfterFrames = 300;
        constexpr std::size_t kPendingCapacity = 128;
        constexpr std::size_t kCompletedCapacity = 256;
        constexpr std::size_t kInjectionDepth = 8;
        constexpr std::size_t kDispatchDepth = 8;

        constexpr std::array<std::uint8_t, 8> kPreLimbCalculationBytes{
            0x4C, 0x8B, 0xA4, 0x24, 0xD0, 0x02, 0x00, 0x00
        };
        constexpr std::array<std::uint8_t, 7> kImpactScalarBytes{ 0x48, 0x8B, 0x47, 0x60, 0x48, 0x85, 0xC0 };
        constexpr std::array<std::uint8_t, 5> kActorHitDispatcherCallsiteBytes{ 0xE8, 0x5E, 0x0F, 0x0A, 0x00 };
        constexpr std::array<std::uint8_t, 5> kActorValueMutationCallsiteBytes{ 0xE8, 0x4B, 0xC8, 0xFF, 0xFF };

        using HitDataCtor_t = void (*)(RE::HitData*);
        using HitDataDtor_t = void (*)(RE::HitData*);
        using InitializeWeaponHitData_t = void (*)(
            RE::HitData*,
            RE::TESObjectREFR*,
            RE::TESObjectREFR*,
            RE::BGSObjectInstanceT<RE::TESObjectWEAP>*,
            std::uint32_t,
            bool);
        using GetEquippedWeapon_t = RE::BGSObjectInstanceT<RE::TESObjectWEAP>* (*)(
            RE::Actor*,
            RE::BGSObjectInstanceT<RE::TESObjectWEAP>*,
            std::uint32_t);
        using ActorHitMe_t = void (*)(RE::Actor*, RE::HitData&);
        using ActorHitDispatcher_t = void (*)(RE::Actor*, RE::HitData*);
        using ActorValueMutation_t = void (*)(RE::Actor*, RE::ActorValueInfo*, float, float, RE::Actor*);

        struct HitInjectionContext
        {
            RE::HitData* hitData{ nullptr };
            RE::Actor* target{ nullptr };
            RE::BGSBodyPart* bodyPart{ nullptr };
            RE::bhkNPCollisionObject* targetCollisionObject{ nullptr };
            float pointGame[3]{};
            float normal[3]{};
            float relativeVelocityGame[3]{};
            float multiplier{ 1.0f };
            bool applied{ false };
        };

        struct PendingImpact
        {
            bool occupied{ false };
            RE::Actor* target{ nullptr };
            std::uint32_t sourceWeaponFormId{ 0 };
            std::uint32_t nativeDamageLimb{ 0xFFFF'FFFFu };
            std::uint32_t limbActorValueFormId{ 0 };
            std::uint32_t submitThreadId{ 0 };
            float pointGame[3]{};
            rock::provider::RockProviderImpactOutcomeV1 outcome{};
            std::uint64_t bridgeEpoch{ 0 };
        };

        struct DispatchObservation
        {
            PendingImpact pending{};
            float healthDelta{ 0.0f };
            float limbDelta{ 0.0f };
            std::uint32_t limbMutationCount{ 0 };
        };

        thread_local std::array<HitInjectionContext*, kInjectionDepth> s_injectionStack{};
        thread_local std::uint32_t s_injectionDepth = 0;
        thread_local std::array<DispatchObservation*, kDispatchDepth> s_dispatchStack{};
        thread_local std::uint32_t s_dispatchDepth = 0;

        std::atomic_bool s_bridgeInstalled{ false };
        std::atomic<std::uintptr_t> s_originalActorHitDispatcher{ 0 };
        std::atomic<std::uintptr_t> s_originalActorValueMutation{ 0 };
        std::atomic<std::uint64_t> s_bridgeEpoch{ 1 };

        std::mutex s_outcomeMutex;
        std::array<PendingImpact, kPendingCapacity> s_pendingImpacts{};
        std::array<rock::provider::RockProviderImpactOutcomeV1, kCompletedCapacity> s_completedOutcomes{};
        std::size_t s_completedHead = 0;
        std::size_t s_completedCount = 0;

        [[nodiscard]] bool isFinite3(const float* value)
        {
            return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
        }

        [[nodiscard]] float length3(const float* value)
        {
            return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
        }

        [[nodiscard]] std::uint32_t currentThreadId()
        {
#if defined(_MSC_VER)
            return ::GetCurrentThreadId();
#else
            return 0;
#endif
        }

        [[nodiscard]] HitInjectionContext* currentInjection()
        {
            return s_injectionDepth > 0 ? s_injectionStack[s_injectionDepth - 1] : nullptr;
        }

        [[nodiscard]] DispatchObservation* currentDispatchObservation()
        {
            return s_dispatchDepth > 0 ? s_dispatchStack[s_dispatchDepth - 1] : nullptr;
        }

        void queueCompletedOutcomeLocked(const rock::provider::RockProviderImpactOutcomeV1& outcome)
        {
            if (s_completedCount < s_completedOutcomes.size()) {
                const auto index = (s_completedHead + s_completedCount) % s_completedOutcomes.size();
                s_completedOutcomes[index] = outcome;
                ++s_completedCount;
            } else {
                s_completedOutcomes[s_completedHead] = outcome;
                s_completedHead = (s_completedHead + 1) % s_completedOutcomes.size();
            }
        }

        [[nodiscard]] bool insertPendingImpact(const PendingImpact& pending)
        {
            std::scoped_lock lock(s_outcomeMutex);
            if (pending.bridgeEpoch != s_bridgeEpoch.load(std::memory_order_acquire)) {
                return false;
            }
            for (auto& slot : s_pendingImpacts) {
                if (!slot.occupied) {
                    slot = pending;
                    slot.occupied = true;
                    return true;
                }
            }
            return false;
        }

        void removePendingImpact(std::uint64_t impactId)
        {
            std::scoped_lock lock(s_outcomeMutex);
            for (auto& slot : s_pendingImpacts) {
                if (slot.occupied && slot.outcome.impactId == impactId) {
                    slot = {};
                    return;
                }
            }
        }

        [[nodiscard]] float squaredDistance3(const float* lhs, const RE::NiPoint3A& rhs)
        {
            const float dx = lhs[0] - rhs.x;
            const float dy = lhs[1] - rhs.y;
            const float dz = lhs[2] - rhs.z;
            return dx * dx + dy * dy + dz * dz;
        }

        [[nodiscard]] bool takeMatchingPendingImpact(RE::Actor* target, const RE::HitData* hitData, PendingImpact& outPending)
        {
            if (!target || !hitData) {
                return false;
            }

            const auto weaponFormId = hitData->weapon.object ? hitData->weapon.object->GetFormID() : 0;
            const auto bodyPartIndex = *reinterpret_cast<const std::uint32_t*>(
                reinterpret_cast<const std::byte*>(hitData) + 0xD0);
            constexpr float kMaximumPointDistanceSquared = 16.0f;

            std::scoped_lock lock(s_outcomeMutex);
            PendingImpact* best = nullptr;
            float bestDistance = kMaximumPointDistanceSquared;
            for (auto& slot : s_pendingImpacts) {
                if (!slot.occupied || slot.bridgeEpoch != s_bridgeEpoch.load(std::memory_order_acquire) ||
                    slot.target != target || slot.sourceWeaponFormId != weaponFormId ||
                    slot.nativeDamageLimb != bodyPartIndex) {
                    continue;
                }
                const float distance = squaredDistance3(slot.pointGame, hitData->impactData.location);
                if (std::isfinite(distance) && distance <= bestDistance) {
                    best = &slot;
                    bestDistance = distance;
                }
            }
            if (!best) {
                return false;
            }
            outPending = *best;
            *best = {};
            return true;
        }

        void finalizeDispatchObservation(DispatchObservation& observation)
        {
            if (observation.pending.bridgeEpoch != s_bridgeEpoch.load(std::memory_order_acquire)) {
                return;
            }
            auto outcome = observation.pending.outcome;
            outcome.lifecycle =
                (std::fabs(observation.healthDelta) > 0.000001f || std::fabs(observation.limbDelta) > 0.000001f) ?
                    rock::provider::RockProviderImpactOutcomeLifecycleV1::Applied :
                    rock::provider::RockProviderImpactOutcomeLifecycleV1::CompletedWithoutMutation;
            outcome.flags |= static_cast<std::uint32_t>(
                observation.pending.submitThreadId == currentThreadId() ?
                    rock::provider::RockProviderImpactOutcomeFlagV1::SynchronousConsumer :
                    rock::provider::RockProviderImpactOutcomeFlagV1::QueuedConsumer);
            outcome.observedHealthComponentDelta = observation.healthDelta;
            outcome.observedLimbComponentDelta = observation.limbDelta;
            if (std::fabs(observation.healthDelta) > 0.000001f) {
                outcome.flags |= static_cast<std::uint32_t>(rock::provider::RockProviderImpactOutcomeFlagV1::HealthMutationObserved);
            }
            if (std::fabs(observation.limbDelta) > 0.000001f) {
                outcome.flags |= static_cast<std::uint32_t>(rock::provider::RockProviderImpactOutcomeFlagV1::LimbMutationObserved);
            }
            if (observation.limbMutationCount > 1) {
                outcome.flags |= static_cast<std::uint32_t>(rock::provider::RockProviderImpactOutcomeFlagV1::MultipleLimbMutations);
            }
            if (std::fabs(observation.healthDelta) > 0.000001f || std::fabs(observation.limbDelta) > 0.000001f) {
                outcome.flags |= static_cast<std::uint32_t>(rock::provider::RockProviderImpactOutcomeFlagV1::ExactComponentDelta);
            }

            std::scoped_lock lock(s_outcomeMutex);
            if (observation.pending.bridgeEpoch != s_bridgeEpoch.load(std::memory_order_acquire)) {
                return;
            }
            queueCompletedOutcomeLocked(outcome);
        }

        void queueAbnormalDispatchOutcome(const PendingImpact& pending)
        {
            auto outcome = pending.outcome;
            outcome.lifecycle = rock::provider::RockProviderImpactOutcomeLifecycleV1::UnknownAfterSubmission;
            outcome.flags |= static_cast<std::uint32_t>(
                rock::provider::RockProviderImpactOutcomeFlagV1::SynchronousConsumer);
            std::scoped_lock lock(s_outcomeMutex);
            if (pending.bridgeEpoch != s_bridgeEpoch.load(std::memory_order_acquire)) {
                return;
            }
            queueCompletedOutcomeLocked(outcome);
        }

        float scaleNativeImpactScalar(RE::HitData* hitData, float nativeScalar)
        {
            auto* injection = currentInjection();
            if (!injection || injection->hitData != hitData || !std::isfinite(nativeScalar) ||
                !std::isfinite(injection->multiplier) || injection->multiplier < 0.0f) {
                return nativeScalar;
            }
            return nativeScalar * injection->multiplier;
        }

        RE::BGSBodyPart* injectPreLimbCalculation(
            RE::HitData* hitData,
            RE::Actor* target,
            RE::BGSBodyPart* nativeBodyPart)
        {
            auto* injection = currentInjection();
            if (!injection || injection->hitData != hitData || injection->target != target ||
                !hitData || !injection->bodyPart || !injection->targetCollisionObject) {
                return nativeBodyPart;
            }
            hitData->impactData.location.x = injection->pointGame[0];
            hitData->impactData.location.y = injection->pointGame[1];
            hitData->impactData.location.z = injection->pointGame[2];
            hitData->impactData.normal.x = injection->normal[0];
            hitData->impactData.normal.y = injection->normal[1];
            hitData->impactData.normal.z = injection->normal[2];
            hitData->impactData.velocity.x = injection->relativeVelocityGame[0];
            hitData->impactData.velocity.y = injection->relativeVelocityGame[1];
            hitData->impactData.velocity.z = injection->relativeVelocityGame[2];
            hitData->impactData.colObj.reset(injection->targetCollisionObject);
            // The native selector immediately writes bodyPart->data.type here.
            // Clearing the sentinel makes that write authoritative and keeps
            // HitData::damageLimb in the engine's LIMB_ENUM domain.
            *reinterpret_cast<std::uint32_t*>(reinterpret_cast<std::byte*>(hitData) + 0xD0) = 0xFFFF'FFFFu;
            injection->applied = true;
            return injection->bodyPart;
        }

#if defined(_MSC_VER)
        RE::BGSBodyPart* injectPreLimbCalculationSafely(
            RE::HitData* hitData,
            RE::Actor* target,
            RE::BGSBodyPart* nativeBodyPart)
        {
            __try {
                return injectPreLimbCalculation(hitData, target, nativeBodyPart);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nativeBodyPart;
            }
        }
#else
        RE::BGSBodyPart* injectPreLimbCalculationSafely(
            RE::HitData* hitData,
            RE::Actor* target,
            RE::BGSBodyPart* nativeBodyPart)
        {
            return injectPreLimbCalculation(hitData, target, nativeBodyPart);
        }
#endif

#if defined(_MSC_VER)
        bool callActorHitDispatcherWithFinally(
            ActorHitDispatcher_t original,
            RE::Actor* target,
            RE::HitData* hitData,
            DispatchObservation* observation)
        {
            bool completed = false;
            __try {
                original(target, hitData);
                completed = true;
            } __finally {
                --s_dispatchDepth;
                s_dispatchStack[s_dispatchDepth] = nullptr;
                if (!completed) {
                    queueAbnormalDispatchOutcome(observation->pending);
                }
            }
            return completed;
        }
#endif

        void actorHitDispatcherHook(RE::Actor* target, RE::HitData* hitData)
        {
            const auto original = reinterpret_cast<ActorHitDispatcher_t>(
                s_originalActorHitDispatcher.load(std::memory_order_acquire));
            if (!original) {
                return;
            }

            PendingImpact pending{};
            if (s_dispatchDepth >= s_dispatchStack.size() ||
                !takeMatchingPendingImpact(target, hitData, pending)) {
                original(target, hitData);
                return;
            }

            DispatchObservation observation{};
            observation.pending = pending;
            s_dispatchStack[s_dispatchDepth++] = &observation;
#if defined(_MSC_VER)
            const bool completed = callActorHitDispatcherWithFinally(original, target, hitData, &observation);
            if (completed) {
                finalizeDispatchObservation(observation);
            }
#else
            original(target, hitData);
            --s_dispatchDepth;
            s_dispatchStack[s_dispatchDepth] = nullptr;
            finalizeDispatchObservation(observation);
#endif
        }

        void actorValueMutationHook(
            RE::Actor* actor,
            RE::ActorValueInfo* actorValue,
            float oldTotal,
            float componentDelta,
            RE::Actor* source)
        {
            const auto original = reinterpret_cast<ActorValueMutation_t>(
                s_originalActorValueMutation.load(std::memory_order_acquire));
            if (original) {
                original(actor, actorValue, oldTotal, componentDelta, source);
            }

            auto* observation = currentDispatchObservation();
            if (!observation || actor != observation->pending.target || !actorValue || !std::isfinite(componentDelta)) {
                return;
            }

            auto* actorValues = RE::ActorValue::GetSingleton();
            if (actorValues && actorValue == actorValues->health) {
                observation->healthDelta += componentDelta;
                return;
            }
            if (observation->pending.limbActorValueFormId != 0 &&
                actorValue->GetFormID() == observation->pending.limbActorValueFormId) {
                observation->limbDelta += componentDelta;
                ++observation->limbMutationCount;
            }
        }

        class PreLimbCalculationHookCode final : public Xbyak::CodeGenerator
        {
        public:
            PreLimbCalculationHookCode(std::uintptr_t helper, std::uintptr_t returnAddress)
            {
                // Original instruction displaced at Fallout4VR.exe+0x1045913.
                mov(r12, ptr[rsp + 0x2D0]);
                pushfq();
                push(rax);
                push(rcx);
                push(rdx);
                push(r8);
                push(r9);
                push(r10);
                push(r11);
                sub(rsp, 0x80);
                movdqu(ptr[rsp + 0x20], xmm0);
                movdqu(ptr[rsp + 0x30], xmm1);
                movdqu(ptr[rsp + 0x40], xmm2);
                movdqu(ptr[rsp + 0x50], xmm3);
                movdqu(ptr[rsp + 0x60], xmm4);
                movdqu(ptr[rsp + 0x70], xmm5);
                mov(rcx, rsi);
                mov(rdx, rbx);
                mov(r8, r14);
                mov(rax, helper);
                call(rax);
                mov(r14, rax);
                movdqu(xmm0, ptr[rsp + 0x20]);
                movdqu(xmm1, ptr[rsp + 0x30]);
                movdqu(xmm2, ptr[rsp + 0x40]);
                movdqu(xmm3, ptr[rsp + 0x50]);
                movdqu(xmm4, ptr[rsp + 0x60]);
                movdqu(xmm5, ptr[rsp + 0x70]);
                add(rsp, 0x80);
                pop(r11);
                pop(r10);
                pop(r9);
                pop(r8);
                pop(rdx);
                pop(rcx);
                pop(rax);
                popfq();
                jmp(ptr[rip]);
                dq(returnAddress);
            }
        };

        class ImpactScalarHookCode final : public Xbyak::CodeGenerator
        {
        public:
            ImpactScalarHookCode(std::uintptr_t helper, std::uintptr_t returnAddress)
            {
                pushfq();
                push(rax);
                push(rcx);
                push(rdx);
                push(r8);
                push(r9);
                push(r10);
                push(r11);
                sub(rsp, 0x80);
                movdqu(ptr[rsp + 0x20], xmm0);
                movdqu(ptr[rsp + 0x30], xmm1);
                movdqu(ptr[rsp + 0x40], xmm2);
                movdqu(ptr[rsp + 0x50], xmm3);
                movdqu(ptr[rsp + 0x60], xmm4);
                movdqu(ptr[rsp + 0x70], xmm5);
                mov(rcx, rdi);
                movaps(xmm1, xmm11);
                mov(rax, helper);
                call(rax);
                movaps(xmm11, xmm0);
                movdqu(xmm0, ptr[rsp + 0x20]);
                movdqu(xmm1, ptr[rsp + 0x30]);
                movdqu(xmm2, ptr[rsp + 0x40]);
                movdqu(xmm3, ptr[rsp + 0x50]);
                movdqu(xmm4, ptr[rsp + 0x60]);
                movdqu(xmm5, ptr[rsp + 0x70]);
                add(rsp, 0x80);
                pop(r11);
                pop(r10);
                pop(r9);
                pop(r8);
                pop(rdx);
                pop(rcx);
                pop(rax);
                popfq();
                mov(rax, ptr[rdi + 0x60]);
                test(rax, rax);
                jmp(ptr[rip]);
                dq(returnAddress);
            }
        };

        [[nodiscard]] std::uintptr_t directCallTarget(std::uintptr_t callsite)
        {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(callsite);
            if (!bytes || bytes[0] != 0xE8) {
                return 0;
            }
            const auto displacement = *reinterpret_cast<const std::int32_t*>(bytes + 1);
            return callsite + 5 + displacement;
        }

        template <std::size_t Size>
        [[nodiscard]] bool bytesMatch(std::uintptr_t address, const std::array<std::uint8_t, Size>& expected)
        {
            return address != 0 && std::memcmp(reinterpret_cast<const void*>(address), expected.data(), expected.size()) == 0;
        }

        [[nodiscard]] bool preflightHooks()
        {
            REL::Relocation<std::uintptr_t> preLimbSite{ REL::Offset(kPreLimbCalculationHookOffset) };
            REL::Relocation<std::uintptr_t> scalarSite{ REL::Offset(kImpactScalarHookOffset) };
            REL::Relocation<std::uintptr_t> dispatcherCallsite{ REL::Offset(kActorHitDispatcherCallsiteOffset) };
            REL::Relocation<std::uintptr_t> mutationCallsite{ REL::Offset(kActorValueMutationCallsiteOffset) };
            REL::Relocation<std::uintptr_t> dispatcherFunction{ REL::Offset(kActorHitDispatcherFunctionOffset) };
            REL::Relocation<std::uintptr_t> mutationFunction{ REL::Offset(kActorValueMutationFunctionOffset) };

            return bytesMatch(preLimbSite.address(), kPreLimbCalculationBytes) &&
                   bytesMatch(scalarSite.address(), kImpactScalarBytes) &&
                   bytesMatch(dispatcherCallsite.address(), kActorHitDispatcherCallsiteBytes) &&
                   bytesMatch(mutationCallsite.address(), kActorValueMutationCallsiteBytes) &&
                   directCallTarget(dispatcherCallsite.address()) == dispatcherFunction.address() &&
                   directCallTarget(mutationCallsite.address()) == mutationFunction.address();
        }

#if defined(_MSC_VER)
        bool callHitDataCtorSafely(HitDataCtor_t function, RE::HitData* hitData)
        {
            __try {
                function(hitData);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool callHitDataDtorSafely(HitDataDtor_t function, RE::HitData* hitData)
        {
            __try {
                function(hitData);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool callGetEquippedWeaponSafely(
            GetEquippedWeapon_t function,
            RE::Actor* actor,
            RE::BGSObjectInstanceT<RE::TESObjectWEAP>* outWeapon,
            std::uint32_t equipIndex)
        {
            __try {
                return function(actor, outWeapon, equipIndex) == outWeapon;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool callInitializeWeaponHitDataSafely(
            InitializeWeaponHitData_t function,
            RE::HitData* hitData,
            RE::Actor* aggressor,
            RE::Actor* target,
            RE::BGSObjectInstanceT<RE::TESObjectWEAP>* weapon,
            std::uint32_t equipIndex)
        {
            __try {
                function(hitData, aggressor, target, weapon, equipIndex, true);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool callActorHitMeSafely(ActorHitMe_t function, RE::Actor* target, RE::HitData* hitData)
        {
            __try {
                function(target, *hitData);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        RE::bhkNPCollisionObject* resolveCollisionObjectSafely(RE::bhkWorld* bhkWorld, std::uint32_t bodyId)
        {
            __try {
                return RE::bhkNPCollisionObject::Getbhk(bhkWorld, *reinterpret_cast<RE::hknpBodyId*>(&bodyId));
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
        }

#endif

        [[nodiscard]] bool fillContactEvidence(const NativeMeleeHitInput& input, NativeMeleeHitResult& result)
        {
            if (!std::isfinite(input.havokToGameScale) || input.havokToGameScale <= 0.0f) {
                result.failure = NativeMeleeHitFailure::InvalidScale;
                return false;
            }
            if (!isFinite3(input.contact->contactPointHavok) || !isFinite3(input.contact->contactNormalHavok) ||
                !isFinite3(input.contact->sourceVelocityHavok) || !isFinite3(input.contact->targetVelocityHavok)) {
                result.failure = NativeMeleeHitFailure::NonFiniteContact;
                return false;
            }

            const float normalLength = length3(input.contact->contactNormalHavok);
            if (!std::isfinite(normalLength) || normalLength <= 0.000001f) {
                result.failure = NativeMeleeHitFailure::NonFiniteContact;
                return false;
            }
            const float inverseNormalLength = 1.0f / normalLength;
            float sourcePointVelocity[3]{};
            float targetPointVelocity[3]{};
            auto pointVelocity = [&](const float* linear,
                                     const float* angular,
                                     const float* centerOfMass,
                                     bool angularValid,
                                     bool centerValid,
                                     float* output) {
                std::copy_n(linear, 3, output);
                if (!angularValid || !centerValid || !isFinite3(angular) || !isFinite3(centerOfMass)) {
                    return;
                }
                const float radius[3]{
                    input.contact->contactPointHavok[0] - centerOfMass[0],
                    input.contact->contactPointHavok[1] - centerOfMass[1],
                    input.contact->contactPointHavok[2] - centerOfMass[2],
                };
                output[0] += angular[1] * radius[2] - angular[2] * radius[1];
                output[1] += angular[2] * radius[0] - angular[0] * radius[2];
                output[2] += angular[0] * radius[1] - angular[1] * radius[0];
            };
            const auto hasFlag = [&](rock::provider::RockProviderExternalContactFlagV1 flag) {
                return (input.contact->flags & static_cast<std::uint32_t>(flag)) != 0;
            };
            pointVelocity(
                input.contact->sourceVelocityHavok,
                input.contact->sourceAngularVelocityHavok,
                input.contact->sourceCenterOfMassHavok,
                hasFlag(rock::provider::RockProviderExternalContactFlagV1::SourceAngularVelocityValid),
                hasFlag(rock::provider::RockProviderExternalContactFlagV1::SourceCenterOfMassValid),
                sourcePointVelocity);
            pointVelocity(
                input.contact->targetVelocityHavok,
                input.contact->targetAngularVelocityHavok,
                input.contact->targetCenterOfMassHavok,
                hasFlag(rock::provider::RockProviderExternalContactFlagV1::TargetAngularVelocityValid),
                hasFlag(rock::provider::RockProviderExternalContactFlagV1::TargetCenterOfMassValid),
                targetPointVelocity);
            for (std::uint32_t i = 0; i < 3; ++i) {
                result.contactPointGame[i] = input.contact->contactPointHavok[i] * input.havokToGameScale;
                result.contactNormal[i] = input.contact->contactNormalHavok[i] * inverseNormalLength;
                result.relativeVelocityGame[i] =
                    (sourcePointVelocity[i] - targetPointVelocity[i]) * input.havokToGameScale;
            }
            if (!isFinite3(result.relativeVelocityGame)) {
                result.failure = NativeMeleeHitFailure::NonFiniteContact;
                return false;
            }

#if defined(_MSC_VER)
            auto* collisionObject = resolveCollisionObjectSafely(input.bhkWorld, input.contact->targetExternalBodyId);
#else
            RE::hknpBodyId targetBodyId{ input.contact->targetExternalBodyId };
            auto* collisionObject = RE::bhkNPCollisionObject::Getbhk(input.bhkWorld, targetBodyId);
#endif
            if (!collisionObject) {
                result.failure = NativeMeleeHitFailure::MissingTargetCollisionObject;
                return false;
            }
            result.targetCollisionObject = reinterpret_cast<std::uintptr_t>(collisionObject);
            return true;
        }

        rock::provider::RockProviderImpactOutcomeV1 makeBaseOutcome(const NativeMeleeHitInput& input)
        {
            rock::provider::RockProviderImpactOutcomeV1 outcome{};
            outcome.impactId = input.contact->impactId;
            outcome.episodeId = input.contact->episodeId;
            outcome.contactSequence = input.contact->sequence;
            outcome.submittedFrameIndex = input.submissionFrameIndex;
            outcome.targetActorFormId = input.contact->targetActorFormId;
            outcome.targetBodyId = input.contact->targetExternalBodyId;
            outcome.sourceBodyId = input.contact->sourceBodyId;
            outcome.sourceWeaponFormId = input.contact->sourceWeaponFormId;
            outcome.targetBodyPartIndex = input.contact->targetBodyPartIndex;
            outcome.sourceSurfaceRegion = input.contact->sourceSurfaceRegion;
            outcome.sourceSurfaceDamageCoefficient = input.contact->sourceSurfaceDamageCoefficient;
            outcome.closingSpeedGame = input.closingSpeedGame;
            outcome.requestedNativeMultiplier = input.nativeDamageMultiplier;
            return outcome;
        }
    }

    bool installNativeMeleeHitBridge()
    {
        if (s_bridgeInstalled.load(std::memory_order_acquire)) {
            return true;
        }
        if (!preflightHooks()) {
            logger::critical(
                "ROCK: Native melee bridge preflight failed; verified FO4VR callsite bytes or targets do not match.");
            return false;
        }

        auto& trampoline = F4SE::GetTrampoline();
        PreLimbCalculationHookCode preLimbHook{
            reinterpret_cast<std::uintptr_t>(&injectPreLimbCalculationSafely),
            REL::Relocation<std::uintptr_t>{ REL::Offset(kPreLimbCalculationReturnOffset) }.address()
        };
        preLimbHook.ready();
        ImpactScalarHookCode scalarHook{
            reinterpret_cast<std::uintptr_t>(&scaleNativeImpactScalar),
            REL::Relocation<std::uintptr_t>{ REL::Offset(kImpactScalarReturnOffset) }.address()
        };
        scalarHook.ready();
        if (trampoline.free_size() < preLimbHook.getSize() + scalarHook.getSize() + 64) {
            logger::critical(
                "ROCK: Native melee bridge install needs {} trampoline bytes but only {} remain.",
                preLimbHook.getSize() + scalarHook.getSize() + 64,
                trampoline.free_size());
            return false;
        }
        auto* preLimbRelay = static_cast<std::uint8_t*>(trampoline.allocate(preLimbHook.getSize()));
        std::memcpy(preLimbRelay, preLimbHook.getCode(), preLimbHook.getSize());
        auto* scalarRelay = static_cast<std::uint8_t*>(trampoline.allocate(scalarHook.getSize()));
        std::memcpy(scalarRelay, scalarHook.getCode(), scalarHook.getSize());

        REL::Relocation<std::uintptr_t> preLimbSite{ REL::Offset(kPreLimbCalculationHookOffset) };
        REL::Relocation<std::uintptr_t> scalarSite{ REL::Offset(kImpactScalarHookOffset) };
        REL::Relocation<std::uintptr_t> dispatcherCallsite{ REL::Offset(kActorHitDispatcherCallsiteOffset) };
        REL::Relocation<std::uintptr_t> mutationCallsite{ REL::Offset(kActorValueMutationCallsiteOffset) };

        // Publish every verified original before exposing a patched callsite.
        // FO4VR can run actor/physics work on other threads during startup;
        // this avoids even a one-instruction window where a hook sees null.
        s_originalActorHitDispatcher.store(
            REL::Relocation<std::uintptr_t>{ REL::Offset(kActorHitDispatcherFunctionOffset) }.address(),
            std::memory_order_release);
        s_originalActorValueMutation.store(
            REL::Relocation<std::uintptr_t>{ REL::Offset(kActorValueMutationFunctionOffset) }.address(),
            std::memory_order_release);
        (void)trampoline.write_branch<5>(preLimbSite.address(), reinterpret_cast<std::uintptr_t>(preLimbRelay));
        REL::safe_fill(preLimbSite.address() + 5, REL::NOP, 3);
        (void)trampoline.write_call<5>(dispatcherCallsite.address(), &actorHitDispatcherHook);
        (void)trampoline.write_call<5>(mutationCallsite.address(), &actorValueMutationHook);
        (void)trampoline.write_branch<5>(scalarSite.address(), reinterpret_cast<std::uintptr_t>(scalarRelay));
        REL::safe_fill(scalarSite.address() + 5, REL::NOP, 2);

        s_bridgeInstalled.store(true, std::memory_order_release);
        logger::info(
            "ROCK: Native melee bridge installed (preLimb=+0x{:X}, scalar=+0x{:X}, dispatcher=+0x{:X}, mutation=+0x{:X}).",
            kPreLimbCalculationHookOffset,
            kImpactScalarHookOffset,
            kActorHitDispatcherCallsiteOffset,
            kActorValueMutationCallsiteOffset);
        return true;
    }

    void resetNativeMeleeHitBridge()
    {
        s_bridgeEpoch.fetch_add(1, std::memory_order_acq_rel);
        std::scoped_lock lock(s_outcomeMutex);
        s_pendingImpacts.fill({});
        s_completedOutcomes.fill({});
        s_completedHead = 0;
        s_completedCount = 0;
    }

    std::uint32_t drainNativeMeleeHitOutcomes(
        std::uint64_t currentFrameIndex,
        rock::provider::RockProviderImpactOutcomeV1* outOutcomes,
        std::uint32_t maxOutcomes)
    {
        if (!outOutcomes || maxOutcomes == 0) {
            return 0;
        }

        std::scoped_lock lock(s_outcomeMutex);
        for (auto& pending : s_pendingImpacts) {
            if (!pending.occupied || currentFrameIndex < pending.outcome.submittedFrameIndex ||
                currentFrameIndex - pending.outcome.submittedFrameIndex <= kUnknownOutcomeAfterFrames) {
                continue;
            }
            auto timedOut = pending.outcome;
            timedOut.lifecycle = rock::provider::RockProviderImpactOutcomeLifecycleV1::UnknownAfterSubmission;
            timedOut.completedFrameIndex = currentFrameIndex;
            timedOut.flags |= static_cast<std::uint32_t>(rock::provider::RockProviderImpactOutcomeFlagV1::QueuedConsumer);
            queueCompletedOutcomeLocked(timedOut);
            pending = {};
        }

        std::uint32_t copied = 0;
        while (copied < maxOutcomes && s_completedCount > 0) {
            auto outcome = s_completedOutcomes[s_completedHead];
            if (outcome.completedFrameIndex == 0) {
                outcome.completedFrameIndex = currentFrameIndex;
            }
            outOutcomes[copied++] = outcome;
            s_completedOutcomes[s_completedHead] = {};
            s_completedHead = (s_completedHead + 1) % s_completedOutcomes.size();
            --s_completedCount;
        }
        return copied;
    }

    NativeMeleeHitResult applyNativeMeleeHit(const NativeMeleeHitInput& input)
    {
        NativeMeleeHitResult result{};
        result.outcome = input.contact ? makeBaseOutcome(input) : rock::provider::RockProviderImpactOutcomeV1{};
        if (!s_bridgeInstalled.load(std::memory_order_acquire)) {
            result.failure = NativeMeleeHitFailure::BridgeNotInstalled;
            return result;
        }
        if (!input.bhkWorld || !input.target || !input.aggressor || !input.contact ||
            input.contact->impactId == 0 || input.contact->sourceBodyId == kInvalidBodyId ||
            input.contact->sourceWeaponFormId == 0 || input.contact->targetBodyPartIndex == 0xFFFF'FFFFu ||
            !std::isfinite(input.nativeDamageMultiplier) || input.nativeDamageMultiplier < 0.0f) {
            result.failure = NativeMeleeHitFailure::InvalidInput;
            return result;
        }
        if (!input.targetBodyPart || input.targetNativeDamageLimb == 0xFFFF'FFFFu) {
            result.failure = NativeMeleeHitFailure::MissingTargetBodyPart;
            return result;
        }
        if (!weaponWitnessMatches(input.expectedWeapon, input.currentWeapon)) {
            result.failure = NativeMeleeHitFailure::WeaponWitnessMismatch;
            return result;
        }
        if (!fillContactEvidence(input, result)) {
            return result;
        }

        // The resolver already selected an exact BPTD entry for this candidate.
        // Consume that same witnessed entry instead of independently walking a
        // second mutable Actor::race chain immediately before native dispatch.
        auto* targetBodyPart = input.targetBodyPart;
        const auto nativeDamageLimb = input.targetNativeDamageLimb;

        RE::BGSObjectInstanceT<RE::TESObjectWEAP> equippedWeapon{ nullptr, nullptr };
        static REL::Relocation<GetEquippedWeapon_t> getEquippedWeapon{ REL::Offset(kGetEquippedWeaponOffset) };
#if defined(_MSC_VER)
        const bool weaponResolved = callGetEquippedWeaponSafely(
            getEquippedWeapon.get(), input.aggressor, &equippedWeapon, input.currentWeapon.equipIndex);
#else
        const bool weaponResolved = getEquippedWeapon(
            input.aggressor, &equippedWeapon, input.currentWeapon.equipIndex) == &equippedWeapon;
#endif
        if (!weaponResolved || !equippedWeapon.object) {
            result.failure = NativeMeleeHitFailure::MissingEquippedWeapon;
            return result;
        }
        if (equippedWeapon.object->GetFormID() != input.contact->sourceWeaponFormId) {
            result.failure = NativeMeleeHitFailure::EquippedWeaponMismatch;
            return result;
        }
        if (input.expectedWeapon.instanceDataAddress != 0 &&
            reinterpret_cast<std::uintptr_t>(equippedWeapon.instanceData.get()) !=
                input.expectedWeapon.instanceDataAddress) {
            result.failure = NativeMeleeHitFailure::WeaponWitnessMismatch;
            return result;
        }

        alignas(RE::HitData) std::byte hitDataStorage[sizeof(RE::HitData)]{};
        auto* hitData = reinterpret_cast<RE::HitData*>(hitDataStorage);
        static REL::Relocation<HitDataCtor_t> hitDataCtor{ REL::Offset(kHitDataCtorOffset) };
        static REL::Relocation<HitDataDtor_t> hitDataDtor{ REL::Offset(kHitDataDtorOffset) };
        static REL::Relocation<InitializeWeaponHitData_t> initializeWeaponHitData{ REL::Offset(kInitializeWeaponHitDataOffset) };
        static REL::Relocation<ActorHitMe_t> actorHitMe{ REL::Offset(kActorHitMeOffset) };

#if defined(_MSC_VER)
        const bool constructed = callHitDataCtorSafely(hitDataCtor.get(), hitData);
#else
        hitDataCtor(hitData);
        const bool constructed = true;
#endif
        if (!constructed) {
            result.failure = NativeMeleeHitFailure::HitDataConstructorFailed;
            return result;
        }

        HitInjectionContext injection{};
        injection.hitData = hitData;
        injection.target = input.target;
        injection.bodyPart = targetBodyPart;
        injection.targetCollisionObject = reinterpret_cast<RE::bhkNPCollisionObject*>(result.targetCollisionObject);
        std::copy_n(result.contactPointGame, 3, injection.pointGame);
        std::copy_n(result.contactNormal, 3, injection.normal);
        std::copy_n(result.relativeVelocityGame, 3, injection.relativeVelocityGame);
        injection.multiplier = input.nativeDamageMultiplier;

        bool initialized = false;
        if (s_injectionDepth < s_injectionStack.size()) {
            s_injectionStack[s_injectionDepth++] = &injection;
#if defined(_MSC_VER)
            initialized = callInitializeWeaponHitDataSafely(
                initializeWeaponHitData.get(), hitData, input.aggressor, input.target, &equippedWeapon,
                input.currentWeapon.equipIndex);
#else
            initializeWeaponHitData(
                hitData, input.aggressor, input.target, &equippedWeapon, input.currentWeapon.equipIndex, true);
            initialized = true;
#endif
            --s_injectionDepth;
            s_injectionStack[s_injectionDepth] = nullptr;
        }

        if (!initialized) {
            result.failure = NativeMeleeHitFailure::InitializeWeaponHitDataFailed;
        } else if (!injection.applied ||
            *reinterpret_cast<std::uint32_t*>(reinterpret_cast<std::byte*>(hitData) + 0xD0) != nativeDamageLimb) {
            result.failure = NativeMeleeHitFailure::EvidenceInjectionFailed;
        } else {
            result.outcome.submittedHealthDamage = hitData->healthDamage;
            result.outcome.submittedLimbDamage = hitData->targetedLimbDamage;
            result.outcome.lifecycle = rock::provider::RockProviderImpactOutcomeLifecycleV1::Submitted;

            PendingImpact pending{};
            pending.occupied = true;
            pending.target = input.target;
            pending.sourceWeaponFormId = input.contact->sourceWeaponFormId;
            pending.nativeDamageLimb = nativeDamageLimb;
            pending.limbActorValueFormId = input.contact->targetLimbActorValueFormId;
            pending.submitThreadId = currentThreadId();
            std::copy_n(result.contactPointGame, 3, pending.pointGame);
            pending.outcome = result.outcome;
            pending.bridgeEpoch = s_bridgeEpoch.load(std::memory_order_acquire);

            if (!insertPendingImpact(pending)) {
                result.failure = NativeMeleeHitFailure::PendingQueueFull;
            } else {
#if defined(_MSC_VER)
                result.submitted = callActorHitMeSafely(actorHitMe.get(), input.target, hitData);
#else
                actorHitMe(input.target, *hitData);
                result.submitted = true;
#endif
                if (!result.submitted) {
                    removePendingImpact(input.contact->impactId);
                    result.failure = NativeMeleeHitFailure::ActorHitMeFailed;
                }
            }
        }

#if defined(_MSC_VER)
        if (!callHitDataDtorSafely(hitDataDtor.get(), hitData)) {
            if (result.failure == NativeMeleeHitFailure::None) {
                result.failure = NativeMeleeHitFailure::HitDataDestructorFailed;
            }
            return result;
        }
#else
        hitDataDtor(hitData);
#endif
        return result;
    }

    const char* nativeMeleeHitFailureName(NativeMeleeHitFailure failure)
    {
        switch (failure) {
        case NativeMeleeHitFailure::None:
            return "None";
        case NativeMeleeHitFailure::BridgeNotInstalled:
            return "BridgeNotInstalled";
        case NativeMeleeHitFailure::InvalidInput:
            return "InvalidInput";
        case NativeMeleeHitFailure::InvalidScale:
            return "InvalidScale";
        case NativeMeleeHitFailure::NonFiniteContact:
            return "NonFiniteContact";
        case NativeMeleeHitFailure::MissingTargetCollisionObject:
            return "MissingTargetCollisionObject";
        case NativeMeleeHitFailure::MissingTargetBodyPart:
            return "MissingTargetBodyPart";
        case NativeMeleeHitFailure::EvidenceInjectionFailed:
            return "EvidenceInjectionFailed";
        case NativeMeleeHitFailure::WeaponWitnessMismatch:
            return "WeaponWitnessMismatch";
        case NativeMeleeHitFailure::MissingEquippedWeapon:
            return "MissingEquippedWeapon";
        case NativeMeleeHitFailure::EquippedWeaponMismatch:
            return "EquippedWeaponMismatch";
        case NativeMeleeHitFailure::PendingQueueFull:
            return "PendingQueueFull";
        case NativeMeleeHitFailure::HitDataConstructorFailed:
            return "HitDataConstructorFailed";
        case NativeMeleeHitFailure::InitializeWeaponHitDataFailed:
            return "InitializeWeaponHitDataFailed";
        case NativeMeleeHitFailure::ActorHitMeFailed:
            return "ActorHitMeFailed";
        case NativeMeleeHitFailure::HitDataDestructorFailed:
            return "HitDataDestructorFailed";
        default:
            return "Unknown";
        }
    }
}
