#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"

#include <REL/Relocation.h>
#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::weapon_transition_animation_acceleration
{
    namespace
    {
        using Direction =
            weapon_transition_animation_acceleration_policy::Direction;
        using ClipGeneratorActivateFn = void (*)(void*, void*);
        using ClipGeneratorUpdateFn = void (*)(void*, void*, float);
        using ClipGeneratorDeactivateFn = void (*)(void*, void*);
        using BlendUpdateFn = void (*)(void*, void*, float, void*);
        BlendUpdateFn s_originalBlendUpdate{};

        /*
         * PAPER's passive ClipTelemetry and ROCK's former clip harvester
         * independently established this FO4VR 1.2.72 contract. Hook the
         * concrete lifecycle entries instead of the hkbClipGenerator vtable:
         * PAPER may own those slots, but its shims call these original entry
         * addresses, so the two plugins compose in either load order.
         *
         * Raw FO4VR disassembly:
         *  - activate   0x14192CA40, vtable +0x38;
         *  - update     0x14192D0D0, vtable +0x40, timestep in XMM2;
         *  - deactivate 0x14192D510, vtable +0x50.
         * Each validated prefix ends on a complete, position-independent
         * instruction boundary suitable for EntryTrampolineHook.
         */
        constexpr std::uintptr_t kClipGeneratorActivate = 0x192CA40;
        constexpr std::uintptr_t kClipGeneratorUpdate = 0x192D0D0;
        constexpr std::uintptr_t kClipGeneratorDeactivate = 0x192D510;
        constexpr std::array<std::uint8_t, 14> kExpectedActivateEntry{
            0x40, 0x53,
            0x55,
            0x56,
            0x57,
            0x41, 0x57,
            0x48, 0x81, 0xEC, 0x50, 0x01, 0x00, 0x00,
        };
        constexpr std::array<std::uint8_t, 15> kExpectedUpdateEntry{
            0x4C, 0x8B, 0xDC,
            0x49, 0x89, 0x5B, 0x10,
            0x56,
            0x48, 0x81, 0xEC, 0xB0, 0x00, 0x00, 0x00,
        };
        constexpr std::array<std::uint8_t, 17> kExpectedDeactivateEntry{
            0x48, 0x89, 0x5C, 0x24, 0x20,
            0x56,
            0x48, 0x83, 0xEC, 0x30,
            0x48, 0x8B, 0x81, 0xD0, 0x00, 0x00, 0x00,
        };

        /*
         * Player graph-manager access was raw-disassembly verified for the
         * retired ROCK clip harvester and remains in active PAPER telemetry:
         * TESObjectREFR+0x48 is IAnimationGraphManagerHolder, vtable +0x20 is
         * GetAnimationGraphManagerImpl(out&), and the returned add-ref'd
         * BSAnimationGraphManager has exact vtable +0x2E00550. The manager's
         * bounded graph array yields BShkbAnimationGraph objects (exact vtable
         * +0x2E00A48), whose hkbCharacter is inline at graph+0x1C8.
         */
        constexpr std::uintptr_t kRefrGraphHolderInterfaceOffset = 0x48;
        constexpr std::uintptr_t kGetGraphManagerVtableSlotOffset = 0x20;
        constexpr std::uintptr_t kGraphManagerRefCountOffset = 0x8;
        constexpr std::uintptr_t kGraphManagerVtableModuleOffset = 0x2E00550;
        constexpr std::uintptr_t kManagerGraphsCapacityOffset = 0x40;
        constexpr std::uintptr_t kManagerGraphsStorageOffset = 0x48;
        constexpr std::uint32_t kGraphsInlineStorageFlag = 0x8000'0000u;
        constexpr std::uintptr_t kGraphVtableModuleOffset = 0x2E00A48;
        constexpr std::uintptr_t kGraphCharacterOffset = 0x1C8;
        constexpr std::uint32_t kMaxGraphSlots = 8;
        constexpr std::size_t kMaxTransitionClips =
            kMaximumActivationEvidenceClips;

        constexpr std::uintptr_t kEncodedStateMask = 0x7;
        constexpr std::uint32_t kInvalidNativeState = 0xFFFFFFFFu;
        constexpr std::uint32_t kMaxResolveFailureLogs = 3;

        struct RuntimeLease
        {
            RE::PlayerCharacter* player{ nullptr };
            Identity identity{};
            Direction direction{ Direction::Draw };
            /*
             * Wall-clock lease by contract: the acceleration supervises an
             * asynchronous native animation request and must expire even when
             * game-frame updates stall (menus, loading). Deliberately not
             * gameplay time.
             */
            std::chrono::steady_clock::time_point requestedAt{};
            std::uint64_t sequence{ 0 };
            bool active{ false };
        };

        enum class GraphTargetResult : std::uint8_t
        {
            Ready,
            MissingManager,
            MissingCharacters,
        };

        using GetGraphManagerFn = bool (*)(void*, void**);
        using DestroyGraphManagerFn = void* (*)(void*, std::uint32_t);

        ClipGeneratorActivateFn s_originalClipActivate = nullptr;
        ClipGeneratorUpdateFn s_originalClipUpdate = nullptr;
        ClipGeneratorDeactivateFn s_originalClipDeactivate = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };

        // Low pointer bits carry the exact requested native state (2/5), so
        // animation callbacks consume one coherent lease snapshot.
        std::atomic<std::uintptr_t> s_encodedLease{ 0 };
        std::array<std::atomic<std::uintptr_t>, kMaxGraphSlots>
            s_targetCharacters{};
        // Diagnostic address-only witnesses, never dereferenced after the
        // callback. Per-effect admission avoids one graph's long transition
        // suppressing evidence of a shorter transition in another graph.
        std::array<std::atomic<std::uintptr_t>, 32> s_observedBlends{};
        std::atomic<std::uint32_t> s_targetCharacterCount{ 0 };
        std::array<std::atomic<std::uintptr_t>, kMaxTransitionClips>
            s_transitionClips{};
        std::array<std::atomic<std::uintptr_t>, kMaxTransitionClips>
            s_updatedTransitionClips{};

        std::atomic<std::uint32_t> s_activationCalls{ 0 };
        std::atomic<std::uint32_t> s_matchedActivations{ 0 };
        std::atomic<std::uint32_t> s_registeredClips{ 0 };
        std::atomic<std::uint32_t> s_clipRegistryOverflows{ 0 };
        std::atomic<std::uint32_t> s_registeredUpdateCalls{ 0 };
        std::atomic<std::uint32_t> s_acceleratedSamples{ 0 };
        std::atomic<std::uint32_t> s_lastOriginalTimestepBits{ 0 };
        std::atomic<std::uint32_t> s_lastAcceleratedTimestepBits{ 0 };
        std::atomic<DWORD> s_animationThreadId{ 0 };
        std::atomic<bool> s_multipleAnimationThreads{ false };
        std::uint32_t s_resolveFailureLogs{ 0 };
        RuntimeLease s_runtimeLease{};
        std::uint64_t s_nextSequence{ 0 };

        [[nodiscard]] bool pointerInModuleImage(
            const std::uintptr_t value) noexcept
        {
            const auto base = REL::Module::get().base();
            return value > base && value - base < 0x800'0000ull;
        }

        class AcquiredGraphManager
        {
        public:
            explicit AcquiredGraphManager(RE::TESObjectREFR* refr) noexcept
            {
                if (!refr) {
                    return;
                }
                auto* holderInterface = reinterpret_cast<void*>(
                    reinterpret_cast<std::uintptr_t>(refr) +
                    kRefrGraphHolderInterfaceOffset);
                std::uintptr_t holderVtable = 0;
                if (!native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(
                            holderInterface),
                        holderVtable) ||
                    !pointerInModuleImage(holderVtable)) {
                    return;
                }

                std::uintptr_t getManagerAddress = 0;
                if (!native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(
                            holderVtable +
                            kGetGraphManagerVtableSlotOffset),
                        getManagerAddress) ||
                    !pointerInModuleImage(getManagerAddress)) {
                    return;
                }

                void* raw = nullptr;
                const bool managerAvailable =
                    reinterpret_cast<GetGraphManagerFn>(getManagerAddress)(
                        holderInterface,
                        &raw);
                // The engine writes an add-ref'd pointer. Retain it for RAII
                // release even if the boolean result and pointer disagree.
                _reference = raw;
                if (!managerAvailable || !raw) {
                    return;
                }

                std::uintptr_t managerVtable = 0;
                if (native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(raw),
                        managerVtable) &&
                    managerVtable == REL::Module::get().base() +
                        kGraphManagerVtableModuleOffset) {
                    _manager = raw;
                }
            }

            AcquiredGraphManager(const AcquiredGraphManager&) = delete;
            AcquiredGraphManager& operator=(
                const AcquiredGraphManager&) = delete;

            ~AcquiredGraphManager()
            {
                if (!_reference) {
                    return;
                }
                auto* refCount = reinterpret_cast<std::uint32_t*>(
                    reinterpret_cast<std::uintptr_t>(_reference) +
                    kGraphManagerRefCountOffset);
                if (!native_memory::pointerRangeLooksWritable(
                        refCount,
                        sizeof(*refCount))) {
                    return;
                }
                if (std::atomic_ref<std::uint32_t>{ *refCount }.fetch_sub(
                        1,
                        std::memory_order_acq_rel) != 1) {
                    return;
                }

                std::uintptr_t vtable = 0;
                std::uintptr_t destroy = 0;
                if (native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(_reference),
                        vtable) &&
                    pointerInModuleImage(vtable) &&
                    native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(vtable),
                        destroy) &&
                    pointerInModuleImage(destroy)) {
                    reinterpret_cast<DestroyGraphManagerFn>(destroy)(
                        _reference,
                        1);
                }
            }

            [[nodiscard]] const void* get() const noexcept
            {
                return _manager;
            }

        private:
            void* _reference{ nullptr };
            const void* _manager{ nullptr };
        };

        [[nodiscard]] bool claimOrValidateOwnerThread() noexcept
        {
            const DWORD currentThreadId = GetCurrentThreadId();
            DWORD expectedThreadId = 0;
            (void)s_ownerThreadId.compare_exchange_strong(
                expectedThreadId,
                currentThreadId,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            return s_ownerThreadId.load(std::memory_order_acquire) ==
                   currentThreadId;
        }

        [[nodiscard]] std::uintptr_t encodeLease(
            RE::PlayerCharacter* player,
            const Direction direction) noexcept
        {
            auto* actor = static_cast<RE::Actor*>(player);
            const auto actorAddress = reinterpret_cast<std::uintptr_t>(actor);
            if (!actorAddress || (actorAddress & kEncodedStateMask) != 0) {
                return 0;
            }
            return actorAddress |
                   weapon_transition_animation_acceleration_policy::
                       transitionState(direction);
        }

        void clearPublishedTargets() noexcept
        {
            s_targetCharacterCount.store(0, std::memory_order_release);
            for (auto& character : s_targetCharacters) {
                character.store(0, std::memory_order_relaxed);
            }
            for (auto& blend : s_observedBlends) blend.store(0, std::memory_order_relaxed);
            for (auto& clip : s_transitionClips) {
                clip.store(0, std::memory_order_relaxed);
            }
            for (auto& clip : s_updatedTransitionClips) {
                clip.store(0, std::memory_order_relaxed);
            }
        }

        [[nodiscard]] GraphTargetResult publishPlayerGraphCharacters(
            RE::PlayerCharacter* player) noexcept
        {
            AcquiredGraphManager acquired{ player };
            const auto manager = reinterpret_cast<std::uintptr_t>(
                acquired.get());
            if (!manager) {
                return GraphTargetResult::MissingManager;
            }

            std::uint32_t capacityAndFlags = 0;
            if (!native_memory::tryReadValue(
                    reinterpret_cast<const std::uint32_t*>(
                        manager + kManagerGraphsCapacityOffset),
                    capacityAndFlags)) {
                return GraphTargetResult::MissingCharacters;
            }
            const std::uint32_t capacity = (std::min)(
                capacityAndFlags & ~kGraphsInlineStorageFlag,
                kMaxGraphSlots);
            if (capacity == 0) {
                return GraphTargetResult::MissingCharacters;
            }

            const auto storageAddress =
                manager + kManagerGraphsStorageOffset;
            std::uintptr_t graphsBase = storageAddress;
            if ((capacityAndFlags & kGraphsInlineStorageFlag) == 0 &&
                !native_memory::tryReadValue(
                    reinterpret_cast<const std::uintptr_t*>(storageAddress),
                    graphsBase)) {
                return GraphTargetResult::MissingCharacters;
            }
            if (!native_memory::pointerRangeLooksReadable(
                    reinterpret_cast<const void*>(graphsBase),
                    static_cast<std::size_t>(capacity) *
                        sizeof(std::uintptr_t))) {
                return GraphTargetResult::MissingCharacters;
            }

            std::array<std::uintptr_t, kMaxGraphSlots> characters{};
            std::uint32_t characterCount = 0;
            for (std::uint32_t index = 0;
                 index < capacity && characterCount < characters.size();
                 ++index) {
                std::uintptr_t graph = 0;
                std::uintptr_t graphVtable = 0;
                if (!native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(
                            graphsBase +
                            index * sizeof(std::uintptr_t)),
                        graph) ||
                    !graph ||
                    !native_memory::tryReadValue(
                        reinterpret_cast<const std::uintptr_t*>(graph),
                        graphVtable) ||
                    graphVtable != REL::Module::get().base() +
                        kGraphVtableModuleOffset) {
                    continue;
                }

                const auto character = graph + kGraphCharacterOffset;
                if (!native_memory::pointerRangeLooksReadable(
                        reinterpret_cast<const void*>(character),
                        sizeof(std::uintptr_t))) {
                    continue;
                }
                characters[characterCount++] = character;
            }
            if (characterCount == 0) {
                return GraphTargetResult::MissingCharacters;
            }

            for (std::uint32_t index = 0; index < characterCount; ++index) {
                s_targetCharacters[index].store(
                    characters[index],
                    std::memory_order_relaxed);
            }
            s_targetCharacterCount.store(
                characterCount,
                std::memory_order_release);
            return GraphTargetResult::Ready;
        }

        [[nodiscard]] bool contextMatchesPlayerGraph(
            const void* context) noexcept
        {
            const auto characterCount =
                s_targetCharacterCount.load(std::memory_order_acquire);
            if (!context || characterCount == 0) {
                return false;
            }

            // PAPER's in-game telemetry established that the owning
            // hkbCharacter is one of the first four hkbContext qwords.
            std::array<std::uintptr_t, 4> contextSlots{};
            if (!native_memory::guardedCopyFromMemory(
                    context,
                    contextSlots.data(),
                    sizeof(contextSlots))) {
                return false;
            }
            for (const auto contextSlot : contextSlots) {
                for (std::uint32_t index = 0;
                     index < characterCount;
                     ++index) {
                    if (contextSlot == s_targetCharacters[index].load(
                                           std::memory_order_relaxed)) {
                        return true;
                    }
                }
            }
            return false;
        }

        void registerActivatedClip(void* clipGenerator, void* context) noexcept
        {
            const auto lease = s_encodedLease.load(std::memory_order_acquire);
            if (!clipGenerator || lease == 0) {
                return;
            }
            s_activationCalls.fetch_add(1, std::memory_order_relaxed);
            if (!contextMatchesPlayerGraph(context) ||
                s_encodedLease.load(std::memory_order_acquire) != lease) {
                return;
            }
            const auto direction = weapon_transition_animation_acceleration_policy::directionForTransitionState(
                static_cast<std::uint32_t>(lease & kEncodedStateMask));
            if (direction == Direction::Draw) {
                // FO4VR hkbClipGenerator animationName is hkStringPtr +90,
                // independently decoded by existing PAPER clip telemetry.
                // A draw lease spans late native model replacement, so matching
                // the character alone would also accelerate idle/fire/reload.
                std::uintptr_t namePointer{};
                std::array<char, 256> name{};
                if (!native_memory::tryReadField(clipGenerator, 0x90, namePointer) ||
                    !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(namePointer & ~std::uintptr_t{1}), name.data(), name.size())) return;
                const auto end = std::find(name.begin(), name.end(), '\0');
                if (end == name.end() || !weapon_transition_animation_acceleration_policy::isDrawClip(
                        std::string_view(name.data(), static_cast<std::size_t>(end-name.begin())))) return;
                ROCK_LOG_SAMPLE_INFO(Weapon, 10,
                    "Weapon draw clip admitted after native draw request clip={} name='{}' nativeState={}",
                    clipGenerator, name.data(), f4vr::getNativeWeaponState(reinterpret_cast<RE::Actor*>(lease & ~kEncodedStateMask)));
            }
            s_matchedActivations.fetch_add(1, std::memory_order_relaxed);

            const auto clip = reinterpret_cast<std::uintptr_t>(clipGenerator);
            for (auto& slot : s_transitionClips) {
                if (slot.load(std::memory_order_acquire) == clip) {
                    return;
                }
            }
            for (std::size_t index = 0;
                 index < s_transitionClips.size();
                 ++index) {
                std::uintptr_t empty = 0;
                if (s_transitionClips[index].compare_exchange_strong(
                        empty,
                        clip,
                        std::memory_order_acq_rel,
                        std::memory_order_relaxed)) {
                    s_updatedTransitionClips[index].store(
                        0,
                        std::memory_order_release);
                    s_registeredClips.fetch_add(
                        1,
                        std::memory_order_relaxed);
                    return;
                }
            }
            s_clipRegistryOverflows.fetch_add(1, std::memory_order_relaxed);
        }

        void unregisterClip(void* clipGenerator) noexcept
        {
            const auto clip = reinterpret_cast<std::uintptr_t>(clipGenerator);
            if (!clip ||
                s_encodedLease.load(std::memory_order_acquire) == 0) {
                return;
            }
            for (std::size_t index = 0;
                 index < s_transitionClips.size();
                 ++index) {
                auto expected = clip;
                // Clear every match: concurrent graph workers can observe
                // the same activation before either publishes its slot.
                if (s_transitionClips[index].compare_exchange_strong(
                        expected,
                        0,
                        std::memory_order_acq_rel,
                        std::memory_order_relaxed)) {
                    auto updated = clip;
                    (void)s_updatedTransitionClips[index].
                        compare_exchange_strong(
                            updated,
                            0,
                            std::memory_order_acq_rel,
                            std::memory_order_relaxed);
                }
            }
        }

        [[nodiscard]] bool markUpdatedRegisteredClip(
            void* clipGenerator) noexcept
        {
            const auto clip = reinterpret_cast<std::uintptr_t>(clipGenerator);
            if (!clip) {
                return false;
            }
            for (std::size_t index = 0;
                 index < s_transitionClips.size();
                 ++index) {
                if (s_transitionClips[index].load(
                        std::memory_order_acquire) == clip) {
                    s_updatedTransitionClips[index].store(
                        clip,
                        std::memory_order_release);
                    return true;
                }
            }
            return false;
        }

        void recordAnimationThread() noexcept
        {
            const DWORD currentThreadId = GetCurrentThreadId();
            DWORD expectedThreadId = 0;
            if (s_animationThreadId.compare_exchange_strong(
                    expectedThreadId,
                    currentThreadId,
                    std::memory_order_relaxed,
                    std::memory_order_relaxed) ||
                expectedThreadId == currentThreadId) {
                return;
            }
            s_multipleAnimationThreads.store(true, std::memory_order_relaxed);
        }

        [[nodiscard]] float acceleratedTimestep(
            void* clipGenerator,
            const float timestep) noexcept
        {
            const auto lease = s_encodedLease.load(std::memory_order_acquire);
            if (lease == 0 ||
                !markUpdatedRegisteredClip(clipGenerator)) {
                return timestep;
            }
            s_registeredUpdateCalls.fetch_add(1, std::memory_order_relaxed);

            auto* player = reinterpret_cast<RE::Actor*>(
                lease & ~kEncodedStateMask);
            const auto transitionState = static_cast<std::uint32_t>(
                lease & kEncodedStateMask);
            const Direction direction =
                weapon_transition_animation_acceleration_policy::
                    directionForTransitionState(transitionState);
            const auto nativeState = f4vr::getNativeWeaponState(player);
            if (!weapon_transition_animation_acceleration_policy::
                    shouldAccelerateSample(
                        true,
                        direction,
                        nativeState) ||
                !std::isfinite(timestep) ||
                timestep <= 0.0f ||
                s_encodedLease.load(std::memory_order_acquire) != lease) {
                return timestep;
            }

            float accelerated =
                weapon_transition_animation_acceleration_policy::
                    scaleTransitionTimestep(timestep);
            if (direction == Direction::Draw) {
                // Finish a loaded draw clip in its native update, crossing its
                // own weaponDraw/00NextClip events rather than fabricating them.
                // These read-only offsets are shared with verified PAPER clip
                // telemetry. Before a binding is readable, retain the existing
                // accelerated step and let the next native update resolve it.
                void* wrapper{};
                void* binding{};
                void* animation{};
                float duration{};
                if (native_memory::tryReadField(clipGenerator,0xD0,wrapper) &&
                    native_memory::tryReadField(wrapper,0x38,binding) &&
                    native_memory::tryReadField(binding,0x18,animation) &&
                    native_memory::tryReadField(animation,0x14,duration) &&
                    std::isfinite(duration) && duration > 0.0f && duration <= 300.0f) {
                    accelerated = (std::max)(accelerated,duration + timestep);
                }
            }
            if (!std::isfinite(accelerated)) {
                return timestep;
            }

            recordAnimationThread();
            s_acceleratedSamples.fetch_add(1, std::memory_order_relaxed);
            s_lastOriginalTimestepBits.store(
                std::bit_cast<std::uint32_t>(timestep),
                std::memory_order_relaxed);
            s_lastAcceleratedTimestepBits.store(
                std::bit_cast<std::uint32_t>(accelerated),
                std::memory_order_relaxed);
            return accelerated;
        }

        bool outgoingBranchHasDrawClip(void* context, void* root) noexcept
        {
            // Native 141F7A330 resolves serialized generators through the
            // behavior's clone map. 14194DEC0 enumerates active blender children;
            // 141A407F0 unwraps modifier/synchronized generators at +90.
            // Bound the walk and retain no generator pointer across callbacks.
            void* behavior{};
            if (!native_memory::tryReadField(context, 8, behavior)) return false;
            if (!behavior) {
                void* character{};
                if (!native_memory::tryReadField(context, 0, character) ||
                    !native_memory::tryReadField(character, 0x80, behavior)) return false;
            }
            if (!behavior || !native_memory::pointerRangeLooksReadable(behavior, 0x120)) return false;
            using ResolveClone = void* (*)(void*, void*);
            const auto resolve = reinterpret_cast<ResolveClone>(REL::Offset(0x17BE290).address());
            std::array<void*, 32> pending{}, visited{};
            std::size_t count = 1, seen = 0;
            pending[0] = root;
            while (count && seen < visited.size()) {
                auto* node = pending[--count];
                if (!node || !native_memory::pointerRangeLooksReadable(node, 0x44)) return false;
                node = resolve(behavior, node);
                if (!node) return false;
                if (std::find(visited.begin(), visited.begin() + seen, node) != visited.begin() + seen) continue;
                visited[seen++] = node;
                for (const auto& clip : s_transitionClips)
                    if (clip.load(std::memory_order_acquire) == reinterpret_cast<std::uintptr_t>(node)) return true;
                std::uint8_t kind{};
                if (!native_memory::tryReadField(node, 0x43, kind)) return false;
                if (kind == 2 || kind == 6) {
                    void* child{};
                    if (!native_memory::tryReadField(node, 0x90, child) || count == pending.size()) return false;
                    pending[count++] = child;
                } else if (kind == 3) {
                    void* children{};
                    void* enabled{};
                    std::int32_t childCount{}, enabledCount{};
                    if (!native_memory::tryReadField(node, 0xA0, children) ||
                        !native_memory::tryReadField(node, 0xA8, childCount) ||
                        !native_memory::tryReadField(node, 0xB0, enabled) ||
                        !native_memory::tryReadField(node, 0xB8, enabledCount) ||
                        childCount < 0 || childCount > 32 || enabledCount != childCount) return false;
                    for (std::int32_t i = 0; i < childCount; ++i) {
                        std::uint8_t active{};
                        if (!native_memory::tryReadField(enabled, i * 2, active)) return false;
                        if (!active) continue;
                        void* childInfo{};
                        void* child{};
                        if (!native_memory::tryReadField(children, i * sizeof(void*), childInfo) ||
                            !native_memory::tryReadField(childInfo, 0x30, child) || count == pending.size()) return false;
                        if (child) pending[count++] = child;
                    }
                }
            }
            return false;
        }

        void traceBlend(void* blend, void* context, void* from, void* to,
            float remaining, bool outgoingDraw) noexcept
        {
            if (!logger::isDebugEnabled()) return;
            const auto address = reinterpret_cast<std::uintptr_t>(blend);
            bool admitted = false;
            for (auto& slot : s_observedBlends) {
                auto expected = slot.load(std::memory_order_acquire);
                if (expected == address) return;
                if (expected == 0 && slot.compare_exchange_strong(expected, address, std::memory_order_acq_rel)) {
                    admitted = true;
                    break;
                }
                if (expected == address) return;
            }
            if (!admitted) return;

            struct NodeEvidence
            {
                std::array<char, 96> name{};
                std::uintptr_t vtable{};
                std::uint8_t kind{};
            };
            const auto describe = [](const void* node) {
                NodeEvidence evidence{};
                std::uintptr_t name{};
                (void)native_memory::tryReadField(node, 0, evidence.vtable);
                (void)native_memory::tryReadField(node, 0x43, evidence.kind);
                // hkbNode ctor 193D7D0 initializes hkStringPtr at +38;
                // native subgraph installation 17762E0 reads the same name.
                if (native_memory::tryReadField(node, 0x38, name) && name &&
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(name & ~std::uintptr_t{1}),
                        evidence.name.data(), evidence.name.size())) {
                    evidence.name.back() = '\0';
                } else {
                    evidence.name.fill('\0');
                }
                return evidence;
            };
            const auto source = describe(from);
            const auto target = describe(to);
            void* character{};
            void* behavior{};
            float elapsed{};
            (void)native_memory::tryReadField(context, 0, character);
            (void)native_memory::tryReadField(context, 8, behavior);
            if (!behavior) (void)native_memory::tryReadField(character, 0x80, behavior);
            (void)native_memory::tryReadField(blend, 0x134, elapsed);
            const auto base = REL::Module::get().base();
            ROCK_LOG_DEBUG(Weapon,
                "Equip blend identity effect={} character={} behavior={} elapsed={:.6f} remaining={:.6f} outgoingDraw={} "
                "from={} kind={} vtableRva={:#x} name='{}' to={} kind={} vtableRva={:#x} name='{}'",
                blend, character, behavior, elapsed, remaining, outgoingDraw,
                from, source.kind, source.vtable >= base ? source.vtable - base : 0, source.name.data(),
                to, target.kind, target.vtable >= base ? target.vtable - base : 0, target.name.data());
        }

        void onBlendUpdate(void* blend, void* context, float timestep, void* syncOwner)
        {
            const auto lease = s_encodedLease.load(std::memory_order_acquire);
            float remaining{};
            void* from{};
            void* to{};
            std::uintptr_t vtable{};
            bool complete = false;
            // Restrict to the verified concrete blending effect, in the exact
            // leased player graph, with an actively registered outgoing equip
            // clip. Other actors and ordinary
            // reload/fire/locomotion transitions retain their original timestep.
            if (lease && (lease & kEncodedStateMask) ==
                    weapon_transition_animation_acceleration_policy::transitionState(Direction::Draw) &&
                weapon_transition_animation_acceleration_policy::shouldAccelerateSample(true, Direction::Draw,
                    f4vr::getNativeWeaponState(reinterpret_cast<RE::Actor*>(lease & ~kEncodedStateMask))) &&
                std::isfinite(timestep) && timestep > 0.0f && contextMatchesPlayerGraph(context) &&
                native_memory::tryReadField(blend, 0, vtable) && vtable == REL::Offset(0x2EE4F28).address() &&
                native_memory::tryReadField(blend, 0x130, remaining) && std::isfinite(remaining) && remaining > 0.0f &&
                native_memory::tryReadField(blend, 0x98, from) && native_memory::tryReadField(blend, 0xA0, to) && from && to && from != to) {
                const bool outgoingDraw = outgoingBranchHasDrawClip(context, from);
                traceBlend(blend, context, from, to, remaining, outgoingDraw);
                if (outgoingDraw && s_encodedLease.load(std::memory_order_acquire) == lease) {
                    timestep = (std::max)(timestep, remaining);
                    complete = true;
                }
            }
            // Native update owns its sync bookkeeping, elapsed/remaining clocks
            // and later graph cleanup. Never deactivate/free a clip ourselves.
            s_originalBlendUpdate(blend, context, timestep, syncOwner);
            if (complete) {
                float after{};
                const bool readable = native_memory::tryReadField(blend, 0x130, after);
                ROCK_LOG_SAMPLE_INFO(Weapon, 10,
                    "Equip blend completed from={} to={} remaining={:.6f}->{:.6f} readable={} timestep={:.6f}",
                    from, to, remaining, after, readable, timestep);
            }
        }

        __declspec(noinline) void onClipGeneratorActivate(
            void* clipGenerator,
            void* context)
        {
            if (s_originalClipActivate) {
                s_originalClipActivate(clipGenerator, context);
            }
            registerActivatedClip(clipGenerator, context);
        }

        __declspec(noinline) void onClipGeneratorUpdate(
            void* clipGenerator,
            void* context,
            const float timestep)
        {
            if (s_originalClipUpdate) {
                s_originalClipUpdate(
                    clipGenerator,
                    context,
                    acceleratedTimestep(clipGenerator, timestep));
            }
        }

        __declspec(noinline) void onClipGeneratorDeactivate(
            void* clipGenerator,
            void* context)
        {
            unregisterClip(clipGenerator);
            if (s_originalClipDeactivate) {
                s_originalClipDeactivate(clipGenerator, context);
            }
        }

        [[nodiscard]] float elapsedSeconds() noexcept
        {
            if (!s_runtimeLease.active) {
                return 0.0f;
            }
            return (std::max)(
                0.0f,
                std::chrono::duration<float>(
                    std::chrono::steady_clock::now() -
                    s_runtimeLease.requestedAt)
                    .count());
        }

        void resetDiagnostics() noexcept
        {
            s_activationCalls.store(0, std::memory_order_relaxed);
            s_matchedActivations.store(0, std::memory_order_relaxed);
            s_registeredClips.store(0, std::memory_order_relaxed);
            s_clipRegistryOverflows.store(0, std::memory_order_relaxed);
            s_registeredUpdateCalls.store(0, std::memory_order_relaxed);
            s_acceleratedSamples.store(0, std::memory_order_relaxed);
            s_lastOriginalTimestepBits.store(0, std::memory_order_relaxed);
            s_lastAcceleratedTimestepBits.store(0, std::memory_order_relaxed);
            s_animationThreadId.store(0, std::memory_order_relaxed);
            s_multipleAnimationThreads.store(false, std::memory_order_relaxed);
        }

        void clearLease(
            const char* reason,
            const std::uint32_t nativeState) noexcept
        {
            s_encodedLease.store(0, std::memory_order_release);
            if (!s_runtimeLease.active) {
                clearPublishedTargets();
                return;
            }

            const auto characterCount =
                s_targetCharacterCount.load(std::memory_order_acquire);
            const float lastOriginalTimestep = std::bit_cast<float>(
                s_lastOriginalTimestepBits.load(std::memory_order_relaxed));
            const float lastAcceleratedTimestep = std::bit_cast<float>(
                s_lastAcceleratedTimestepBits.load(
                    std::memory_order_relaxed));
            ROCK_LOG_INFO(
                Weapon,
                "Weapon transition clip acceleration ended sequence={} direction={} reason={} formID={:08X} instance={:#x} state={} elapsedMs={:.1f} characters={} activationCalls={} matchedActivations={} registeredClips={} registryOverflows={} registeredUpdates={} acceleratedSamples={} lastDt={:.5f} lastAcceleratedDt={:.5f} animationThread={} multipleAnimationThreads={}",
                s_runtimeLease.sequence,
                s_runtimeLease.direction == Direction::Draw ?
                    "draw" :
                    "sheathe",
                reason ? reason : "unknown",
                s_runtimeLease.identity.formID,
                s_runtimeLease.identity.instanceData,
                nativeState,
                elapsedSeconds() * 1000.0f,
                characterCount,
                s_activationCalls.load(std::memory_order_relaxed),
                s_matchedActivations.load(std::memory_order_relaxed),
                s_registeredClips.load(std::memory_order_relaxed),
                s_clipRegistryOverflows.load(std::memory_order_relaxed),
                s_registeredUpdateCalls.load(std::memory_order_relaxed),
                s_acceleratedSamples.load(std::memory_order_relaxed),
                lastOriginalTimestep,
                lastAcceleratedTimestep,
                s_animationThreadId.load(std::memory_order_relaxed),
                s_multipleAnimationThreads.load(
                    std::memory_order_relaxed) ?
                    "yes" :
                    "no");
            clearPublishedTargets();
            s_runtimeLease = {};
        }

        void logGraphTargetFailure(
            const GraphTargetResult result,
            const Identity& identity,
            const Direction direction) noexcept
        {
            if (s_resolveFailureLogs >= kMaxResolveFailureLogs) {
                return;
            }
            ++s_resolveFailureLogs;
            ROCK_LOG_WARN(
                Weapon,
                "Weapon transition clip acceleration not armed direction={} formID={:08X} instance={:#x}: {}",
                direction == Direction::Draw ? "draw" : "sheathe",
                identity.formID,
                identity.instanceData,
                result == GraphTargetResult::MissingManager ?
                    "player graph manager unavailable" :
                    "no verified player graph characters");
        }
    }

    bool install() noexcept
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(
                Init,
                "Weapon transition clip acceleration unavailable: unsupported runtime");
            return false;
        }

        void* activateOriginal =
            reinterpret_cast<void*>(s_originalClipActivate);
        const bool activateInstalled = entry_trampoline_hook::install(
            "hkbClipGenerator activate",
            kClipGeneratorActivate,
            kExpectedActivateEntry.data(),
            kExpectedActivateEntry.size(),
            reinterpret_cast<void*>(&onClipGeneratorActivate),
            activateOriginal);
        s_originalClipActivate =
            reinterpret_cast<ClipGeneratorActivateFn>(activateOriginal);

        void* updateOriginal = reinterpret_cast<void*>(s_originalClipUpdate);
        const bool updateInstalled = entry_trampoline_hook::install(
            "hkbClipGenerator update",
            kClipGeneratorUpdate,
            kExpectedUpdateEntry.data(),
            kExpectedUpdateEntry.size(),
            reinterpret_cast<void*>(&onClipGeneratorUpdate),
            updateOriginal);
        s_originalClipUpdate =
            reinterpret_cast<ClipGeneratorUpdateFn>(updateOriginal);

        void* deactivateOriginal =
            reinterpret_cast<void*>(s_originalClipDeactivate);
        const bool deactivateInstalled = entry_trampoline_hook::install(
            "hkbClipGenerator deactivate",
            kClipGeneratorDeactivate,
            kExpectedDeactivateEntry.data(),
            kExpectedDeactivateEntry.size(),
            reinterpret_cast<void*>(&onClipGeneratorDeactivate),
            deactivateOriginal);
        s_originalClipDeactivate =
            reinterpret_cast<ClipGeneratorDeactivateFn>(deactivateOriginal);

        const bool ready =
            activateInstalled && updateInstalled && deactivateInstalled &&
            s_originalClipActivate && s_originalClipUpdate &&
            s_originalClipDeactivate;
        s_encodedLease.store(0, std::memory_order_release);
        clearPublishedTargets();
        s_ownerThreadId.store(0, std::memory_order_release);
        s_installed.store(ready, std::memory_order_release);
        if (!ready) {
            ROCK_LOG_ERROR(
                Init,
                "Weapon transition clip acceleration disabled: all three lifecycle entry hooks are required");
        } else {
            // Update wrapper 141F79FA0 tail-calls this common implementation
            // with R9=self. Prefix ends before its first relative instruction.
            constexpr std::array<std::uint8_t, 15> prefix{
                0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x40 };
            constexpr std::array<std::uint8_t, 14> clonePrefix{
                0x48,0x8B,0x89,0x18,0x01,0x00,0x00,0x48,0x85,0xC9,0x74,0x08,0x45,0x33 };
            std::array<std::uint8_t, 14> actual{};
            void* original{};
            const bool verified = native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(0x17BE290).address()), actual.data(), actual.size()) && actual == clonePrefix;
            if (verified && entry_trampoline_hook::install("Scoped outgoing equip blend completion", 0x1F7AD90,
                    prefix.data(), prefix.size(), reinterpret_cast<void*>(&onBlendUpdate), original)) {
                s_originalBlendUpdate = reinterpret_cast<BlendUpdateFn>(original);
                ROCK_LOG_INFO(Init, "Outgoing equip blend completion installed for leased player draw clips");
            } else {
                ROCK_LOG_WARN(Init, "Outgoing equip blend completion unavailable; native blend timing retained");
            }
        }
        return ready;
    }

    RequestResult request(const RequestInput& input) noexcept
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return RequestResult::NotInstalled;
        }
        if (!input.player || !input.identity.valid()) {
            return RequestResult::MissingInput;
        }
        if (!claimOrValidateOwnerThread()) {
            return RequestResult::WrongThread;
        }

        const auto encodedLease = encodeLease(input.player, input.direction);
        if (!encodedLease) {
            return RequestResult::UnencodablePlayer;
        }
        if (s_runtimeLease.active &&
            s_runtimeLease.player == input.player &&
            s_runtimeLease.identity == input.identity &&
            s_runtimeLease.direction == input.direction &&
            s_encodedLease.load(std::memory_order_acquire) == encodedLease) {
            return RequestResult::AlreadyArmed;
        }
        if (s_runtimeLease.active) {
            clearLease("superseded", kInvalidNativeState);
        } else {
            s_encodedLease.store(0, std::memory_order_release);
            clearPublishedTargets();
        }

        const auto graphTargets =
            publishPlayerGraphCharacters(input.player);
        if (graphTargets != GraphTargetResult::Ready) {
            logGraphTargetFailure(
                graphTargets,
                input.identity,
                input.direction);
            return graphTargets == GraphTargetResult::MissingManager ?
                RequestResult::MissingGraphManager :
                RequestResult::MissingGraphCharacters;
        }

        resetDiagnostics();
        s_runtimeLease = RuntimeLease{
            .player = input.player,
            .identity = input.identity,
            .direction = input.direction,
            .requestedAt = std::chrono::steady_clock::now(),
            .sequence = ++s_nextSequence,
            .active = true,
        };
        s_encodedLease.store(encodedLease, std::memory_order_release);
        ROCK_LOG_INFO(
            Weapon,
            "Weapon transition clip acceleration armed sequence={} direction={} formID={:08X} instance={:#x} equipIndex={} characters={} multiplier={:.1f}",
            s_runtimeLease.sequence,
            input.direction == Direction::Draw ? "draw" : "sheathe",
            input.identity.formID,
            input.identity.instanceData,
            input.identity.equipIndex,
            s_targetCharacterCount.load(std::memory_order_acquire),
            weapon_transition_animation_acceleration_policy::
                kAcceleratedSpeedMultiplier);
        return RequestResult::Armed;
    }

    ActivationEvidence observeExactLeaseActivation(
        RE::PlayerCharacter* player,
        const Identity& identity,
        const Direction direction) noexcept
    {
        ActivationEvidence evidence{};
        const DWORD ownerThreadId =
            s_ownerThreadId.load(std::memory_order_acquire);
        if (!s_installed.load(std::memory_order_acquire) || !player ||
            !identity.valid() || ownerThreadId == 0 ||
            ownerThreadId != GetCurrentThreadId() ||
            !s_runtimeLease.active || s_runtimeLease.player != player ||
            s_runtimeLease.identity != identity ||
            s_runtimeLease.direction != direction ||
            s_encodedLease.load(std::memory_order_acquire) == 0) {
            return evidence;
        }

        for (std::size_t index = 0;
             index < s_transitionClips.size();
             ++index) {
            const auto clipIdentity = s_transitionClips[index].load(
                std::memory_order_acquire);
            if (clipIdentity != 0 &&
                evidence.activeClips <
                    evidence.activeClipIdentities.size()) {
                evidence.activeClipIdentities[evidence.activeClips++] =
                    clipIdentity;
            }
            const auto updatedIdentity =
                s_updatedTransitionClips[index].load(
                    std::memory_order_acquire);
            if (updatedIdentity == clipIdentity && updatedIdentity != 0 &&
                evidence.updatedActiveClips <
                    evidence.updatedActiveClipIdentities.size()) {
                evidence.updatedActiveClipIdentities
                    [evidence.updatedActiveClips++] = updatedIdentity;
            }
        }
        evidence.sequence = s_runtimeLease.sequence;
        evidence.matchedActivations =
            s_matchedActivations.load(std::memory_order_acquire);
        evidence.registeredUpdates =
            s_registeredUpdateCalls.load(std::memory_order_acquire);
        evidence.exactLease = true;
        return evidence;
    }

    void service(const ServiceInput& input) noexcept
    {
        if (!s_runtimeLease.active) {
            return;
        }
        if (!claimOrValidateOwnerThread()) {
            s_encodedLease.store(0, std::memory_order_release);
            return;
        }

        const bool identityMatches =
            input.player == s_runtimeLease.player &&
            input.identity == s_runtimeLease.identity;
        const float elapsed = elapsedSeconds();
        const auto action =
            weapon_transition_animation_acceleration_policy::
                classifyLifecycle(
                    input.runtimeAllowed,
                    identityMatches,
                    s_runtimeLease.direction,
                    input.nativeWeaponState,
                    elapsed);
        switch (action) {
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Complete:
            clearLease(s_runtimeLease.direction == Direction::Draw ? "draw-clip-window-complete" : "native-complete", input.nativeWeaponState);
            break;
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Cancel: {
            const char* reason = !input.runtimeAllowed ?
                "runtime-unavailable" :
                !identityMatches ?
                    "identity-changed" :
                    !held_weapon_equip_state_policy::
                            isValidNativeWeaponState(
                                input.nativeWeaponState) ?
                        "invalid-native-state" :
                        "watchdog-timeout";
            clearLease(reason, input.nativeWeaponState);
            break;
        }
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::Accelerate:
        case weapon_transition_animation_acceleration_policy::
            LifecycleAction::KeepPending:
        default:
            break;
        }
    }

    void cancel(const char* reason) noexcept
    {
        s_encodedLease.store(0, std::memory_order_release);
        if (!s_runtimeLease.active) {
            clearPublishedTargets();
            return;
        }
        const DWORD ownerThreadId =
            s_ownerThreadId.load(std::memory_order_acquire);
        if (ownerThreadId != 0 && ownerThreadId != GetCurrentThreadId()) {
            ROCK_LOG_ERROR(
                Weapon,
                "Weapon transition clip acceleration cancellation arrived on non-owner thread owner={} caller={}; atomic lease disabled and runtime record retained for owner cleanup",
                ownerThreadId,
                GetCurrentThreadId());
            return;
        }
        clearLease(reason, kInvalidNativeState);
    }
}
