// Loot to Hand: an item taken one at a time through the quick-loot view (a wand
// pointing at a container, no menu open) lands in that hand. The full
// ContainerMenu is for bulk looting and keeps the game behaviour: items go to
// inventory.
//
// Admission: container -> player, a wand's pick target is that container, and
// neither ContainerMenu nor BarterMenu is open. Script transfers from
// containers have no wand pointing at the source, so they stay in inventory.
//
// Thread model: TESContainerChangedEvent is dispatched on the game thread; the
// queue is drained on ROCK's frame thread by
// PhysicsInteraction::serviceLootToHand on the next frame. The mutex guards
// only the fixed queue below (no allocation, no engine calls under the lock).
// The drop goes through the same inventory-transfer commit as grenade
// quick-draw: on failure the item returns to inventory.
#include "physics-interaction/core/interaction/LootToHand.h"

#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/input/InputRemapRuntime.h"

#include <mutex>

namespace rock::loot_to_hand
{
    namespace
    {
        constexpr std::uint32_t kPlayerFormId = 0x14;

        std::mutex s_queueMutex;
        std::array<LootedItem, kQueueCapacity> s_queue{};
        std::size_t s_queueSize{ 0 };
        std::atomic<bool> s_registered{ false };
        std::atomic<bool> s_missingSourceLogged{ false };

        [[nodiscard]] bool menuOpen(RE::UI* ui, std::string_view name)
        {
            return ui && ui->GetMenuOpen(RE::BSFixedString(name.data()));
        }

        class ContainerChangedSink final : public RE::BSTEventSink<RE::TESContainerChangedEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::TESContainerChangedEvent& event,
                RE::BSTEventSource<RE::TESContainerChangedEvent>*) override
            {
                const bool taken = event.baseObjectFormID != 0 && event.oldContainerFormID != 0 &&
                    event.oldContainerFormID != kPlayerFormId && event.newContainerFormID == kPlayerFormId;
                if (!g_rockConfig.rockLootToHand || !taken) {
                    return RE::BSEventNotifyControl::kContinue;
                }
                auto* ui = RE::UI::GetSingleton();
                const bool containerMenu = menuOpen(ui, "ContainerMenu");
                const bool barterMenu = menuOpen(ui, "BarterMenu");
                const auto primaryTarget = input_remap_runtime::wandPickRefFormId(true);
                const auto secondaryTarget = input_remap_runtime::wandPickRefFormId(false);
                const bool primaryWand = primaryTarget == event.oldContainerFormID;
                const bool secondaryWand = secondaryTarget == event.oldContainerFormID;
                const bool admitted = !containerMenu && !barterMenu && (primaryWand || secondaryWand);
                // Player takes are rare (no per-frame volume); one line each shows which
                // condition admits or rejects the item.
                ROCK_LOG_INFO(Hand,
                    "Loot to hand event: item={:08X} count={} from={:08X} uniqueID={} containerMenu={} barterMenu={} "
                    "primaryWand={:08X} secondaryWand={:08X} admitted={}",
                    event.baseObjectFormID, event.itemCount, event.oldContainerFormID, event.uniqueID, containerMenu,
                    barterMenu, primaryTarget, secondaryTarget, admitted);
                if (!admitted) {
                    return RE::BSEventNotifyControl::kContinue;
                }
                // The primary wand is the primary hand (right, or left in left-handed mode).
                const bool primaryIsLeft = g_rockConfig.rockDefaultHandLeft;
                const bool preferLeft = primaryWand ? primaryIsLeft : !primaryIsLeft;
                bool queued = false;
                {
                    const std::scoped_lock lock{ s_queueMutex };
                    if (s_queueSize < s_queue.size()) {
                        s_queue[s_queueSize++] = { event.baseObjectFormID, event.uniqueID, preferLeft };
                        queued = true;
                    }
                }
                ROCK_LOG_INFO(Hand, "Loot to hand {}: item={:08X} count={} from={:08X} uniqueID={}",
                    queued ? "queued" : "queue full, stays in inventory", event.baseObjectFormID, event.itemCount,
                    event.oldContainerFormID, event.uniqueID);
                return RE::BSEventNotifyControl::kContinue;
            }
        };

        ContainerChangedSink s_sink;
    }

    void ensureRegistered()
    {
        bool expected = false;
        if (!s_registered.compare_exchange_strong(expected, true, std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            return;
        }
        auto* source = RE::TESContainerChangedEvent::GetEventSource();
        if (!source) {
            s_registered.store(false, std::memory_order_release);
            if (!s_missingSourceLogged.exchange(true, std::memory_order_acq_rel)) {
                ROCK_LOG_WARN(Hand, "Container-changed event source unavailable; Loot to Hand will retry");
            }
            return;
        }
        source->RegisterSink(&s_sink);
        ROCK_LOG_INFO(Hand, "Registered Loot to Hand container-changed sink");
    }

    std::size_t drain(std::array<LootedItem, kQueueCapacity>& out)
    {
        const std::scoped_lock lock{ s_queueMutex };
        const auto count = s_queueSize;
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = s_queue[i];
        }
        s_queueSize = 0;
        return count;
    }
}

namespace rock
{
    void PhysicsInteraction::serviceLootToHand(const PhysicsFrameContext& frame)
    {
        if (frame.menuBlocked || !frame.worldReady || !frame.bhkWorld || !frame.hknpWorld ||
            !physicsWritesAllowedForWorld(frame.hknpWorld)) {
            return; // A menu is open or the world is not ready: keep the queue.
        }
        std::array<loot_to_hand::LootedItem, loot_to_hand::kQueueCapacity> items{};
        const auto count = loot_to_hand::drain(items);
        if (count > 0) {
            ROCK_LOG_INFO(Hand, "Loot to hand: servicing {} queued item(s)", count);
        }
        for (std::size_t i = 0; i < count; ++i) {
            const auto& item = items[i];
            int chosen = -1; // 0 right, 1 left
            for (const bool isLeft : { item.preferLeft, !item.preferLeft }) {
                const auto index = isLeft ? 1u : 0u;
                const auto& hand = isLeft ? _leftHand : _rightHand;
                const auto& input = isLeft ? frame.left : frame.right;
                if (!_forceGrab.pendingCommits[index].active &&
                    forceGrabHandBlockerMask(hand, isLeft, input.disabled, true) == 0) {
                    chosen = static_cast<int>(index);
                    break;
                }
            }
            if (chosen < 0) {
                ROCK_LOG_INFO(Hand, "Loot to hand: no free hand, item={:08X} stays in inventory", item.baseFormId);
                continue;
            }
            const bool isLeft = chosen == 1;
            auto location = (isLeft ? frame.left : frame.right).grabAnchorWorld;
            location.z -= 3.0f;
            const auto drop = loose_grenade_runtime::dropLootedItemToWorld(item.baseFormId, item.uniqueId, location);
            if (!drop.success) {
                ROCK_LOG_INFO(Hand, "Loot to hand: item={:08X} stays in inventory ({})", item.baseFormId, drop.reason);
                continue;
            }
            _forceGrab.pendingCommits[static_cast<std::size_t>(chosen)] = PendingForceGrabCommit{
                .active = true,
                .isLeft = isLeft,
                .phase = PendingForceGrabCommitPhase::WaitingForReference,
                .targetHandle = drop.handle,
                .targetIsLooseThrowable = drop.droppedRef && loose_grenade_runtime::isThrowableRef(drop.droppedRef),
                .inventoryTransfer = true,
                .lootToHand = true,
                .maxDistanceGame = 96.0f,
            };
            ROCK_LOG_INFO(Hand, "Loot to hand: item={:08X} ref={:08X} stack={} hand={}", item.baseFormId,
                drop.droppedRef ? drop.droppedRef->GetFormID() : 0, drop.stackId, isLeft ? "left" : "right");
        }
    }
}
