#include "physics-interaction/weapon/WorldWeaponMaterialVisibility.h"
#include "physics-interaction/weapon/WorldWeaponVisibilityEvents.h"
#include "physics-interaction/weapon/WeaponMaterialVisibility.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/TESCellAttachDetachEvent.h"
#include "RE/Bethesda/TESForms.h"
#include "REL/Relocation.h"

#include <algorithm>
#include <memory>

namespace rock::world_weapon_material_visibility
{
    namespace
    {
        bool isWeapon(const RE::TESObjectREFR* reference)
        {
            const auto* base = reference ? reference->GetObjectReference() : nullptr;
            return base && base->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        class Runtime final : public RE::BSTEventSink<RE::TESCellAttachDetachEvent>
        {
        public:
            ~Runtime() override
            {
                if (_registered) {
                    // Native unregister, paired with RegisterForCellAttachDetach:
                    // 14041D1F0/14041D250 both obtain source 14042A610, then
                    // call native register 140451840 / unregister 14045C670.
                    static REL::Relocation<void (*)(RE::BSTEventSink<RE::TESCellAttachDetachEvent>*)>
                        unregister{ REL::Offset(0x41D250) };
                    unregister(this);
                }
                reset();
            }

            void install()
            {
                if (_registered) return;
                RE::RegisterForCellAttachDetach(this);
                _registered = true;
                ROCK_LOG_INFO(Weapon, "World weapon material visibility: attached-reference tracking installed");
            }

            void suspend()
            {
                for (std::size_t i = 0; i < _count; ++i) _slots[i].clearCulls();
            }

            void reset()
            {
                suspend();
                for (std::size_t i = 0; i < _count; ++i) _slots[i] = {};
                _count = 0;
                _cursor = 0;
                _pending.discard();
            }

            void update(bool canCull)
            {
                if (_overflow.exchange(false, std::memory_order_relaxed)) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "World weapon material visibility event capacity exhausted");
                }
                AttachmentEvent event;
                for (unsigned i = 0; i < 64 && _pending.pop(event); ++i) apply(event);
                if (!canCull) return;

                // Only references announced by the engine are visited. Bound
                // work per frame, including opaque weapons and loading graphs.
                const auto visits = (std::min)(_count, std::size_t{8});
                for (std::size_t i = 0; i < visits && _count; ++i) {
                    _cursor %= _count;
                    auto& slot = _slots[_cursor];
                    const auto reference = slot.handle.get();
                    if (!reference || reference->IsDeleted() || reference->IsDisabled() || !isWeapon(reference.get())) {
                        erase(_cursor);
                        continue;
                    }
                    const RE::NiPointer<RE::NiAVObject> root{ reference->Get3D() };
                    if (!root) {
                        // An attach notification can precede 3D readiness. Keep
                        // its weak handle until ready or explicitly detached.
                        slot.clearCulls();
                        ++_cursor;
                        continue;
                    }
                    if (!slot.visibility) slot.visibility = std::make_unique<weapon_material_visibility::State>();
                    const std::array<RE::NiAVObject*, 1> roots{ root.get() };
                    if (slot.visibility->update(roots, reference->GetObjectReference()->GetFormID(), false)) {
                        ROCK_LOG_INFO(Weapon, "World weapon material visibility updated: reference={:08X} weapon={:08X}",
                            slot.referenceID, reference->GetObjectReference()->GetFormID());
                    }
                    ++_cursor;
                }
            }

        private:
            RE::BSEventNotifyControl ProcessEvent(const RE::TESCellAttachDetachEvent& event,
                RE::BSTEventSource<RE::TESCellAttachDetachEvent>*) override
            {
                // 14041D280 retains the reference while notifying; +0 is the
                // NiPointer, +8 the attached byte. Callers 1403AA39A (true) and
                // 1403AA628 (false) bracket native attachment/detachment. Read
                // immutable form identity only; never touch 3D on this thread.
                if (isWeapon(event.reference.get()) &&
                    !_pending.push({ event.reference->GetFormID(), event.attached }))
                    _overflow.store(true, std::memory_order_relaxed);
                return RE::BSEventNotifyControl::kContinue;
            }

            void apply(AttachmentEvent event)
            {
                std::size_t index = 0;
                while (index < _count && _slots[index].referenceID != event.referenceID) ++index;
                if (!event.attached) {
                    if (index < _count) erase(index);
                    return;
                }
                auto* reference = RE::TESForm::GetFormByID<RE::TESObjectREFR>(event.referenceID);
                if (!isWeapon(reference) || reference->IsDeleted() || reference->IsDisabled()) return;
                const auto handle = reference->GetHandle();
                if (!handle) return;
                if (index < _count && _slots[index].handle == handle) return;
                if (index == _slots.size()) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "World weapon material visibility reference capacity exhausted: {}", _count);
                    return;
                }
                auto& slot = _slots[index];
                slot.clearCulls();
                slot.referenceID = event.referenceID;
                slot.handle = handle;
                if (index == _count) ++_count;
            }

            void erase(std::size_t index)
            {
                _slots[index].clearCulls();
                --_count;
                if (index != _count) _slots[index] = std::move(_slots[_count]);
                _slots[_count] = {};
            }

            struct Slot
            {
                std::uint32_t referenceID = 0;
                RE::ObjectRefHandle handle;
                std::unique_ptr<weapon_material_visibility::State> visibility;

                void clearCulls()
                {
                    if (visibility) visibility->clear();
                    visibility.reset();
                }
            };
            PendingEvents _pending;
            std::atomic<bool> _overflow{ false };
            std::array<Slot, 1024> _slots{};
            std::size_t _count = 0, _cursor = 0;
            bool _registered = false;
        };

        Runtime& runtime()
        {
            // Constructed at GameLoaded, before references attach. Registration
            // lasts for the plugin; scene ownership is cleared on provider loss
            // and before save loads, independently of the retained event sink.
            static Runtime instance;
            return instance;
        }
    }

    void install()
    {
        // Initialize the native source before constructing our static owner;
        // the owner unregisters before the source's registered exit cleanup.
        using Source = RE::BSTEventSource<RE::TESCellAttachDetachEvent>;
        static REL::Relocation<Source* (*)()> source{ REL::Offset(0x42A610) };
        (void)source();
        runtime().install();
    }
    void suspend() { runtime().suspend(); }
    void reset() { runtime().reset(); }
    void update(bool canCull)
    {
        try {
            runtime().update(canCull);
        } catch (const std::exception& error) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "World weapon material visibility update failed: {}", error.what());
        } catch (...) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "World weapon material visibility update failed");
        }
    }
}
