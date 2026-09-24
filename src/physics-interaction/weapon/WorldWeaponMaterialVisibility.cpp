#include "physics-interaction/weapon/WorldWeaponMaterialVisibility.h"
#include "physics-interaction/weapon/WorldWeaponVisibilityEvents.h"
#include "physics-interaction/weapon/WeaponMaterialVisibility.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
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
                    _registered = false;
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
                if (_registered) {
                    ROCK_LOG_INFO(Weapon,
                        "World weapon visibility reset: callbacks={} weaponEvents={} drained={} tracked={}",
                        _callbacks.load(std::memory_order_relaxed), _weaponEvents.load(std::memory_order_relaxed), _drained, _count);
                }
                suspend();
                for (std::size_t i = 0; i < _count; ++i) _slots[i] = {};
                _count = 0;
                _cursor = 0;
                _pending.discard();
                _traceFrames = 0;
                _everReady = false;
                _detailedReferences = 0;
            }

            void update(bool canCull)
            {
                if (_overflow.exchange(false, std::memory_order_relaxed)) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "World weapon material visibility event capacity exhausted");
                }
                AttachmentEvent event;
                for (unsigned i = 0; i < 64 && _pending.pop(event); ++i) {
                    ++_drained;
                    if (event.attached) ++_attached;
                    else ++_detached;
                    apply(event);
                }
                for (std::size_t i = 0; i < _eventSamples.size(); ++i) {
                    const auto sample = _eventSamples[i].load(std::memory_order_acquire);
                    if (sample && !_samplePrinted[i]) {
                        _samplePrinted[i] = true;
                        ROCK_LOG_INFO(Weapon, "World weapon event sample: reference={:08X} baseType={} attached={}",
                            static_cast<std::uint32_t>(sample), static_cast<unsigned>(sample >> 33), (sample & (1ull << 32)) != 0);
                    }
                }
                if (!canCull) {
                    trace(false);
                    return;
                }

                // Only references announced by the engine are visited. Bound
                // work per frame, including opaque weapons and loading graphs.
                const auto visits = (std::min)(_count, std::size_t{8});
                for (std::size_t i = 0; i < visits && _count; ++i) {
                    _cursor %= _count;
                    auto& slot = _slots[_cursor];
                    const auto reference = slot.handle.get();
                    if (!reference || reference->IsDeleted() || reference->IsDisabled() || !isWeapon(reference.get())) {
                        ++_expired;
                        erase(_cursor);
                        continue;
                    }
                    const RE::NiPointer<RE::NiAVObject> root{ reference->Get3D() };
                    if (!root) {
                        ++_rootMissing;
                        // An attach notification can precede 3D readiness. Keep
                        // its weak handle until ready or explicitly detached.
                        slot.clearCulls();
                        ++_cursor;
                        continue;
                    }
                    ++_rootReady;
                    if (!slot.rootReported && _detailedReferences < 8) {
                        slot.rootReported = true;
                        slot.traceDetails = true;
                        ++_detailedReferences;
                        std::size_t triangles = 0, geometry = 0;
                        const auto traversal = weapon_scene::visitScene(root.get(), [&](RE::NiAVObject* node) {
                            triangles += node->IsTriShape() != nullptr;
                            geometry += node->IsGeometry() != nullptr;
                            return true;
                        });
                        ROCK_LOG_INFO(Weapon,
                            "World weapon root: reference={:08X} weapon={:08X} root={:X} name='{}' nodes={} geometry={} triShapes={} truncated={} appCulled={} scale={}",
                            slot.referenceID, reference->GetObjectReference()->GetFormID(), reinterpret_cast<std::uintptr_t>(root.get()),
                            root->name.c_str(), traversal.visited, geometry, triangles, traversal.truncated, root->GetAppCulled(), root->local.scale);
                    }
                    if (!slot.visibility) slot.visibility = std::make_unique<weapon_material_visibility::State>();
                    const std::array<RE::NiAVObject*, 1> roots{ root.get() };
                    if (slot.visibility->update(roots, reference->GetObjectReference()->GetFormID(), slot.traceDetails)) {
                        ++_changed;
                        ROCK_LOG_INFO(Weapon, "World weapon material visibility updated: reference={:08X} weapon={:08X}",
                            slot.referenceID, reference->GetObjectReference()->GetFormID());
                    }
                    ++_cursor;
                }
                trace(true);
            }

        private:
            RE::BSEventNotifyControl ProcessEvent(const RE::TESCellAttachDetachEvent& event,
                RE::BSTEventSource<RE::TESCellAttachDetachEvent>*) override
            {
                // 14041D280 retains the reference while notifying; +0 is the
                // NiPointer, +8 the attached byte. Callers 1403AA39A (true) and
                // 1403AA628 (false) bracket native attachment/detachment. Read
                // immutable form identity only; never touch 3D on this thread.
                const auto callback = _callbacks.fetch_add(1, std::memory_order_relaxed);
                if (callback < _eventSamples.size() && event.reference) {
                    const auto* base = event.reference->GetObjectReference();
                    const auto type = base ? static_cast<unsigned>(base->GetFormType()) : 0xFFu;
                    _eventSamples[callback].store(static_cast<std::uint64_t>(event.reference->GetFormID()) |
                        (static_cast<std::uint64_t>(event.attached) << 32) | (static_cast<std::uint64_t>(type) << 33),
                        std::memory_order_release);
                }
                if (isWeapon(event.reference.get())) {
                    _weaponEvents.fetch_add(1, std::memory_order_relaxed);
                    if (!_pending.push({ event.reference->GetFormID(), event.attached }))
                        _overflow.store(true, std::memory_order_relaxed);
                }
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
                if (!isWeapon(reference) || reference->IsDeleted() || reference->IsDisabled()) {
                    if (++_rejectedReferences <= 8) {
                        ROCK_LOG_INFO(Weapon, "World weapon admission rejected: reference={:08X} resolved={} weapon={} deleted={} disabled={}",
                            event.referenceID, reference != nullptr, isWeapon(reference), reference && reference->IsDeleted(), reference && reference->IsDisabled());
                    }
                    return;
                }
                const auto handle = reference->GetHandle();
                if (!handle) { ++_missingHandle; return; }
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
                ++_admitted;
            }

            void trace(bool canCull)
            {
                if (canCull && !_everReady) { _everReady = true; _traceFrames = 0; }
                if (_traceFrames < 601) ++_traceFrames;
                if (_traceFrames != 1 && _traceFrames != 120 && _traceFrames != 600) return;
                ROCK_LOG_INFO(Weapon,
                    "World weapon visibility trace revision=1 frame={} canCull={} callbacks={} weaponEvents={} drained={} attached={} detached={} admitted={} rejected={} noHandle={} tracked={} rootReady={} rootMissing={} expired={} changed={}",
                    _traceFrames, canCull, _callbacks.load(std::memory_order_relaxed), _weaponEvents.load(std::memory_order_relaxed),
                    _drained, _attached, _detached, _admitted, _rejectedReferences, _missingHandle, _count, _rootReady, _rootMissing, _expired, _changed);
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
                bool rootReported = false;
                bool traceDetails = false;

                void clearCulls()
                {
                    if (visibility) visibility->clear();
                    visibility.reset();
                }
            };
            PendingEvents _pending;
            std::atomic<bool> _overflow{ false };
            // Event-thread counters/samples contain values only. All formatting,
            // reference resolution and bounded scene traces run on the game thread.
            std::atomic<std::uint64_t> _callbacks{ 0 }, _weaponEvents{ 0 };
            std::array<std::atomic<std::uint64_t>, 8> _eventSamples{};
            std::array<bool, 8> _samplePrinted{};
            std::uint64_t _drained = 0, _attached = 0, _detached = 0, _admitted = 0;
            std::uint64_t _rejectedReferences = 0, _missingHandle = 0, _rootReady = 0, _rootMissing = 0, _expired = 0, _changed = 0;
            unsigned _traceFrames = 0, _detailedReferences = 0;
            bool _everReady = false;
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
