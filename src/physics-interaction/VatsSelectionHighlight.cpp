#include "VatsSelectionHighlight.h"

#include "PhysicsLog.h"
#include "SelectionHighlightPolicy.h"

#include "RE/Bethesda/BSTSmartPointer.h"
#include "RE/Bethesda/MemoryManager.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <REL/Relocation.h>

#include <cstdlib>
#include <cstddef>
#include <cstdint>

namespace frik::rock
{
    namespace detail
    {
        /*
         * This is a deliberately narrow local mirror of the FO4VR
         * VatsEffectTarget allocation contract. CommonLibF4VR does not expose
         * the class, but Ghidra verifies a 0xA8 object with BSIntrusiveRefCounted
         * at offset 0, constructed by REL::ID(820921), destroyed by REL::ID(1073050),
         * and retained/released by VatsEffectControl's target array.
         */
        struct VatsEffectTarget : RE::BSIntrusiveRefCounted
        {
            VatsEffectTarget() = default;

            ~VatsEffectTarget()
            {
                using dtor_t = void (*)(VatsEffectTarget*);
                static REL::Relocation<dtor_t> dtor{ REL::ID(1073050) };
                dtor(this);
            }

            [[nodiscard]] void* operator new(std::size_t size)
            {
                if (auto* memory = RE::malloc(size)) {
                    return memory;
                }

                std::abort();
            }

            void operator delete(void* memory) noexcept
            {
                RE::free(memory);
            }

            void operator delete(void* memory, std::size_t) noexcept
            {
                RE::free(memory);
            }

            [[nodiscard]] std::uint32_t targetObjectCount() const noexcept
            {
                return *reinterpret_cast<const std::uint32_t*>(reinterpret_cast<const std::byte*>(this) + 0x30);
            }

            std::byte storage[0xA8 - sizeof(RE::BSIntrusiveRefCounted)]{};
        };

        static_assert(sizeof(VatsEffectTarget) == 0xA8);
        static_assert(offsetof(VatsEffectTarget, refCount) == 0);
    }

    namespace
    {
        using TargetSmartPointer = RE::BSTSmartPointer<detail::VatsEffectTarget>;

        std::uint32_t s_activeRockVatsTargets = 0;

        detail::VatsEffectTarget* constructTarget(RE::NiAVObject* root3D)
        {
            using ctor_t = detail::VatsEffectTarget* (*)(detail::VatsEffectTarget*, RE::NiAVObject*);
            static REL::Relocation<ctor_t> ctor{ REL::ID(820921) };

            auto* target = new detail::VatsEffectTarget();
            return ctor(target, root3D);
        }

        void enableVatsEffectControlForWorldRollover()
        {
            using enable_t = void (*)(bool, bool);
            static REL::Relocation<enable_t> enable{ REL::ID(499745) };
            enable(selection_highlight_policy::kVatsHighlightEnableImageSpaceEffect,
                selection_highlight_policy::kVatsHighlightUseObjectRolloverState);
        }

        void disableVatsEffectControl(bool clearTargets)
        {
            using disable_t = void (*)(bool);
            static REL::Relocation<disable_t> disable{ REL::ID(845361) };
            disable(clearTargets);
        }

        void addTarget(TargetSmartPointer& target)
        {
            using add_t = int (*)(TargetSmartPointer&);
            static REL::Relocation<add_t> add{ REL::ID(126855) };
            add(target);
        }

        void removeTarget(TargetSmartPointer& target)
        {
            using remove_t = int (*)(TargetSmartPointer&);
            static REL::Relocation<remove_t> remove{ REL::ID(1335846) };
            remove(target);
        }

        void releaseOwnedTarget(detail::VatsEffectTarget* target)
        {
            if (!target) {
                return;
            }

            if (target->DecRef() == 0) {
                delete target;
            }
        }
    }

    VatsSelectionHighlight::~VatsSelectionHighlight()
    {
        stop();
    }

    bool VatsSelectionHighlight::play(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName)
    {
        const bool sameAsCurrent = isActiveFor(refr, targetNode);
        if (!selection_highlight_policy::shouldStartVatsHighlight(enabled, refr != nullptr && targetNode != nullptr, sameAsCurrent)) {
            if (!enabled || !refr || !targetNode) {
                stop();
            }
            if (enabled && refr && !targetNode) {
                ROCK_LOG_DEBUG(Hand, "{} selection highlight skipped: no visual node for reference 0x{:08X}",
                    handName ? handName : "Hand",
                    refr->formID);
            }
            return sameAsCurrent;
        }

        stop();

        auto* target = constructTarget(targetNode);
        if (!target) {
            ROCK_LOG_WARN(Hand, "{} selection highlight target allocation failed for reference 0x{:08X} source={}",
                handName ? handName : "Hand",
                refr->formID,
                targetSourceName ? targetSourceName : "unknown");
            return false;
        }

        target->IncRef();

        if (target->targetObjectCount() == 0) {
            ROCK_LOG_DEBUG(Hand, "{} selection highlight skipped: VatsEffectTarget has no target objects for reference 0x{:08X} source={}",
                handName ? handName : "Hand",
                refr->formID,
                targetSourceName ? targetSourceName : "unknown");
            releaseOwnedTarget(target);
            return false;
        }

        enableVatsEffectControlForWorldRollover();

        TargetSmartPointer targetRef(target);
        addTarget(targetRef);
        ++s_activeRockVatsTargets;

        _target = target;
        _highlightedRef = refr;
        _highlightedNode = targetNode;

        ROCK_LOG_DEBUG(Hand, "{} selection highlight attached through whole-object VatsEffectTarget for reference 0x{:08X} source={} targetObjects={} rolloverFlag={}",
            handName ? handName : "Hand",
            refr->formID,
            targetSourceName ? targetSourceName : "unknown",
            target->targetObjectCount(),
            selection_highlight_policy::kVatsHighlightUseObjectRolloverState ? "true" : "false");
        return true;
    }

    bool VatsSelectionHighlight::refresh(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName)
    {
        if (!enabled || !refr || !targetNode) {
            stop();
            return false;
        }

        if (!isActiveFor(refr, targetNode)) {
            return play(refr, targetNode, enabled, handName, targetSourceName);
        }

        if (_target->targetObjectCount() == 0) {
            ROCK_LOG_DEBUG(Hand, "{} selection highlight refresh rebuilding empty VatsEffectTarget for reference 0x{:08X} source={}",
                handName ? handName : "Hand",
                refr->formID,
                targetSourceName ? targetSourceName : "unknown");
            stop();
            return play(refr, targetNode, enabled, handName, targetSourceName);
        }

        enableVatsEffectControlForWorldRollover();

        ROCK_LOG_TRACE(Hand, "{} selection highlight refreshed for reference 0x{:08X} source={} targetObjects={}",
            handName ? handName : "Hand",
            refr->formID,
            targetSourceName ? targetSourceName : "unknown",
            _target->targetObjectCount());
        return true;
    }

    bool VatsSelectionHighlight::refreshActive(bool enabled, const char* handName)
    {
        if (!enabled || !_highlightedRef || !_highlightedNode) {
            stop();
            return false;
        }

        return refresh(_highlightedRef, _highlightedNode, enabled, handName, "active");
    }

    void VatsSelectionHighlight::stop()
    {
        auto* target = _target;
        auto* highlightedRef = _highlightedRef;
        _target = nullptr;
        _highlightedRef = nullptr;
        _highlightedNode = nullptr;

        if (!target) {
            return;
        }

        std::uint32_t remainingRockTargets = 0;
        if (s_activeRockVatsTargets > 0) {
            remainingRockTargets = --s_activeRockVatsTargets;
        }

        if (remainingRockTargets == 0) {
            disableVatsEffectControl(true);
        } else {
            TargetSmartPointer targetRef(target);
            removeTarget(targetRef);
            enableVatsEffectControlForWorldRollover();
        }

        releaseOwnedTarget(target);

        ROCK_LOG_DEBUG(Hand, "Selection highlight stopped for reference 0x{:08X}; remainingRockTargets={}",
            highlightedRef ? highlightedRef->formID : 0,
            remainingRockTargets);
    }
}
