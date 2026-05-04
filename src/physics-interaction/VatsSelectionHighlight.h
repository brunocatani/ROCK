#pragma once

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
}

namespace frik::rock
{
    namespace detail
    {
        struct VatsEffectTarget;
    }

    class VatsSelectionHighlight
    {
    public:
        VatsSelectionHighlight() = default;
        ~VatsSelectionHighlight();

        VatsSelectionHighlight(const VatsSelectionHighlight&) = delete;
        VatsSelectionHighlight& operator=(const VatsSelectionHighlight&) = delete;

        VatsSelectionHighlight(VatsSelectionHighlight&&) = delete;
        VatsSelectionHighlight& operator=(VatsSelectionHighlight&&) = delete;

        bool play(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName);
        bool refresh(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode, bool enabled, const char* handName, const char* targetSourceName);
        bool refreshActive(bool enabled, const char* handName);
        void stop();

        [[nodiscard]] bool isActive() const noexcept { return _target != nullptr; }
        [[nodiscard]] bool isActiveForReference(RE::TESObjectREFR* refr) const noexcept { return _target != nullptr && _highlightedRef == refr; }
        [[nodiscard]] bool isActiveFor(RE::TESObjectREFR* refr, RE::NiAVObject* targetNode) const noexcept
        {
            return _target != nullptr && _highlightedRef == refr && _highlightedNode == targetNode;
        }

    private:
        detail::VatsEffectTarget* _target = nullptr;
        RE::TESObjectREFR* _highlightedRef = nullptr;
        RE::NiAVObject* _highlightedNode = nullptr;
    };
}
