#pragma once

#include "RE/Bethesda/BSAnimationGraph.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiNode.h"
#include <array>
#include <cstdint>
#include <memory>
#include <span>

namespace RE { class Actor; class TESRace; }

namespace rock::native_equipped_action_graph
{
    // A native behavior instance for one persistently equipped inventory item.
    // Owns no actor, inventory entry, firing clock, or synthesized clip timeline.
    class Graph final
    {
    public:
        enum class Load { Pending, Ready, Failed };
        struct Event { RE::BSFixedString tag, payload; };
        using Events = std::array<Event, 128>;

        Graph();
        ~Graph();
        Graph(const Graph&) = delete;
        Graph& operator=(const Graph&) = delete;

        static bool validate() noexcept;
        bool start(RE::Actor* player, const RE::BGSObjectInstance& item);
        Load advance();
        // A native loading task must finish before the holder destructor runs.
        bool canDestroy() const noexcept;
        bool notify(const RE::BSFixedString& event);
        void setInt(const RE::BSFixedString& name, std::int32_t value);
        void setFloat(const RE::BSFixedString& name, float value);
        void update(float deltaSeconds);
        std::size_t takeEvents(Events& output);
        bool eventOverflow() const noexcept;
        const char* stage() const noexcept;

        RE::SimpleAnimationGraphManagerHolder* holder() const noexcept;
        RE::BSAnimationGraphManager* manager() const noexcept;
        RE::NiNode* bone(const RE::BSFixedString& name) const noexcept;
        // Anchor the entire evaluated skeleton to the held weapon. Animation
        // keeps the local part motion; tracking owns only its world frame.
        bool place(const RE::NiTransform& weaponWorld) noexcept;
        // Idle-root records from this graph's base and weapon subgraph
        // requests, in the order native 10197E0 adds them to an actor's
        // process list (de-duplicated by identifier). The native collector
        // EC5C80 walks that list newest first and keeps one person mode.
        struct IdleRoot
        {
            RE::BSFixedString name;
            std::uint8_t person{};
            std::uint64_t identifier{};
        };
        std::span<const IdleRoot> idleRoots() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
}
