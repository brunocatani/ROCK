#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::weapon_scene
{
    // Cached identities are never dereferenced. Resolve follows child slots
    // from the caller's live root, checking each identity before proceeding.
    // A detached/replaced/reordered path must be discovered again by its owner.
    template <class Node, std::size_t MaximumDepth = 64>
    class Path
    {
    public:
        void clear() { _root = 0; _count = 0; }

        bool capture(Node* root, Node* leaf)
        {
            clear();
            if (!root || !leaf) return false;
            std::array<Node*, MaximumDepth> reverse{};
            std::size_t count = 0;
            auto* cursor = leaf;
            while (cursor != root && cursor && count < reverse.size()) {
                reverse[count++] = cursor;
                cursor = cursor->parent;
            }
            if (cursor != root) return false;
            auto* parent = root;
            for (std::size_t i = 0; i < count; ++i) {
                auto* branch = parent->IsNode();
                if (!branch) return false;
                auto* child = reverse[count - i - 1];
                const auto& children = branch->children;
                std::size_t slot = 0;
                while (slot < children.capacity() && children[static_cast<decltype(children.capacity())>(slot)].get() != child) ++slot;
                if (slot == children.capacity()) return false;
                _steps[i] = { slot, reinterpret_cast<std::uintptr_t>(child), children.size(), children.capacity() };
                parent = child;
            }
            _root = reinterpret_cast<std::uintptr_t>(root);
            _count = count;
            return true;
        }

        template <class Visitor>
        Node* resolve(Node* root, Visitor&& visit) const
        {
            if (!root || reinterpret_cast<std::uintptr_t>(root) != _root) return nullptr;
            auto* current = root;
            visit(current);
            for (std::size_t i = 0; i < _count; ++i) {
                auto* branch = current->IsNode();
                if (!branch || _steps[i].slot >= branch->children.capacity() ||
                    _steps[i].childCount != branch->children.size() ||
                    _steps[i].slotCount != branch->children.capacity()) return nullptr;
                auto* child = branch->children[static_cast<decltype(branch->children.capacity())>(_steps[i].slot)].get();
                if (!child || reinterpret_cast<std::uintptr_t>(child) != _steps[i].identity) return nullptr;
                current = child;
                visit(current);
            }
            return current;
        }

        Node* resolve(Node* root) const { return resolve(root, [](Node*) {}); }

    private:
        struct Step
        {
            std::size_t slot{};
            std::uintptr_t identity{};
            std::size_t childCount{}, slotCount{};
        };
        std::array<Step, MaximumDepth> _steps{};
        std::uintptr_t _root = 0;
        std::size_t _count = 0;
    };
}
