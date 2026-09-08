#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

namespace rock::vanilla_weapon_alignment_telemetry
{
    struct TraversalResult
    {
        std::size_t visited{ 0 };
        bool truncated{ false };
    };

    // NiTArray is sparse: size() counts populated slots, while indexing uses
    // slot positions through capacity(). A live Weapon may sit beyond size().
    // Borrow nodes only for this traversal; stop on output, node, or slot bounds.
    template <class Node, class Visitor>
    TraversalResult visitScene(Node* root, Visitor&& visit)
    {
        std::array<Node*, 512> pending{};
        std::size_t count = 0;
        TraversalResult result{};
        if (root) {
            pending[count++] = root;
        }
        while (count && result.visited < pending.size()) {
            auto* current = pending[--count];
            ++result.visited;
            if (!visit(current)) {
                result.truncated = true;
                break;
            }
            if (auto* branch = current->IsNode()) {
                const auto& children = branch->children;
                const auto limit = (std::min)(static_cast<std::size_t>(children.capacity()), pending.size());
                result.truncated = result.truncated || children.capacity() > limit;
                for (std::size_t i = 0; i < limit; ++i) {
                    if (auto* child = children[static_cast<decltype(children.capacity())>(i)].get()) {
                        if (count == pending.size()) {
                            result.truncated = true;
                            break;
                        }
                        pending[count++] = child;
                    }
                }
            }
        }
        result.truncated = result.truncated || count != 0;
        return result;
    }
}
