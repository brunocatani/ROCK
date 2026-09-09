#include "physics-interaction/weapon/telemetry/WeaponTelemetryTraversal.h"
#include "physics-interaction/weapon/WeaponSceneChildren.h"

#include <array>
#include <cstddef>

namespace
{
    struct Node;
    struct Link
    {
        Node* value{};
        Node* get() const { return value; }
    };
    struct SparseChildren
    {
        std::array<Link, 520> slots{};
        std::size_t slotCount{ 8 };
        std::size_t size() const
        {
            std::size_t filled = 0;
            for (std::size_t i = 0; i < slotCount; ++i) filled += slots[i].value != nullptr;
            return filled;
        }
        std::size_t capacity() const { return slotCount; }
        const Link& operator[](std::size_t index) const { return slots[index]; }
    };
    struct Node
    {
        SparseChildren children;
        Node* IsNode() { return this; }
    };
}

int main()
{
    using rock::vanilla_weapon_alignment_telemetry::visitScene;
    Node skeleton, hand, weapon, muzzle;
    // Reproduce a weapon branch after holes in the skeleton's child slots.
    skeleton.children.slots[6].value = &hand;
    hand.children.slots[7].value = &weapon;
    weapon.children.slots[5].value = &muzzle;
    if (skeleton.children.size() != 1) return 1;
    bool foundMuzzle = false;
    const auto sparse = visitScene(&skeleton, [&](Node* value) {
        foundMuzzle = foundMuzzle || value == &muzzle;
        return true;
    });
    if (!foundMuzzle || sparse.visited != 4 || sparse.truncated) return 2;

    if (visitScene(static_cast<Node*>(nullptr), [](Node*) { return true; }).visited != 0) return 3;
    const auto stopped = visitScene(&skeleton, [](Node*) { return false; });
    if (stopped.visited != 1 || !stopped.truncated) return 4;

    weapon.children.slots[0].value = &skeleton;
    const auto cycle = visitScene(&skeleton, [](Node*) { return true; });
    if (cycle.visited != 512 || !cycle.truncated) return 5;

    Node oversized;
    oversized.children.slotCount = 520;
    oversized.children.slots[519].value = &muzzle;
    const auto bounded = visitScene(&oversized, [](Node*) { return true; });
    if (bounded.visited != 1 || !bounded.truncated) return 6;

    // Production collider capture and visual identity share this slot walk.
    // A receiver with two populated entries can hold P-Grip at slot seven,
    // with both stock shapes also beyond that branch's populated count.
    Node receiver, barrel, stock, housing, tube;
    receiver.children.slots[0].value = &barrel;
    receiver.children.slots[7].value = &stock;
    stock.children.slots[6].value = &housing;
    stock.children.slots[7].value = &tube;
    if (receiver.children.size() != 2 || stock.children.size() != 2) return 7;
    std::size_t captured = 0, slotSignature = 0;
    const auto capture = [&](Node* node, const auto& self) -> void {
        captured += node == &housing || node == &tube;
        rock::visitWeaponChildSlots(node->children, [&](Node* child, std::size_t slot) {
            slotSignature = slotSignature * 31 + slot + 1;
            self(child, self);
            return true;
        });
    };
    capture(&receiver, capture);
    if (captured != 2) return 8;
    const auto previousSignature = slotSignature;
    receiver.children.slots[7].value = nullptr;
    receiver.children.slots[5].value = &stock;
    captured = slotSignature = 0;
    capture(&receiver, capture);
    if (captured != 2 || slotSignature == previousSignature) return 9;

    std::size_t callbacks = 0;
    const bool completed = rock::visitWeaponChildSlots(receiver.children, [&](Node*, auto) {
        ++callbacks;
        return false;
    });
    if (completed || callbacks != 1) return 10;
    Node empty;
    if (!rock::visitWeaponChildSlots(empty.children, [&](Node*, auto) { ++callbacks; return true; }) || callbacks != 1) return 11;
    return 0;
}
