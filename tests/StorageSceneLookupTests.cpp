#include "api/StorageSceneLookup.h"
#include <array>
#include <cassert>
#include <string_view>

namespace
{
    struct Node;
    using Name = std::string_view;
    using Lookup = Node* (*)(Node*, const Name&);
    struct Node { Lookup* table; Node* child{}; };
    unsigned flatCalls{}, vrCalls{};
    Node poison{}, child{};
    const Name* forwardedName{};
    Node* flatSlot(Node*, const Name&) { ++flatCalls; return &poison; }
    Node* vrSlot(Node* root, const Name& name)
    {
        ++vrCalls; forwardedName = &name;
        return name == "mounted-weapon" ? root->child : nullptr;
    }
}

int main()
{
    // Reproduce a non-null garbage result from the flat slot; the production
    // helper must dispatch the independently verified VR slot instead.
    std::array<Lookup, 0x32> table{};
    table[0x2E] = flatSlot;
    table[0x31] = vrSlot;
    Node root{table.data(), &child};
    const Name name{"mounted-weapon"};
    assert(rock::storage_scene_lookup::find(&root,name) == &child);
    assert(flatCalls == 0 && vrCalls == 1 && forwardedName == &name);
    assert(rock::storage_scene_lookup::find(&root,Name{"absent"}) == nullptr);
    assert(flatCalls == 0 && vrCalls == 2);
    assert(rock::storage_scene_lookup::find(static_cast<Node*>(nullptr),name) == nullptr);
    Node noTable{};
    assert(rock::storage_scene_lookup::find(&noTable,name) == nullptr);
    table[0x31] = nullptr;
    assert(rock::storage_scene_lookup::find(&root,name) == nullptr);
    assert(flatCalls == 0 && vrCalls == 2);
}
