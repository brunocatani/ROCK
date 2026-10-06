#include "api/AttachedSceneLookup.h"
#include <cassert>
#include <string_view>
#include <vector>

struct Node;
struct Pointer { Node* value{}; Node* get() const { return value; } };
struct Node { std::string_view name; Node* parent{}; bool mesh{}; std::vector<Pointer> children; };
int main()
{
    using namespace rock::attached_scene_lookup;
    Node skeleton{"skeleton"},hip{"thigh",&skeleton},proxy{"stored",&hip},visual{"stored",&hip},mesh{"mesh",&visual,true};
    skeleton.children={{&hip}}; hip.children={{&visual},{&proxy}}; visual.children={{&mesh}};
    const auto children=[](Node* node) { return &node->children; };
    const auto matches=[&](Node* node) { return node->name=="stored" && find(node,[](Node* n) { return n->mesh; },children); };
    assert(find(&skeleton,matches,children)==&visual); // Empty bone proxy is visited first, but must never win.
    assert(beneath(&skeleton,&visual) && !beneath(&visual,&visual));
    visual.parent=nullptr;
    assert(!beneath(&skeleton,&visual)); // A retained cached pointer is not sufficient after detach.
    hip.children={{&proxy}};
    assert(find(&skeleton,matches,children)==nullptr);
    Node cycle{"cycle"}; cycle.children={{&cycle}}; cycle.parent=&cycle;
    assert(find(&cycle,[](Node*) { return false; },children)==nullptr && !beneath(&skeleton,&cycle));
}
