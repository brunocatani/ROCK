#pragma once
namespace RE { class NiAVObject; }
namespace rock::scene_relation {
    bool nodeContainsNode(RE::NiAVObject* root,RE::NiAVObject* target,int maxDepth=32);
}
