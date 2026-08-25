#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace RE
{
    class NiAVObject;
}

namespace rock::weapon_omod_scene_scan
{
    struct NodeMatch
    {
        RE::NiAVObject* node{ nullptr };
        const char* rootLabel{ "" };
    };

    struct TokenSlot
    {
        std::string lowerToken;
        std::vector<std::string> lowerWords;
        std::vector<NodeMatch> matches;
    };

    [[nodiscard]] std::string makeModelToken(const char* modelPath);
    [[nodiscard]] std::vector<std::string> makeTokenWords(
        const std::string& lowerToken);
    [[nodiscard]] bool nameContainsToken(
        const char* name,
        const std::string& lowerToken);
    [[nodiscard]] bool nameIsConnectPoint(const char* name);

    /*
     * Scans one root with a per-root bound and a caller-owned shared visited
     * set. The shared set is the cross-root authority: nested candidate roots
     * do not rescan a subtree already reached from an earlier root.
     */
    void scanTree(
        RE::NiAVObject* root,
        std::size_t maxVisitedForRoot,
        std::uint32_t maxDepth,
        const char* rootLabel,
        std::unordered_set<std::uintptr_t>& visitedNodeAddresses,
        std::vector<TokenSlot>& tokenSlots,
        std::vector<NodeMatch>& connectPointMatches,
        std::size_t& outVisitedForRoot);
}
