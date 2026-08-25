#include "physics-interaction/weapon/WeaponOmodSceneScan.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include <algorithm>

namespace rock::weapon_omod_scene_scan
{
    namespace
    {
        constexpr std::size_t kMaximumMatchesPerToken = 8;
        constexpr std::size_t kMaximumConnectPointMatches = 64;

        [[nodiscard]] char toLowerAscii(const char value)
        {
            return value >= 'A' && value <= 'Z' ?
                static_cast<char>(value + ('a' - 'A')) :
                value;
        }

        [[nodiscard]] bool matchesContainNode(
            const std::vector<NodeMatch>& matches,
            const RE::NiAVObject* node)
        {
            return std::ranges::any_of(
                matches,
                [node](const NodeMatch& match) { return match.node == node; });
        }

        [[nodiscard]] bool nameMatchesTokenSlot(
            const char* name,
            const TokenSlot& slot)
        {
            if (nameContainsToken(name, slot.lowerToken)) {
                return true;
            }
            if (slot.lowerWords.empty()) {
                return false;
            }
            return std::ranges::all_of(
                slot.lowerWords,
                [name](const std::string& word) {
                    return nameContainsToken(name, word);
                });
        }

        void scanTreeRecursive(
            RE::NiAVObject* node,
            const std::uint32_t depth,
            const std::size_t maxVisitedForRoot,
            const std::uint32_t maxDepth,
            const char* rootLabel,
            std::unordered_set<std::uintptr_t>& visitedNodeAddresses,
            std::vector<TokenSlot>& tokenSlots,
            std::vector<NodeMatch>& connectPointMatches,
            std::size_t& visitedForRoot)
        {
            if (!node || visitedForRoot >= maxVisitedForRoot || depth > maxDepth) {
                return;
            }

            const auto address = reinterpret_cast<std::uintptr_t>(node);
            if (!visitedNodeAddresses.insert(address).second) {
                return;
            }
            ++visitedForRoot;

            const char* name = node->name.c_str();
            if (name && name[0] != '\0') {
                if (nameIsConnectPoint(name) &&
                    connectPointMatches.size() < kMaximumConnectPointMatches &&
                    !matchesContainNode(connectPointMatches, node)) {
                    connectPointMatches.push_back(NodeMatch{ node, rootLabel });
                }
                for (auto& slot : tokenSlots) {
                    if (slot.lowerToken.empty() ||
                        slot.matches.size() >= kMaximumMatchesPerToken ||
                        !nameMatchesTokenSlot(name, slot) ||
                        matchesContainNode(slot.matches, node)) {
                        continue;
                    }
                    slot.matches.push_back(NodeMatch{ node, rootLabel });
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }
            const auto& children = niNode->children;
            for (auto index = decltype(children.size()){ 0 };
                 index < children.size() && visitedForRoot < maxVisitedForRoot;
                 ++index) {
                scanTreeRecursive(
                    children[index].get(),
                    depth + 1,
                    maxVisitedForRoot,
                    maxDepth,
                    rootLabel,
                    visitedNodeAddresses,
                    tokenSlots,
                    connectPointMatches,
                    visitedForRoot);
            }
        }
    }

    std::string makeModelToken(const char* modelPath)
    {
        if (!modelPath || modelPath[0] == '\0') {
            return {};
        }
        const char* base = modelPath;
        for (const char* cursor = modelPath; *cursor; ++cursor) {
            if (*cursor == '\\' || *cursor == '/') {
                base = cursor + 1;
            }
        }
        std::string token(base);
        const auto dot = token.find_last_of('.');
        if (dot != std::string::npos) {
            token.resize(dot);
        }
        for (auto& character : token) {
            character = toLowerAscii(character);
        }
        return token;
    }

    std::vector<std::string> makeTokenWords(const std::string& lowerToken)
    {
        std::vector<std::string> words;
        std::string current;
        for (const char character : lowerToken) {
            if (character == '_' || character == '-' || character == ' ') {
                if (current.size() >= 2) {
                    words.push_back(current);
                }
                current.clear();
            } else {
                current += character;
            }
        }
        if (current.size() >= 2) {
            words.push_back(current);
        }
        if (words.size() < 2) {
            words.clear();
        }
        return words;
    }

    bool nameContainsToken(const char* name, const std::string& lowerToken)
    {
        if (!name || lowerToken.empty()) {
            return false;
        }
        const std::size_t tokenLength = lowerToken.size();
        for (const char* cursor = name; *cursor; ++cursor) {
            std::size_t index = 0;
            while (index < tokenLength) {
                const char character = cursor[index];
                if (character == '\0' ||
                    toLowerAscii(character) != lowerToken[index]) {
                    break;
                }
                ++index;
            }
            if (index == tokenLength) {
                return true;
            }
        }
        return false;
    }

    bool nameIsConnectPoint(const char* name)
    {
        return name &&
               (name[0] == 'P' || name[0] == 'p') &&
               name[1] == '-';
    }

    void scanTree(
        RE::NiAVObject* root,
        const std::size_t maxVisitedForRoot,
        const std::uint32_t maxDepth,
        const char* rootLabel,
        std::unordered_set<std::uintptr_t>& visitedNodeAddresses,
        std::vector<TokenSlot>& tokenSlots,
        std::vector<NodeMatch>& connectPointMatches,
        std::size_t& outVisitedForRoot)
    {
        outVisitedForRoot = 0;
        scanTreeRecursive(
            root,
            0,
            maxVisitedForRoot,
            maxDepth,
            rootLabel,
            visitedNodeAddresses,
            tokenSlots,
            connectPointMatches,
            outVisitedForRoot);
    }
}
