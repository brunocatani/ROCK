#pragma once
#include <cstdint>
#include <cstddef>
namespace rock::storage_interaction_policy
{
    constexpr std::uint32_t invalidBody=0x7FFFFFFF;
    constexpr bool fresh(std::uint64_t now,std::uint64_t contact)
    { return contact && now>=contact && now-contact<=2; }
    constexpr bool allowsHand(std::uint32_t freeMask,bool isLeft)
    { return (freeMask & (isLeft?2u:1u))!=0; }
    constexpr bool validLease(std::uint32_t frames) { return frames>=1 && frames<=3; }
    constexpr bool validRouting(std::uint32_t mask,std::uint32_t frames)
    { return mask<=3 && validLease(frames); }
    constexpr bool exactHand(std::uint32_t peer,std::uintptr_t peerObject,std::uint32_t hand,std::uintptr_t handObject)
    { return hand!=invalidBody && peer==hand && handObject && peerObject==handObject; }
    constexpr std::size_t preparationIndex(std::size_t ordinal,std::uint64_t cursor,std::size_t count)
    { return (ordinal+cursor%count)%count; }
}
