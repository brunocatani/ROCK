#pragma once
#include "RE/Bethesda/BSFixedString.h"
#include "RE/Bethesda/MemoryManager.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::native_subgraph_metadata
{
    // FO4VR 1016340 accepts this heap list as argument nine. Supplying it
    // keeps off-screen graph requests out of the actor's live idle-root list.
    // 10198D0 copies retained strings and allocates blocks via 1B96110. This
    // is also the list used by native ExtraAnimGraphPreload at 0C9650/0C98D0.
    // Its temporary source uses a different scrap allocator; do not call
    // 101B590 (the scrap destructor) on this heap-owned destination.
    struct List
    {
        struct Record { RE::BSFixedString name; std::uint64_t flags; std::uint64_t identifier; };
        struct Block { std::array<std::byte,0x300> records; Block* next; };
        Block* head{};
        Block** tail{&head};
        Block* current{};
        Block* free{};
        Record* write{};
        Record* read{};
        std::uint32_t count{};
        std::uint32_t pad{};
        List()=default;
        List(const List&)=delete;
        List& operator=(const List&)=delete;
        ~List()
        {
            auto* block=head;
            auto* record=read;
            for (std::uint32_t i=0;i<count;++i) {
                record->name.~BSFixedString();
                ++record;
                if (block && reinterpret_cast<std::byte*>(record)>=block->records.data()+0x300) {
                    block=block->next;
                    record=block?reinterpret_cast<Record*>(block->records.data()):nullptr;
                }
            }
            while (head) { auto* next=head->next; RE::free(head); head=next; }
            while (free) { auto* next=free->next; RE::free(free); free=next; }
        }
    };
    static_assert(sizeof(List)==0x38);
    static_assert(sizeof(List::Record)==0x18);
    static_assert(sizeof(List::Block)==0x308);
}
