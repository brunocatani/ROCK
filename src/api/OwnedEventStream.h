#pragma once
#include <ROCK/Abi.h>
#include <array>
#include <algorithm>
#include <mutex>

namespace rock::provider {
    // Each owner receives its own sequence. Events belonging to other owners
    // cannot inflate loss counts or consume this owner's retention capacity.
    template<class Event, std::size_t Capacity=256, std::size_t Owners=64>
    class OwnedEventStream {
        struct Slot {
            api::OwnerToken owner{};
            std::uint64_t next{1};
            std::uint32_t count{};
            std::array<Event,Capacity> records{};
        };
        std::array<Slot,Owners> _slots{};
        mutable std::mutex _mutex;
    public:
        api::Status bind(api::OwnerToken owner) {
            if (!owner) return api::Status::InvalidArgument;
            std::scoped_lock lock(_mutex);
            for (const auto& slot:_slots) if (slot.owner==owner) return api::Status::Ok;
            for (auto& slot:_slots) if (!slot.owner) { slot.owner=owner; slot.next=1; slot.count=0; return api::Status::Ok; }
            return api::Status::CapacityFull;
        }
        void remove(api::OwnerToken owner) {
            std::scoped_lock lock(_mutex);
            for (auto& slot:_slots) if (slot.owner==owner) { slot.owner=0; slot.count=0; return; }
        }
        template<class Capture> void publish(Event event, api::OwnerToken recipient, Capture&& capture) {
            std::scoped_lock lock(_mutex);
            for (auto& slot:_slots) {
                if (!slot.owner || (recipient && recipient!=slot.owner)) continue;
                event.sequence=slot.next++;
                slot.records[(event.sequence-1)%Capacity]=event;
                slot.count=std::min<std::uint32_t>(slot.count+1,Capacity);
                capture(slot.owner,event);
            }
        }
        void publish(Event event, api::OwnerToken recipient=0) {
            publish(event,recipient,[](api::OwnerToken,const Event&){});
        }
        api::Status copy(api::OwnerToken owner,std::uint64_t after,Event* output,std::uint32_t capacity,api::StreamV1& state) const {
            state={};
            if (capacity>Capacity) return api::Status::CapacityFull;
            if (capacity && !output) return api::Status::InvalidArgument;
            std::scoped_lock lock(_mutex);
            for (const auto& slot:_slots) {
                if (slot.owner!=owner) continue;
                const auto oldest=slot.next-slot.count;
                state.oldestSequence=slot.count?oldest:0;
                state.latestSequence=slot.next-1;
                state.nextSequence=after;
                if (after>=slot.next) return api::Status::InvalidArgument;
                const auto first=std::max(after+1,oldest);
                state.lostCount=oldest>after+1?oldest-after-1:0;
                const auto available=slot.next-first;
                state.copiedCount=static_cast<std::uint32_t>(std::min<std::uint64_t>(available,capacity));
                state.remainingCount=static_cast<std::uint32_t>(available-state.copiedCount);
                for (std::uint32_t i=0;i<state.copiedCount;++i) output[i]=slot.records[(first+i-1)%Capacity];
                if (state.copiedCount) state.nextSequence=first+state.copiedCount-1;
                return api::Status::Ok;
            }
            return api::Status::PermissionDenied;
        }
    };
}
