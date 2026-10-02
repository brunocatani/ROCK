#pragma once
#include "AuthoredInteractionLease.h"
#include <RE/NetImmerse/NiPoint.h>
#include <span>
namespace rock::authored_interaction {
    // Frame-owner state only. Native collision snapshots are published separately
    // through the existing simulation filter's quiescence boundary.
    api::Status set(api::OwnerToken,const Request&);
    api::Status clear(api::OwnerToken,api::Hand);
    bool reserves(bool isLeft);
    bool isAttached(bool isLeft);
    std::uint32_t appendIndicators(std::span<RE::NiPoint3>);
    void reset() noexcept;
}
