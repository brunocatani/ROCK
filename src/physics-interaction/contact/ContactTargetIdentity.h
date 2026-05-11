#pragma once

/*
 * Contact target identity is intentionally separated from soft-contact math.
 * ROCK treats contact callbacks as body-pair evidence first, then lets higher
 * systems decide what those contacts mean. Native hknp callbacks and swept
 * queries provide body IDs, points, and normals; this layer resolves those IDs
 * into optional FO4 refs and stable diagnostics without changing collision
 * state or visual hand authority.
 */

#include "physics-interaction/contact/NativeContactEvidence.h"

#include "RE/NetImmerse/NiPoint.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace RE
{
    class bhkWorld;
    class hknpWorld;
}

namespace rock::contact_target_identity
{
    inline constexpr std::uint32_t kUnknownLayer = 0xFFFF'FFFFu;
    inline constexpr std::uint32_t kUnknownFilterInfo = 0xFFFF'FFFFu;
    inline constexpr std::uint32_t kInvalidFormId = 0;
    inline constexpr std::uint32_t kInvalidMotionIndex = 0xFFFF'FFFFu;

    enum class ContactTargetResolutionStatus : std::uint8_t
    {
        InvalidInput = 0,
        BodyUnreadable,
        BodyResolved,
        MissingBhkWorld,
        UnresolvedReference,
        ResolvedReference,
    };

    enum class SurfaceHint : std::uint8_t
    {
        Unknown = 0,
        WallLike,
        FloorLike,
        CeilingLike,
        SlopeLike,
    };

    struct ContactTargetIdentity
    {
        bool valid = false;
        bool hasResolvedReference = false;
        bool hasRichText = false;
        std::uint32_t bodyId = contact_evidence::kInvalidBodyId;
        std::uint32_t layer = kUnknownLayer;
        std::uint32_t filterInfo = kUnknownFilterInfo;
        std::uint32_t motionIndex = kInvalidMotionIndex;
        contact_evidence::NativeContactEndpointKind endpointKind = contact_evidence::NativeContactEndpointKind::Unknown;
        ContactTargetResolutionStatus status = ContactTargetResolutionStatus::InvalidInput;
        SurfaceHint surfaceHint = SurfaceHint::Unknown;
        bool hasContactPoint = false;
        bool hasContactNormal = false;
        RE::NiPoint3 contactPointGame{};
        RE::NiPoint3 contactNormalGame{};
        std::uint32_t refFormId = kInvalidFormId;
        std::uint32_t baseFormId = kInvalidFormId;
        std::string formType;
        std::string displayName;
        std::string refEditorId;
        std::string baseEditorId;
    };

    struct ContactTargetResolutionOptions
    {
        bool resolveReference = false;
        bool includeRichText = false;
    };

    inline bool isFinitePoint(const RE::NiPoint3& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    inline SurfaceHint classifySurfaceHint(const RE::NiPoint3& normal)
    {
        if (!isFinitePoint(normal)) {
            return SurfaceHint::Unknown;
        }

        const float lengthSquared = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
        if (!std::isfinite(lengthSquared) || lengthSquared <= 1.0e-6f) {
            return SurfaceHint::Unknown;
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        const float z = normal.z * inverseLength;
        if (z >= 0.75f) {
            return SurfaceHint::FloorLike;
        }
        if (z <= -0.75f) {
            return SurfaceHint::CeilingLike;
        }
        if (std::fabs(z) <= 0.35f) {
            return SurfaceHint::WallLike;
        }
        return SurfaceHint::SlopeLike;
    }

    inline const char* surfaceHintName(SurfaceHint hint)
    {
        switch (hint) {
        case SurfaceHint::WallLike:
            return "WallLike";
        case SurfaceHint::FloorLike:
            return "FloorLike";
        case SurfaceHint::CeilingLike:
            return "CeilingLike";
        case SurfaceHint::SlopeLike:
            return "SlopeLike";
        default:
            return "Unknown";
        }
    }

    inline const char* resolutionStatusName(ContactTargetResolutionStatus status)
    {
        switch (status) {
        case ContactTargetResolutionStatus::InvalidInput:
            return "InvalidInput";
        case ContactTargetResolutionStatus::BodyUnreadable:
            return "BodyUnreadable";
        case ContactTargetResolutionStatus::BodyResolved:
            return "BodyResolved";
        case ContactTargetResolutionStatus::MissingBhkWorld:
            return "MissingBhkWorld";
        case ContactTargetResolutionStatus::UnresolvedReference:
            return "UnresolvedReference";
        case ContactTargetResolutionStatus::ResolvedReference:
            return "ResolvedReference";
        default:
            return "Unknown";
        }
    }

    inline const char* endpointKindName(contact_evidence::NativeContactEndpointKind kind)
    {
        using contact_evidence::NativeContactEndpointKind;
        switch (kind) {
        case NativeContactEndpointKind::RightHand:
            return "RightHand";
        case NativeContactEndpointKind::LeftHand:
            return "LeftHand";
        case NativeContactEndpointKind::Weapon:
            return "Weapon";
        case NativeContactEndpointKind::RightHeldObject:
            return "RightHeldObject";
        case NativeContactEndpointKind::LeftHeldObject:
            return "LeftHeldObject";
        case NativeContactEndpointKind::External:
            return "External";
        case NativeContactEndpointKind::WorldSurface:
            return "WorldSurface";
        case NativeContactEndpointKind::DynamicProp:
            return "DynamicProp";
        case NativeContactEndpointKind::Actor:
            return "Actor";
        case NativeContactEndpointKind::QueryOnly:
            return "QueryOnly";
        default:
            return "Unknown";
        }
    }

    ContactTargetIdentity resolveContactTarget(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        std::uint32_t bodyId,
        contact_evidence::NativeContactEndpointKind endpointKind,
        const RE::NiPoint3* contactPointGame,
        const RE::NiPoint3* contactNormalGame,
        ContactTargetResolutionOptions options = {});
}
