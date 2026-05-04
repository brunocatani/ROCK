#pragma once

/*
 * Held grabs must not have a hidden character-controller authority fighting the
 * grab motor. HIGGS removes held/ignored bodies from the player-proxy collector
 * before Havok builds constraints. In FO4VR, the verified character-proxy
 * callback exposes paired manifold/contact rows before ROCK calls the original
 * listener, so ROCK can apply the same ownership rule without a broad world-cast
 * hook: compact held-body rows out before the original callback sees them. The
 * rejected alternative was a post-callback surface-velocity scrub, because it
 * leaves the engine-generated held-body contact alive and adds a late writer
 * against the same body the grab constraint is already controlling.
 */

#include <cstdint>
#include <cstring>

namespace frik::rock::held_grab_cc_policy
{
    inline constexpr int kGeneratedContactStride = 0x40;
    inline constexpr int kGeneratedContactBodyIdOffset = 0x28;
    inline constexpr int kGeneratedConstraintRowsOffset = 0x48;
    inline constexpr int kGeneratedConstraintCountOffset = 0x50;

    enum class HeldGrabContactIntervention : std::uint8_t
    {
        None = 0,
        PreOriginalFilter = 1,
    };

    struct HeldGrabContactPolicyInput
    {
        bool hooksEnabled = false;
        bool holdingHeldObject = false;
        bool diagnosticsEnabled = false;
    };

    struct HeldGrabContactPolicyDecision
    {
        HeldGrabContactIntervention intervention = HeldGrabContactIntervention::None;
        bool mayFilterBeforeOriginal = false;
        bool mayInspectGeneratedConstraint = false;
        bool mayMutateGeneratedConstraint = false;
        const char* reason = "postConstraintScrubDisabled";
    };

    struct GeneratedContactBufferView
    {
        bool valid = false;
        char* manifoldEntries = nullptr;
        char* constraintEntries = nullptr;
        int* manifoldCountPtr = nullptr;
        int* constraintCountPtr = nullptr;
        int manifoldCount = 0;
        int constraintCount = 0;
        int pairCount = 0;
        const char* reason = "uninitialized";
    };

    struct GeneratedContactFilterResult
    {
        bool valid = false;
        int originalPairCount = 0;
        int keptPairCount = 0;
        int removedPairCount = 0;
        const char* reason = "uninitialized";
    };

    inline HeldGrabContactPolicyDecision evaluateHeldGrabContactPolicy(const HeldGrabContactPolicyInput& input)
    {
        if (!input.hooksEnabled) {
            return HeldGrabContactPolicyDecision{
                .intervention = HeldGrabContactIntervention::None,
                .mayFilterBeforeOriginal = false,
                .mayInspectGeneratedConstraint = false,
                .mayMutateGeneratedConstraint = false,
                .reason = "hooksDisabled",
            };
        }

        if (!input.holdingHeldObject) {
            return HeldGrabContactPolicyDecision{
                .intervention = HeldGrabContactIntervention::None,
                .mayFilterBeforeOriginal = false,
                .mayInspectGeneratedConstraint = false,
                .mayMutateGeneratedConstraint = false,
                .reason = "notHolding",
            };
        }

        return HeldGrabContactPolicyDecision{
            .intervention = HeldGrabContactIntervention::PreOriginalFilter,
            .mayFilterBeforeOriginal = true,
            .mayInspectGeneratedConstraint = input.diagnosticsEnabled,
            .mayMutateGeneratedConstraint = false,
            .reason = "preOriginalHeldBodyFilter",
        };
    }

    inline GeneratedContactBufferView makeGeneratedContactBufferView(void* manifold, void* simplexInput)
    {
        if (!manifold) {
            return GeneratedContactBufferView{ .valid = false, .reason = "missingManifold" };
        }
        if (!simplexInput) {
            return GeneratedContactBufferView{ .valid = false, .reason = "missingSimplexInput" };
        }

        auto* manifoldBytes = static_cast<char*>(manifold);
        auto* simplexBytes = static_cast<char*>(simplexInput);
        char* manifoldEntries = *reinterpret_cast<char**>(manifoldBytes);
        int* manifoldCountPtr = reinterpret_cast<int*>(manifoldBytes + sizeof(char*));
        char* constraintEntries = *reinterpret_cast<char**>(simplexBytes + kGeneratedConstraintRowsOffset);
        int* constraintCountPtr = reinterpret_cast<int*>(simplexBytes + kGeneratedConstraintCountOffset);
        const int manifoldCount = *manifoldCountPtr;
        const int constraintCount = *constraintCountPtr;

        if (!manifoldEntries) {
            return GeneratedContactBufferView{
                .valid = false,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "missingManifoldEntries",
            };
        }
        if (!constraintEntries) {
            return GeneratedContactBufferView{
                .valid = false,
                .manifoldEntries = manifoldEntries,
                .manifoldCountPtr = manifoldCountPtr,
                .constraintCountPtr = constraintCountPtr,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "missingConstraintEntries",
            };
        }
        if (manifoldCount <= 0 || constraintCount <= 0) {
            return GeneratedContactBufferView{
                .valid = false,
                .manifoldEntries = manifoldEntries,
                .constraintEntries = constraintEntries,
                .manifoldCountPtr = manifoldCountPtr,
                .constraintCountPtr = constraintCountPtr,
                .manifoldCount = manifoldCount,
                .constraintCount = constraintCount,
                .reason = "emptyContactBuffers",
            };
        }

        return GeneratedContactBufferView{
            .valid = true,
            .manifoldEntries = manifoldEntries,
            .constraintEntries = constraintEntries,
            .manifoldCountPtr = manifoldCountPtr,
            .constraintCountPtr = constraintCountPtr,
            .manifoldCount = manifoldCount,
            .constraintCount = constraintCount,
            .pairCount = manifoldCount < constraintCount ? manifoldCount : constraintCount,
            .reason = "ok",
        };
    }

    template <class IsHeldBody>
    inline GeneratedContactFilterResult filterGeneratedContactBuffers(const GeneratedContactBufferView& view, IsHeldBody&& isHeldBody)
    {
        if (!view.valid) {
            return GeneratedContactFilterResult{
                .valid = false,
                .reason = view.reason,
            };
        }

        if (!view.manifoldEntries || !view.constraintEntries || !view.manifoldCountPtr || !view.constraintCountPtr) {
            return GeneratedContactFilterResult{
                .valid = false,
                .originalPairCount = view.pairCount,
                .reason = "missingFilterBuffer",
            };
        }

        int writeIndex = 0;
        for (int readIndex = 0; readIndex < view.pairCount; ++readIndex) {
            char* manifoldEntry = view.manifoldEntries + readIndex * kGeneratedContactStride;
            const auto bodyId = *reinterpret_cast<std::uint32_t*>(manifoldEntry + kGeneratedContactBodyIdOffset);
            if (isHeldBody(bodyId)) {
                continue;
            }

            if (writeIndex != readIndex) {
                std::memmove(view.manifoldEntries + writeIndex * kGeneratedContactStride,
                    manifoldEntry,
                    kGeneratedContactStride);
                std::memmove(view.constraintEntries + writeIndex * kGeneratedContactStride,
                    view.constraintEntries + readIndex * kGeneratedContactStride,
                    kGeneratedContactStride);
            }
            ++writeIndex;
        }

        const int removedCount = view.pairCount - writeIndex;
        if (removedCount > 0) {
            *view.manifoldCountPtr = writeIndex;
            *view.constraintCountPtr = writeIndex;
        }

        return GeneratedContactFilterResult{
            .valid = true,
            .originalPairCount = view.pairCount,
            .keptPairCount = writeIndex,
            .removedPairCount = removedCount,
            .reason = removedCount > 0 ? "filteredHeldBodyContacts" : "noHeldBodyContacts",
        };
    }
}
