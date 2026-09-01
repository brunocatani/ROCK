#pragma once

#include <cstdint>

namespace RE
{
    class NiNode;
}

namespace rock
{
    class TwoHandedGrip;

    /*
     * Phase 1 of the weapon-transform arbiter: the single owner of the
     * equipped-weapon carrier claim. Every feature that wants the weapon
     * carried by the left firing hand, or restored to the native right
     * hand, routes through this class; nothing else may call
     * TwoHandedGrip::beginPersistentEquippedCarry,
     * restoreNativeRightEquippedCarry, or clearPersistentEquippedCarry.
     *
     * Carrier precedence, strongest first:
     *   shoulder sheath        weapon stashed; no carrier may claim
     *   pending primary-only   committed transfer awaiting manual grip start
     *   hand assignment        Pip-Boy / provider deliberate side choice
     *   manual handoff         live ambidextrous firing-grip ownership
     *   fixed hand             configured left-handed default, weakest
     *
     * Later phases move the per-frame weapon-node transform publication
     * (native / authored-align / two-hand solve / part-carry /
     * visual-return) behind the same single owner.
     */
    class WeaponTransformArbiter
    {
    public:
        enum class CarrySource : std::uint8_t
        {
            HandlingModeReconcile,
            HandAssignment,
            FixedHand,
        };

        struct FrameContext
        {
            bool shoulderSheathActive{ false };
            bool pendingPrimaryOnlyGripStart{ false };
            bool handAssignmentEngaged{ false };
            bool ambidextrousHandoffEnabled{ false };
        };

        explicit WeaponTransformArbiter(TwoHandedGrip& grip) noexcept :
            _grip(grip)
        {}

        WeaponTransformArbiter(const WeaponTransformArbiter&) = delete;
        WeaponTransformArbiter& operator=(const WeaponTransformArbiter&) = delete;

        // Published once per frame before the carrier drivers run.
        void beginFrame(const FrameContext& context) noexcept
        {
            _context = context;
        }

        // The configured fixed left hand is the weakest claimant: any
        // stash, committed transfer, deliberate assignment, or live
        // manual handoff outranks it.
        [[nodiscard]] bool fixedHandMayClaim() const;

        [[nodiscard]] bool requestLeftCarry(
            CarrySource source,
            RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            std::uint64_t weaponOwnershipKey);
        void restoreNativeRight(CarrySource source, const char* reason);
        void clearPersistentCarry(CarrySource source, const char* reason);

    private:
        TwoHandedGrip& _grip;
        FrameContext _context{};
    };
}
