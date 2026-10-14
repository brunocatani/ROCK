#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <array>

namespace rock
{
    /*
     * Equip/drop seamlessness diagnostic. For a bounded window after each
     * weapon transition it records, once per rendered frame, which model is
     * visible in each hand (equip bridge, incoming second-weapon bridge,
     * index-0 or index-1 native model), how many candidates are visible there,
     * and how far that visible weapon and the rendered hand moved since the
     * previous frame. A gap is a frame with no visible weapon; a duplicate is
     * a count above one; a pop is a large move, flagged when the visible model
     * changed. The baseline is the last rendered frame before the transition.
     */
    namespace
    {
        enum class SeamSource : std::uint8_t { None, EquipBridge, IncomingBridge, Slot0, Slot1 };

        const char* seamSourceName(SeamSource source)
        {
            switch (source) {
            case SeamSource::EquipBridge: return "equip-bridge";
            case SeamSource::IncomingBridge: return "incoming-bridge";
            case SeamSource::Slot0: return "slot0";
            case SeamSource::Slot1: return "slot1";
            case SeamSource::None: default: return "none";
            }
        }

        struct SeamHand
        {
            const RE::NiAVObject* weapon = nullptr;
            RE::NiTransform weaponWorld{}, handWorld{};
            SeamSource source = SeamSource::None;
            std::uint32_t visibleCount = 0;
            bool handValid = false;
        };

        struct SeamTrace
        {
            const char* event = nullptr;
            std::array<SeamHand, 2> hands{}; // right, left
            std::uint32_t framesLeft = 0, frame = 0;
            float elapsedMs = 0.0f;
        };
        SeamTrace s_seam{};
        constexpr std::uint32_t kSeamFrames = 40;

        bool renderedPath(const RE::NiAVObject* node)
        {
            for (std::uint32_t depth = 0; node && depth < 48; ++depth, node = node->parent) {
                if (node->GetAppCulled()) return false;
            }
            return node == nullptr;
        }

    std::array<SeamHand, 2> observeSeamHands(const EquipVisualBridge& bridge, bool slot0Left)
    {
        std::array<SeamHand, 2> hands{};
        const auto views = native_hand_equipment::equippedViews();
        bool incomingLeft = false;
        const auto* incoming = native_secondary_presentation::incomingModelForTrace(incomingLeft);
        const bool slot1Left = views[1].weapon && native_hand_equipment::secondaryHandIsLeft();
        struct Candidate { const RE::NiAVObject* node; bool left; SeamSource source; };
        const std::array<Candidate, 4> candidates{ {
            { bridge.isModelPresented() ? bridge.traceModel() : nullptr, bridge.isLeftHandForTrace(), SeamSource::EquipBridge },
            { incoming, incomingLeft, SeamSource::IncomingBridge },
            { views[0].root, slot0Left, SeamSource::Slot0 },
            { views[1].root, slot1Left, SeamSource::Slot1 },
        } };
        for (std::size_t h = 0; h < hands.size(); ++h) {
            const bool left = h == 1;
            auto& hand = hands[h];
            hand.handValid = frik_visual_authority::tryGetPresentedHandWorldTransform(left, hand.handWorld);
            for (const auto& candidate : candidates) {
                if (!candidate.node || candidate.left != left || !renderedPath(candidate.node)) continue;
                if (++hand.visibleCount == 1) {
                    hand.weapon = candidate.node;
                    hand.weaponWorld = candidate.node->world;
                    hand.source = candidate.source;
                }
            }
        }
        return hands;
    }
    }

    void PhysicsInteraction::armTransitionSeamTrace(const char* event) noexcept
    {
        try {
            // A transition inside an open window keeps its baseline.
            if (!s_seam.framesLeft) {
                s_seam.hands = observeSeamHands(_equipped.transition.bridgeForTrace(), _twoHandedGrip.isFiringHandLeft());
                s_seam.frame = 0;
                s_seam.elapsedMs = 0.0f;
            }
            s_seam.event = event;
            s_seam.framesLeft = kSeamFrames;
            ROCK_LOG_INFO(Weapon, "SEAM begin event={} right={} left={}", event,
                seamSourceName(s_seam.hands[0].source), seamSourceName(s_seam.hands[1].source));
        } catch (...) {
            s_seam = {};
        }
    }

    void PhysicsInteraction::sampleTransitionSeam() noexcept
    {
        if (!s_seam.framesLeft) return;
        try {
            --s_seam.framesLeft;
            ++s_seam.frame;
            s_seam.elapsedMs += runtime_state::currentFrame().deltaSeconds * 1000.0f;
            const auto hands = observeSeamHands(_equipped.transition.bridgeForTrace(), _twoHandedGrip.isFiringHandLeft());
            std::array<float, 2> weaponMove{ -1.0f, -1.0f }, weaponTurn{ -1.0f, -1.0f }, handMove{ -1.0f, -1.0f }, handTurn{ -1.0f, -1.0f };
            std::array<bool, 2> switched{};
            for (std::size_t h = 0; h < hands.size(); ++h) {
                const auto& previous = s_seam.hands[h];
                const auto& now = hands[h];
                if (previous.weapon && now.weapon) {
                    weaponMove[h] = dynamic_weapon_collision_policy::translationDeltaGameUnits(previous.weaponWorld, now.weaponWorld);
                    weaponTurn[h] = dynamic_weapon_collision_policy::rotationDeltaDegrees(previous.weaponWorld, now.weaponWorld);
                }
                switched[h] = previous.source != now.source || previous.weapon != now.weapon;
                if (previous.handValid && now.handValid) {
                    handMove[h] = dynamic_weapon_collision_policy::translationDeltaGameUnits(previous.handWorld, now.handWorld);
                    handTurn[h] = dynamic_weapon_collision_policy::rotationDeltaDegrees(previous.handWorld, now.handWorld);
                }
            }
            ROCK_LOG_INFO(Weapon,
                "SEAM frame={} event={} ms={:.1f} R[{} n={} switch={} weapon={:.2f}gu/{:.2f}deg hand={:.2f}gu/{:.2f}deg] "
                "L[{} n={} switch={} weapon={:.2f}gu/{:.2f}deg hand={:.2f}gu/{:.2f}deg] nativeState={}",
                s_seam.frame, s_seam.event ? s_seam.event : "?", s_seam.elapsedMs,
                seamSourceName(hands[0].source), hands[0].visibleCount, switched[0], weaponMove[0], weaponTurn[0], handMove[0], handTurn[0],
                seamSourceName(hands[1].source), hands[1].visibleCount, switched[1], weaponMove[1], weaponTurn[1], handMove[1], handTurn[1],
                f4vr::getNativeWeaponState(f4vr::getPlayer()));
            s_seam.hands = hands;
        } catch (...) {
            s_seam = {};
        }
    }
}
