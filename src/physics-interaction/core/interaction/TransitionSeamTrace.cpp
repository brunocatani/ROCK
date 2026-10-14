#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <array>
#include <cmath>

namespace rock
{
    /*
     * Equip/drop seamlessness diagnostic. For a bounded window after each
     * weapon transition it records, once per rendered frame, which model is
     * visible in each hand (equip bridge, incoming second-weapon bridge,
     * index-0 or index-1 native model) and how many are visible there.
     *
     * Model roots of different sources are not registered to each other, so
     * the visible gun is measured by its geometry: the mean of its shapes'
     * world bound centres (C) and the shape centre farthest from it (F, along
     * the gun). Per frame it logs how tip C and F moved in the world and in
     * the hand's frame, and where they sit in the hand against the last frame
     * before the transition. A gap is a frame with no visible weapon; a
     * duplicate is a count above one; a visible pop is a geometry step.
     * Every rendered frame is observed so the baseline is the frame before.
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
            RE::NiTransform handWorld{};
            RE::NiPoint3 center{}, tip{};
            SeamSource source = SeamSource::None;
            std::uint32_t visibleCount = 0;
            bool handValid = false, geometry = false;
        };

        struct SeamTrace
        {
            const char* event = nullptr;
            std::array<SeamHand, 2> hands{}; // right, left; last rendered frame
            std::array<SeamHand, 2> baseline{};
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

        float distance(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const auto d = a - b;
            return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        }

        // Rendered shapes' world bound centres, bounded walk.
        bool measureGeometry(const RE::NiAVObject* root, RE::NiPoint3& center, RE::NiPoint3& tip)
        {
            std::array<const RE::NiAVObject*, 96> pending{ root };
            std::array<RE::NiPoint3, 32> centers{};
            std::size_t count = 1, shapes = 0;
            for (std::size_t i = 0; i < count && shapes < centers.size(); ++i) {
                const auto* object = pending[i];
                if (object->GetAppCulled()) continue;
                if (const auto* node = const_cast<RE::NiAVObject*>(object)->IsNode()) {
                    for (const auto& child : node->children) {
                        if (child && count < pending.size()) pending[count++] = child.get();
                    }
                    continue;
                }
                const auto& bound = object->worldBound;
                if (std::isfinite(bound.center.x) && std::isfinite(bound.center.y) && std::isfinite(bound.center.z) &&
                    bound.fRadius > 0.0f)
                    centers[shapes++] = bound.center;
            }
            if (!shapes) return false;
            center = {};
            for (std::size_t i = 0; i < shapes; ++i) center += centers[i];
            center = center * (1.0f / static_cast<float>(shapes));
            tip = center;
            float best = -1.0f;
            for (std::size_t i = 0; i < shapes; ++i) {
                const float d = distance(centers[i], center);
                if (d > best) { best = d; tip = centers[i]; }
            }
            return true;
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
                        hand.source = candidate.source;
                        hand.geometry = measureGeometry(candidate.node, hand.center, hand.tip);
                    }
                }
            }
            return hands;
        }

        // Geometry point in the hand's frame.
        RE::NiPoint3 inHand(const SeamHand& hand, const RE::NiPoint3& world)
        {
            return transform_math::worldPointToLocal(hand.handWorld, world);
        }

        bool comparable(const SeamHand& a, const SeamHand& b)
        {
            return a.weapon && a.geometry && a.handValid && b.weapon && b.geometry && b.handValid;
        }
    }

    void PhysicsInteraction::armTransitionSeamTrace(const char* event) noexcept
    {
        try {
            // A transition inside an open window keeps its baseline.
            if (!s_seam.framesLeft) {
                s_seam.baseline = s_seam.hands;
                s_seam.frame = 0;
                s_seam.elapsedMs = 0.0f;
            }
            s_seam.event = event;
            s_seam.framesLeft = kSeamFrames;
            ROCK_LOG_INFO(Weapon, "SEAM begin event={} right={} left={}", event,
                seamSourceName(s_seam.baseline[0].source), seamSourceName(s_seam.baseline[1].source));
        } catch (...) {
            s_seam = {};
        }
    }

    void PhysicsInteraction::sampleTransitionSeam() noexcept
    {
        try {
            const auto hands = observeSeamHands(_equipped.transition.bridgeForTrace(), _twoHandedGrip.isFiringHandLeft());
            if (!s_seam.framesLeft) {
                s_seam.hands = hands;
                return;
            }
            --s_seam.framesLeft;
            ++s_seam.frame;
            s_seam.elapsedMs += runtime_state::currentFrame().deltaSeconds * 1000.0f;
            struct Metrics { float worldC = -1, worldF = -1, handC = -1, handF = -1, baseC = -1, baseF = -1, handMove = -1, handTurn = -1; bool switched = false; };
            std::array<Metrics, 2> m{};
            for (std::size_t h = 0; h < hands.size(); ++h) {
                const auto& previous = s_seam.hands[h];
                const auto& base = s_seam.baseline[h];
                const auto& now = hands[h];
                auto& out = m[h];
                out.switched = previous.source != now.source || previous.weapon != now.weapon;
                if (previous.weapon && previous.geometry && now.weapon && now.geometry) {
                    out.worldC = distance(previous.center, now.center);
                    out.worldF = distance(previous.tip, now.tip);
                }
                if (comparable(previous, now)) {
                    out.handC = distance(inHand(previous, previous.center), inHand(now, now.center));
                    out.handF = distance(inHand(previous, previous.tip), inHand(now, now.tip));
                }
                if (comparable(base, now)) {
                    out.baseC = distance(inHand(base, base.center), inHand(now, now.center));
                    out.baseF = distance(inHand(base, base.tip), inHand(now, now.tip));
                }
                if (previous.handValid && now.handValid) {
                    out.handMove = dynamic_weapon_collision_policy::translationDeltaGameUnits(previous.handWorld, now.handWorld);
                    out.handTurn = dynamic_weapon_collision_policy::rotationDeltaDegrees(previous.handWorld, now.handWorld);
                }
            }
            ROCK_LOG_INFO(Weapon,
                "SEAM frame={} event={} ms={:.1f} "
                "R[{} n={} switch={} geomWorldStep={:.2f}/{:.2f}gu geomInHandStep={:.2f}/{:.2f}gu geomInHandVsBefore={:.2f}/{:.2f}gu hand={:.2f}gu/{:.2f}deg] "
                "L[{} n={} switch={} geomWorldStep={:.2f}/{:.2f}gu geomInHandStep={:.2f}/{:.2f}gu geomInHandVsBefore={:.2f}/{:.2f}gu hand={:.2f}gu/{:.2f}deg] nativeState={}",
                s_seam.frame, s_seam.event ? s_seam.event : "?", s_seam.elapsedMs,
                seamSourceName(hands[0].source), hands[0].visibleCount, m[0].switched, m[0].worldC, m[0].worldF,
                m[0].handC, m[0].handF, m[0].baseC, m[0].baseF, m[0].handMove, m[0].handTurn,
                seamSourceName(hands[1].source), hands[1].visibleCount, m[1].switched, m[1].worldC, m[1].worldF,
                m[1].handC, m[1].handF, m[1].baseC, m[1].baseF, m[1].handMove, m[1].handTurn,
                f4vr::getNativeWeaponState(f4vr::getPlayer()));
            s_seam.hands = hands;
        } catch (...) {
            s_seam = {};
        }
    }
}
