// Developer weapon visibility trace (ROCK_Developer.toml
// [LoggingAndProfiling] WeaponVisibilityTrace). Equipped weapons that vanish in
// some places and not others, whole or only their sights, have several possible
// owners, and each leaves a different mark in the presented frame:
//
//   culled      a node above the weapon root has AppCulled; the line names it
//               and, when ROCK set it, the ROCK call site (rock=file:line)
//   parts       nodes inside the weapon are culled (sights, scope parts)
//   geometry    the weapon root has no shapes left (lost mod parts)
//   bound       a shape's world bound is far from where it is drawn, so the
//               engine's view culling tests the wrong place
//   away        the weapon root sits far from its hand
//   scale       zero or non-finite scale or position
//   bridge      ROCK's own equip copy presents the weapon instead
//
// A state change logs one VISIBILITY line per hand at once; a VISIBILITY tick
// with the full state, cell, position and view follows once per second, so a
// vanish that leaves no mark in the scene graph (engine culling, render side)
// still has a timestamped record of what the weapon looked like at that moment.
// Frame thread, AfterWorldFinal. The walk is bounded and runs only while the
// setting is on.
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/equip/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/native/NativeHandEquipment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace rock
{
    namespace
    {
        constexpr std::size_t kWalkNodes = 256;
        constexpr int kParentDepth = 48;
        // A drawn shape whose world bound centre is this far from its drawn
        // centre is tested for view culling in the wrong place.
        constexpr float kStaleBoundGameUnits = 15.0f;
        // A weapon root this far from its hand is not where the player looks for it.
        constexpr float kAwayFromHandGameUnits = 60.0f;
        constexpr float kTickSeconds = 1.0f;

        const char* nameOf(const RE::NiAVObject* node)
        {
            const char* name = node ? node->name.c_str() : nullptr;
            return name && *name ? name : "<unnamed>";
        }

        // Short source name for a ROCK call site.
        const char* shortFile(const char* path)
        {
            if (!path)
                return "?";
            const char* last = path;
            for (const char* c = path; *c; ++c) {
                if (*c == '\\' || *c == '/')
                    last = c + 1;
            }
            return last;
        }

        bool finite(const RE::NiPoint3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

        float distance(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const auto d = a - b;
            return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        }

        struct SlotState
        {
            const RE::NiAVObject* root = nullptr;
            std::uint32_t formID = 0;
            bool left = false;
            // Above the root.
            const RE::NiAVObject* culledAbove = nullptr;
            int culledDepth = -1;
            const RE::NiAVObject* top = nullptr;
            // Inside the root.
            std::uint32_t shapes = 0, drawnShapes = 0, culledInside = 0;
            const RE::NiAVObject* firstCulledInside = nullptr;
            bool walkTruncated = false;
            float worstBoundOffset = 0.0f;
            const RE::NiAVObject* worstBoundShape = nullptr;
            // Placement.
            RE::NiPoint3 rootWorld{};
            float rootScale = 0.0f;
            float handDistance = -1.0f;
            float hmdDistance = -1.0f;
            float viewDegrees = -1.0f; // angle off the headset's forward to the root
            bool bridge = false;

            [[nodiscard]] bool badPlacement() const
            {
                return !finite(rootWorld) || !std::isfinite(rootScale) || rootScale <= 0.0001f;
            }
            [[nodiscard]] bool away() const { return handDistance > kAwayFromHandGameUnits; }
            [[nodiscard]] bool staleBound() const { return worstBoundOffset > kStaleBoundGameUnits; }

            // The first reason in order of how completely it hides the weapon.
            [[nodiscard]] const char* reason() const
            {
                if (!root)
                    return "none";
                if (bridge)
                    return "bridge";
                if (culledAbove)
                    return "culled";
                if (badPlacement())
                    return "scale";
                if (drawnShapes == 0)
                    return "geometry";
                if (away())
                    return "away";
                if (staleBound())
                    return "bound";
                if (culledInside)
                    return "parts";
                return "visible";
            }

            // Fields whose change is a new state worth a line at once. Parts
            // culled inside the weapon (muzzle flashes toggle every shot) are
            // reported by the tick only.
            [[nodiscard]] bool sameState(const SlotState& other) const
            {
                return root == other.root && formID == other.formID && bridge == other.bridge &&
                    culledAbove == other.culledAbove && top == other.top &&
                    (drawnShapes == 0) == (other.drawnShapes == 0) && badPlacement() == other.badPlacement() &&
                    away() == other.away() &&
                    staleBound() == other.staleBound();
            }
        };

        struct TraceState
        {
            std::array<SlotState, 2> slots{};
            bool hasPrevious = false;
            float secondsToTick = 0.0f;
        };

        TraceState s_trace{};

        void walkWeapon(SlotState& state)
        {
            std::array<const RE::NiAVObject*, kWalkNodes> pending{ state.root };
            std::size_t count = 1;
            for (std::size_t i = 0; i < count; ++i) {
                const auto* object = pending[i];
                auto* mutableObject = const_cast<RE::NiAVObject*>(object);
                const bool culled = object != state.root && object->GetAppCulled();
                if (const auto* node = mutableObject->IsNode()) {
                    if (culled) {
                        if (state.culledInside++ == 0)
                            state.firstCulledInside = object;
                        continue;
                    }
                    for (const auto& child : node->children) {
                        if (!child)
                            continue;
                        if (count < pending.size())
                            pending[count++] = child.get();
                        else
                            state.walkTruncated = true;
                    }
                    continue;
                }
                auto* geometry = mutableObject->IsGeometry();
                if (!geometry)
                    geometry = mutableObject->IsTriShape();
                if (!geometry)
                    continue;
                ++state.shapes;
                if (culled) {
                    if (state.culledInside++ == 0)
                        state.firstCulledInside = object;
                    continue;
                }
                ++state.drawnShapes;
                // Skinned shapes are drawn from their bones; their bound is
                // not comparable to their own transform.
                if (geometry->GetRuntimeData().skinInstance)
                    continue;
                const auto drawn = transform_math::localPointToWorld(object->world, geometry->GetRuntimeData().modelBound.center);
                const auto& bound = object->worldBound;
                if (!finite(drawn) || !finite(bound.center) || !(bound.fRadius > 0.0f))
                    continue;
                const float offset = distance(drawn, bound.center);
                if (offset > state.worstBoundOffset) {
                    state.worstBoundOffset = offset;
                    state.worstBoundShape = object;
                }
            }
        }

        SlotState observeSlot(const native_hand_equipment::EquippedView& view, const bool left, const bool bridge,
            const RE::NiAVObject* hmd)
        {
            SlotState state{};
            if (!view.weapon || !view.root)
                return state;
            state.root = view.root;
            state.formID = view.weapon->GetFormID();
            state.left = left;
            state.bridge = bridge;
            const RE::NiAVObject* node = view.root;
            for (int depth = 0; node && depth < kParentDepth; ++depth) {
                if (!state.culledAbove && node->GetAppCulled()) {
                    state.culledAbove = node;
                    state.culledDepth = depth;
                }
                state.top = node;
                node = node->parent;
            }
            walkWeapon(state);
            state.rootWorld = view.root->world.translate;
            state.rootScale = view.root->world.scale;
            RE::NiTransform hand{};
            if (frik_visual_authority::tryGetPresentedHandWorldTransform(left, hand))
                state.handDistance = distance(hand.translate, state.rootWorld);
            if (hmd) {
                state.hmdDistance = distance(hmd->world.translate, state.rootWorld);
                const auto forward = transform_math::rotateLocalVectorToWorld(hmd->world.rotate, RE::NiPoint3{ 0, 1, 0 });
                const auto to = state.rootWorld - hmd->world.translate;
                const float lengths = std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z) *
                    std::sqrt(to.x * to.x + to.y * to.y + to.z * to.z);
                if (lengths > 0.0001f) {
                    const float cosine =
                        std::clamp((forward.x * to.x + forward.y * to.y + forward.z * to.z) / lengths, -1.0f, 1.0f);
                    state.viewDegrees = std::acos(cosine) * 57.29578f;
                }
            }
            return state;
        }

        void logSlot(const char* kind, std::size_t slot, const SlotState& state)
        {
            equipped_weapon_visual_state::VisibilityWrite above{}, inside{};
            const bool rockAbove = equipped_weapon_visual_state::lastVisibilityWrite(state.culledAbove, above) &&
                !above.visible;
            const bool rockInside = equipped_weapon_visual_state::lastVisibilityWrite(state.firstCulledInside, inside) &&
                !inside.visible;
            ROCK_LOG_INFO(Weapon,
                "VISIBILITY {} slot={} hand={} weapon={:08X} reason={} culledAbove='{}'@{} rock={}:{} top='{}' "
                "shapes={}/{} culledInside={} firstCulledInside='{}' rockInside={}:{} truncated={} "
                "worstBound={:.1f}gu '{}' hand={:.1f}gu hmd={:.1f}gu view={:.0f}deg scale={:.3f} "
                "pos=({:.1f},{:.1f},{:.1f})",
                kind, slot, state.left ? "left" : "right", state.formID, state.reason(),
                state.culledAbove ? nameOf(state.culledAbove) : "none", state.culledDepth,
                rockAbove ? shortFile(above.file) : "-", rockAbove ? above.line : 0, state.top ? nameOf(state.top) : "none",
                state.drawnShapes, state.shapes, state.culledInside,
                state.firstCulledInside ? nameOf(state.firstCulledInside) : "none",
                rockInside ? shortFile(inside.file) : "-", rockInside ? inside.line : 0, state.walkTruncated,
                state.worstBoundOffset, state.worstBoundShape ? nameOf(state.worstBoundShape) : "none",
                state.handDistance, state.hmdDistance, state.viewDegrees, state.rootScale, state.rootWorld.x,
                state.rootWorld.y, state.rootWorld.z);
        }

        void logContext(const RE::NiAVObject* hmd)
        {
            auto* player = f4vr::getPlayer();
            const auto* cell = player ? player->parentCell : nullptr;
            const char* cellName = cell ? cell->GetFormEditorID() : nullptr;
            const auto* world = cell && cell->IsExterior() ? cell->worldSpace : nullptr;
            auto* ui = RE::UI::GetSingleton();
            const bool scopeMenu = ui && ui->GetMenuOpen(RE::BSFixedString{ "ScopeMenu" });
            const RE::NiPoint3 hmdPos = hmd ? hmd->world.translate : RE::NiPoint3{};
            ROCK_LOG_INFO(Weapon,
                "VISIBILITY context cell={:08X} '{}' {} worldspace={:08X} hmd=({:.1f},{:.1f},{:.1f}) scopeMenu={} "
                "weaponState={}",
                cell ? cell->GetFormID() : 0, cellName && *cellName ? cellName : "", cell && cell->IsInterior() ? "interior" : "exterior",
                world ? world->GetFormID() : 0, hmdPos.x, hmdPos.y, hmdPos.z, scopeMenu,
                player ? f4vr::getNativeWeaponState(player) : 0);
        }
    }

    void PhysicsInteraction::sampleWeaponVisibility() noexcept
    {
        if (!g_rockConfig.rockWeaponVisibilityTrace) {
            if (s_trace.hasPrevious)
                s_trace = {};
            return;
        }
        try {
            const auto* nodes = f4vr::getPlayerNodes();
            const RE::NiAVObject* hmd = nodes ? nodes->HmdNode : nullptr;
            const auto views = native_hand_equipment::equippedViews();
            const auto& bridge = _equipped.transition.bridgeForTrace();
            const bool bridgePresented = bridge.isModelPresented();
            const bool slot0Left = _twoHandedGrip.isFiringHandLeft();
            const bool slot1Left = views[1].weapon && native_hand_equipment::secondaryHandIsLeft();
            const std::array<SlotState, 2> slots{
                observeSlot(views[0], slot0Left, bridgePresented && bridge.isLeftHandForTrace() == slot0Left, hmd),
                observeSlot(views[1], slot1Left, false, hmd),
            };

            bool changed = !s_trace.hasPrevious;
            for (std::size_t i = 0; i < slots.size() && !changed; ++i)
                changed = !slots[i].sameState(s_trace.slots[i]);

            s_trace.secondsToTick -= runtime_state::currentFrame().deltaSeconds;
            const bool tick = s_trace.secondsToTick <= 0.0f;
            if (changed || tick) {
                logContext(hmd);
                for (std::size_t i = 0; i < slots.size(); ++i) {
                    if (slots[i].root || s_trace.slots[i].root)
                        logSlot(changed ? "change" : "tick", i, slots[i]);
                }
                s_trace.secondsToTick = kTickSeconds;
            }
            s_trace.slots = slots;
            s_trace.hasPrevious = true;
        } catch (...) {
            s_trace = {};
        }
    }
}
