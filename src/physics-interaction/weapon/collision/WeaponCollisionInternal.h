#pragma once

/*
 * Helpers shared by the WeaponCollision translation units.
 *
 * WeaponCollision is one class implemented across several .cpp files. A helper
 * that more than one of them needs is promoted here into
 * rock::weapon_collision_detail rather than duplicated; a helper used by exactly
 * one of them stays in that file's anonymous namespace, because internal linkage
 * is still the default.
 *
 * The named namespace also keeps these names away from the same-named local
 * helpers other physics-interaction TUs (Hand.cpp, DynamicHandCollision.cpp)
 * define for themselves.
 *
 * INTERNAL. Never reachable from any public include tree.
 */

#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "physics-interaction/weapon/collision/WeaponEffectGeometryPolicy.h"

#include "rock_support/Fo4VrRuntime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace rock::weapon_collision_detail
{
    constexpr std::size_t MAX_CONVEX_HULL_POINTS = 0xFC;
    constexpr float MIN_HULL_DIAGONAL_GAME_UNITS = 0.5f;

    constexpr std::array<const char*, 11> WEAPON_ANIM_NODE_DUMP_TARGETS{
        "Weapon",
        "WeaponLeft",
        "ProjectileNode",
        "AnimObjectR1",
        "AnimObjectR2",
        "AnimObjectR3",
        "AnimObjectL1",
        "AnimObjectL2",
        "AnimObjectL3",
        "AnimObjectA",
        "AnimObjectB",
    };

    constexpr int WEAPON_ANIM_NODE_DUMP_MAX_DEPTH = 32;
    constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_MATCHES_PER_NAME = 32;
    constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_VISITED_NODES = 4096;
    constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_CHILD_NAMES = 16;
    constexpr std::size_t WEAPON_ANIM_NODE_DUMP_MAX_SUBTREE_NODES = 4096;
    constexpr int WEAPON_ANIM_NODE_DUMP_MAX_FLATTENED_TRANSFORMS = 768;

    struct QuantizedPointKey
    {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const QuantizedPointKey& rhs) const noexcept { return x == rhs.x && y == rhs.y && z == rhs.z; }
    };

    struct QuantizedPointKeyHash
    {
        std::size_t operator()(const QuantizedPointKey& key) const noexcept
        {
            const auto hx = std::hash<std::int64_t>{}(key.x);
            const auto hy = std::hash<std::int64_t>{}(key.y);
            const auto hz = std::hash<std::int64_t>{}(key.z);
            return hx ^ (hy + 0x9e3779b97f4a7c15ull + (hx << 6) + (hx >> 2)) ^ (hz + 0x9e3779b97f4a7c15ull + (hy << 6) + (hy >> 2));
        }
    };

    struct WeaponMeshRootCandidate
    {
        RE::NiAVObject* root = nullptr;
        const char* label = "";
    };

    struct PointCloudBounds
    {
        RE::NiPoint3 min{};
        RE::NiPoint3 max{};
    };

    /*
     * Scene-scan bounds.
     *
     * Two different kinds of number live here and they must not be confused:
     *  - a DEPTH cap is a SAFETY net. Weapon and OMOD template hierarchies are
     *    shallow, so exceeding one means a corrupt or cyclic parent chain; the
     *    walk stops instead of recursing forever.
     *  - a VISIT cap is a BUDGET. It bounds what one scan may cost on a
     *    pathological modded mesh. Hitting it is a normal outcome that some
     *    callers report, not evidence of corruption.
     */
    // One-shot scans of authored OMOD/NIF templates: small, shallow, and off
    // the per-frame path.
    constexpr int kTemplateScanMaxDepth = 16;
    constexpr std::size_t kTemplateScanMaxVisitedNodes = 512;
    // ROCK's own enrichment containers sit under the ASSEMBLED weapon, which is
    // deeper than a bare template, and the sweep has to reach every one of them
    // or stale geometry survives.
    constexpr int kEnrichmentContainerScanMaxDepth = 24;
    // The template fingerprint is a name list; past this many names it cannot
    // become more distinctive, only more expensive.
    constexpr std::size_t kOmodTemplateSignatureMaxMeshNames = 96;

    // What a bounded walk does after visiting one node.
    enum class TreeWalkAction : std::uint8_t
    {
        Descend,      // keep going into this node's children
        SkipChildren, // this node answered for its whole subtree
        Stop,         // the walk is finished; unwind everything
    };

    // Live state of one bounded walk. `visited` is carried in the state so a
    // single budget can span several roots in the same scan.
    struct BoundedTreeWalkState
    {
        std::size_t visited{ 0 };
        bool stopped{ false };   // a visitor ended the walk on purpose
        bool truncated{ false }; // a bound cut the walk short
    };

    /*
     * Depth-first bounded walk over an NiAVObject hierarchy - the shared shape
     * behind the WeaponCollision scene scans. The visitor is called as
     * visitor(node, depth) and returns a TreeWalkAction.
     *
     * A caller that must not report a clean "not found" should check
     * state.truncated afterwards and fail closed.
     */
    template <class Visitor>
    void boundedTreeWalk(
        RE::NiAVObject* node,
        const int depthCap,
        const std::size_t visitCap,
        BoundedTreeWalkState& state,
        Visitor& visitor,
        const int depth = 0)
    {
        if (!node || state.stopped) {
            return;
        }
        if (depth > depthCap || state.visited >= visitCap) {
            state.truncated = true;
            return;
        }
        ++state.visited;

        switch (visitor(node, depth)) {
        case TreeWalkAction::Stop:
            state.stopped = true;
            return;
        case TreeWalkAction::SkipChildren:
            return;
        case TreeWalkAction::Descend:
            break;
        }

        auto* niNode = node->IsNode();
        if (!niNode) {
            return;
        }
        const auto& children = niNode->children;
        for (auto i = decltype(children.size()){ 0 }; i < children.size() && !state.stopped; ++i) {
            boundedTreeWalk(children[i].get(), depthCap, visitCap, state, visitor, depth + 1);
        }
    }

    /*
     * How many generated hulls one frame may turn into Havok bodies. Creation is
     * sliced because building a whole weapon's worth of hulls in one frame stalls
     * VR; the staged builder in WeaponCollisionBodies.cpp spends this budget and
     * update() reports against it.
     */
    constexpr std::size_t GENERATED_WEAPON_BODY_CREATION_BATCH = 8;

    // ---- node identity and visibility ----

    // Never returns null: a missing node and a missing name both read as "(null)",
    // so every log line and name comparison has something to work with.
    const char* safeNodeName(const RE::NiAVObject* node);

    // Walks the RTTI base chain (bounded) looking for one type name. The engine's
    // own dynamic_cast is not available across the plugin boundary.
    [[nodiscard]] bool niObjectRttiChainContains(const RE::NiObject* object, const char* typeName);

    // Effect-only geometry - muzzle flashes, laser dots, reticles - must never
    // become collision. Combines shader-property evidence, billboard ancestry and
    // the name policy into one verdict.
    [[nodiscard]] weapon_effect_geometry_policy::ExclusionReason classifyGeneratedWeaponEffectGeometry(
        const RE::BSTriShape* triShape);

    // Per-node render visibility: not hidden, not app-culled, non-zero local scale.
    bool weaponVisualNodeVisible(const RE::NiAVObject* node);

    /*
     * Per-node visibility misses renders hidden by an ANCESTOR: a culled,
     * hidden or zero-scale parent (hand bone, skeleton root) hides the whole
     * weapon while every weapon node still reports visible=yes. This walk is
     * the one place that climbs the parent chain looking for that offender.
     */
    struct WeaponVisibilityWalk
    {
        // First node on the chain the renderer would treat as hidden, or null.
        const RE::NiAVObject* firstHidden{ nullptr };
        // The climb hit maxSteps before reaching the top. A weapon/skeleton
        // chain is shallow, so this means a cycle or a corrupt parent chain -
        // callers must treat it as "not provably visible", never as "clean".
        bool boundExhausted{ false };
    };

    [[nodiscard]] WeaponVisibilityWalk walkWeaponVisibilityChain(const RE::NiAVObject* node, int maxSteps);

    // Budget for the per-frame emitter scan: deep enough for any real weapon
    // under the first-person skeleton, cheap enough to run per emitter.
    constexpr int kWeaponEmitterVisibilityAncestorSteps = 32;
    // The OMOD audit is a diagnostic that has to name the true offender, so it
    // climbs further; the bound exists only to stop a corrupt parent chain.
    constexpr int kOmodAuditVisibilityAncestorSteps = 256;

    // ---- finiteness gates ----

    [[nodiscard]] bool pointFinite(const RE::NiPoint3& point) noexcept;

    /*
     * The finiteness gate for every weapon-space transform. Scale participates on
     * purpose: consumers either compose the transform or invert it to bring a world
     * point into node space, and a zero or near-zero scale makes the inverse
     * explode into infinities that then look like valid geometry.
     */
    [[nodiscard]] bool weaponTransformFinite(const RE::NiTransform& transform) noexcept;

    // ---- bounded hierarchy composition ----

    /*
     * Compose the local transform of `descendant` relative to `ancestor` by walking
     * the parent path. Hot: this runs per body per proximity scan. Bounded by
     * kMaximumWeaponLocalHierarchyDepth and fails closed - a break in the chain, a
     * non-finite link, or an over-deep path zeroes the output and returns false
     * rather than handing back a half-composed frame.
     */
    [[nodiscard]] bool tryResolveDescendantLocalTransform(
        const RE::NiAVObject* ancestor,
        const RE::NiAVObject* descendant,
        RE::NiTransform& outDescendantLocal);

    // World form of the above: ancestorWorld composed with the resolved local.
    [[nodiscard]] bool tryResolveDescendantWorldTransform(
        const RE::NiAVObject* ancestor,
        const RE::NiTransform& ancestorWorld,
        const RE::NiAVObject* descendant,
        RE::NiTransform& outDescendantWorld);

    // ---- point-cloud math ----

    QuantizedPointKey quantizePoint(const RE::NiPoint3& point, float grid);
    std::vector<RE::NiPoint3> dedupePointCloud(const std::vector<RE::NiPoint3>& points, float grid);
    float pointCloudDiagonalSquared(const std::vector<RE::NiPoint3>& points);
    // A hull needs four points and enough spread to be a solid rather than a sliver.
    bool pointCloudCanBuildHull(const std::vector<RE::NiPoint3>& points, float sourceScale = 1.0f);
    PointCloudBounds pointCloudBounds(const std::vector<RE::NiPoint3>& points);
    std::array<float, 3> pointToArray(const RE::NiPoint3& point);
    /*
     * Game-space points -> Havok-space points centered on localCenterGame. The
     * source node's own scale is baked in here because Havok never re-applies
     * NiNode scale to a shape it has already built.
     */
    std::vector<RE::NiPoint3> makeCenteredHavokPointCloud(
        const std::vector<RE::NiPoint3>& localPointsGame,
        const RE::NiPoint3& localCenterGame,
        float sourceScale = 1.0f);

    // ---- visual-composition key mixing ----

    void mixWeaponVisualKey(std::uint64_t& key, std::uint64_t value);
    void mixWeaponVisualString(std::uint64_t& key, const char* value);

    // ---- Havok shape refcount ----

    /*
     * Release one reference on a shape. hknpShape's refcount is the low 16 bits of
     * the dword at +0x08; 0xFFFF marks a shape the engine owns permanently and 0
     * means already released, so both are left alone. The compare-exchange loop
     * keeps this safe against the physics thread.
     */
    void shapeRemoveRef(const RE::hknpShape* shape);

    // ---- equipped-weapon form reads ----

    /*
     * The BGSObjectInstanceExtra carrying the installed-OMOD list for the equipped
     * weapon. First-person biped first, then third: the first-person copy is the
     * one whose 3D ROCK actually collides against.
     */
    const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
        const RE::PlayerCharacter* player,
        const RE::TESForm* weaponForm,
        const RE::TBO_InstanceData* instanceData);

    /*
     * attach-point formID -> installed OMOD formID for the equipped weapon.
     * Optionally fills the composition snapshot with the FULL install list,
     * including the disabled and unresolvable entries the map itself omits.
     */
    std::unordered_map<std::uint32_t, std::uint32_t> readEquippedOmodsByAttachPointFormId(
        WeaponCollision::WeaponCompositionSnapshot* outComposition = nullptr);

    /*
     * Does this attachment mod drive the native scope overlay? Defined in
     * WeaponCollisionIdentity.cpp - it is an equipped-OMOD record question - but the
     * generated-source scan needs it as well, to keep overlay-driving OMODs out of
     * the physical part classification.
     */
    [[nodiscard]] bool attachmentModHasNativeScopeOverlayTarget(std::uint32_t omodFormId);

    // Prefix companion to weapon_effect_geometry_policy::containsAsciiInsensitive.
    [[nodiscard]] bool startsWithAsciiInsensitive(std::string_view value, std::string_view prefix);

    // OMOD model templates loaded through the native BSModelDB entry. The two
    // demand flags are NOT interchangeable - see the definitions.
    RE::NiPointer<RE::NiNode> loadCompleteOmodModelTemplate(const std::string& modelPath);
    RE::NiPointer<RE::NiNode> loadGeometryInspectionOmodModelTemplate(const std::string& modelPath);

    // ---- weapon mesh root candidates ----

    /*
     * The single source of truth for WHICH roots ROCK scans for weapon mesh
     * geometry. Weapon mesh collision has to be rooted on the visual weapon
     * tree, not the native collision attachment tree, so several possible
     * visual roots are offered here; every candidate must still prove itself
     * by producing visible triangles before it is used for body creation.
     *
     * Allocation-free on purpose - this runs from per-frame scan paths.
     */
    template <class Visitor>
    void visitGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode, Visitor&& visitor)
    {
        std::array<WeaponMeshRootCandidate, 4> candidates{};
        std::size_t count = 0;
        const auto addUnique = [&](RE::NiAVObject* root, const char* label) {
            if (!root) {
                return;
            }
            for (std::size_t i = 0; i < count; ++i) {
                if (candidates[i].root == root) {
                    return;
                }
            }
            if (count < candidates.size()) {
                candidates[count++] = WeaponMeshRootCandidate{ root, label };
            }
        };

        addUnique(f4vr::getWeaponNode(), "firstPersonSkeleton:Weapon");
        if (auto* playerNodes = f4vr::getPlayerNodes()) {
            addUnique(playerNodes->primaryWeapontoWeaponNode, "PlayerNodes.primaryWeapontoWeaponNode");
            addUnique(playerNodes->primaryWeaponOffsetNOde, "PlayerNodes.primaryWeaponOffsetNode");
        }
        addUnique(updateWeaponNode, "updateWeaponNode");

        for (std::size_t i = 0; i < count; ++i) {
            visitor(candidates[i]);
        }
    }

    // Materialized form of the visitor above, for callers that must keep the list.
    std::vector<WeaponMeshRootCandidate> makeGeneratedWeaponMeshRootCandidates(RE::NiAVObject* updateWeaponNode);

    // Identity of the root SET, so an emitter snapshot can be invalidated when the
    // set of roots ROCK scans changes even though no single root did.
    std::uint64_t makeWeaponEmitterRootSetKey(RE::NiAVObject* updateWeaponNode);

    // ---- installed-OMOD enumeration ----

    /*
     * The one place that walks the equipped weapon's installed-OMOD list.
     *
     * Every caller needs the same two resolutions per entry - the Mod form
     * behind the object id, and the attach-point keyword behind that Mod - and
     * both can fail independently: an index entry can name a form from a plugin
     * that is no longer loaded, and a Mod can carry an attach-point keyword
     * index the keyword table does not have. `omod` and `attachPointKeyword`
     * are therefore passed possibly-null and each caller decides what that
     * means for it.
     *
     * The visitor receives (modIndex, omod, attachPointKeyword) and is called
     * once per entry, in install order.
     */
    template <class Visitor>
    void visitEquippedOmodIndexData(const RE::BGSObjectInstanceExtra* extra, Visitor&& visitor)
    {
        if (!extra || !extra->values) {
            return;
        }
        for (const auto& modIndex : extra->GetIndexData()) {
            auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
            const RE::BGSKeyword* attachPointKeyword = omod ?
                RE::BGSKeyword::GetTypedKeywordByIndex(
                    RE::KeywordType::kAttachPoint,
                    omod->attachPoint.keywordIndex) :
                nullptr;
            visitor(modIndex, omod, attachPointKeyword);
        }
    }
}
