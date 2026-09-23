#include "physics-interaction/weapon/WeaponMaterialVisibility.h"
#include "physics-interaction/weapon/WeaponMaterialVisibilityPolicy.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "RE/Bethesda/BSGeometry.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTexture.h"

#include <cstdint>
#include <cstring>

namespace rock::weapon_material_visibility
{
    namespace
    {
        bool plausible(const void* pointer)
        {
            const auto address = reinterpret_cast<std::uintptr_t>(pointer);
            return address >= 0x10000 && address < 0x0000800000000000 && (address & 7) == 0;
        }

        // FO4VR 1.2.72 raw witnesses: property material +58 is set by
        // 14278EF20 and read by 1427A6E20; diffuse +38 is read by 1427A6E20
        // and retained/released by 14280B8B0/14280B9F0. NiTexture name +10
        // is constructed/destroyed by 141C24D30/141C24180. The shared lighting
        // material header's +40 diffuse field is NOT the VR layout.
        bool readInvisibleTexture(const RE::BSTriShape* shape, unsigned& stage)
        {
            __try {
                stage = 1;
                if (!plausible(shape)) return false;
                for (const auto& owner : shape->GetRuntimeData().properties) {
                    auto* property = owner.get();
                    if (!property) continue;
                    stage = 2;
                    if (!plausible(property)) return false;
                    auto* rtti = property->GetRTTI();
                    bool lighting = false;
                    for (unsigned depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
                        if (!plausible(rtti)) return false;
                        const auto* name = rtti->GetName();
                        if (name && std::strcmp(name, "BSLightingShaderProperty") == 0) {
                            lighting = true;
                            break;
                        }
                    }
                    if (!lighting) continue;
                    stage = 3;
                    const auto* material = *reinterpret_cast<const char* const*>(reinterpret_cast<const char*>(property) + 0x58);
                    if (!plausible(material)) return false;
                    stage = 4;
                    const auto* texture = *reinterpret_cast<const RE::NiTexture* const*>(material + 0x38);
                    if (!plausible(texture)) return false;
                    stage = 5;
                    const char* path = texture->name.c_str();
                    if (!path || reinterpret_cast<std::uintptr_t>(path) < 0x10000) return false;
                    // No path copies or unbounded string scan in the frame loop.
                    std::size_t length = 0;
                    while (length < 260 && path[length]) ++length;
                    if (length == 0 || length == 260) return false;
                    stage = 0;
                    return isInvisibleTexture({ path, length });
                }
                stage = 0; // Non-lighting geometry is outside this material fix.
                return false;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        struct MaterialRead
        {
            bool hidden;
            bool available;
        };

        MaterialRead inspect(const RE::BSTriShape* shape)
        {
            unsigned stage = 0;
            const bool hidden = readInvisibleTexture(shape, stage);
            if (stage != 0) {
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "Weapon material visibility unavailable: shape={:X} stage={} (shape/property/material/diffuse/path)",
                    reinterpret_cast<std::uintptr_t>(shape), stage);
            }
            return { hidden, stage == 0 };
        }
    }

    bool isHidden(const RE::BSTriShape* shape)
    {
        const auto material = inspect(shape);
        return material.hidden || !material.available;
    }

    bool State::update(std::span<RE::NiAVObject* const> roots)
    {
        for (std::size_t i = 0; i < _count; ++i) _culled[i].seen = false;
        bool changed = false;
        bool truncated = false;
        for (auto* root : roots) {
            const auto traversal = weapon_scene::visitScene(root, [&](RE::NiAVObject* node) {
                auto* shape = node->IsTriShape();
                if (!shape) return true;
                std::size_t index = 0;
                while (index < _count && _culled[index].node.get() != node) ++index;
                const bool tracked = index < _count;
                const bool owned = tracked && _culled[index].owned;
                const auto material = inspect(shape);
                if (!material.available) {
                    // A loading/invalid material is not evidence that a formerly
                    // absent part was installed. Keep only our existing cull.
                    if (tracked) {
                        _culled[index].seen = true;
                        if (owned) node->flags.flags |= 1ull;
                    }
                    return true;
                }
                const bool hidden = material.hidden;
                const bool wasCulled = node->GetAppCulled();
                const auto decision = decideCull(hidden, wasCulled, owned);
                if (hidden && !tracked) {
                    if (_count == _culled.size()) {
                        truncated = true;
                        return true;
                    }
                    _culled[_count++].node.reset(node);
                    changed = true;
                }
                if (hidden || tracked) {
                    _culled[index].seen = hidden;
                    _culled[index].owned = decision.owned;
                }
                if (decision.culled != wasCulled) {
                    if (decision.culled) node->flags.flags |= 1ull;
                    else node->flags.flags &= ~1ull;
                }
                if (hidden != tracked) {
                    ROCK_LOG_INFO(Weapon, "Weapon material visibility: shape='{}' invisibleMaterial={} culled={}",
                        node->name.c_str(), hidden, decision.culled);
                }
                return true;
            });
            truncated |= traversal.truncated;
        }
        for (std::size_t i = 0; i < _count;) {
            if (_culled[i].seen) { ++i; continue; }
            // Detached graphs remain alive through our reference until this
            // exact flag is restored; no transient raw pointer survives a frame.
            if (_culled[i].owned) _culled[i].node->flags.flags &= ~1ull;
            changed = true;
            _culled[i] = std::move(_culled[--_count]);
            _culled[_count] = {};
        }
        if (truncated) {
            ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                "Weapon material visibility traversal/cull capacity exhausted; collider extraction still rejects invisible materials");
        }
        return changed;
    }

    void State::clear()
    {
        for (std::size_t i = 0; i < _count; ++i) {
            if (_culled[i].owned) _culled[i].node->flags.flags &= ~1ull;
            _culled[i] = {};
        }
        _count = 0;
    }
}
