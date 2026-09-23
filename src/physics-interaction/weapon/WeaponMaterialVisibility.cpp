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
        struct ReadTrace
        {
            std::uintptr_t properties[2]{};
            char propertyTypes[2][64]{};
            std::uintptr_t material{};
            std::uintptr_t diffuse{};
            std::uint64_t shaderFlags{};
            float materialAlpha{};
            char path[260]{};
            bool lighting = false;
            unsigned stage = 0;
        };

        template <std::size_t Size>
        void copyTraceText(char (&target)[Size], const char* source)
        {
            if (!source) return;
            for (std::size_t i = 0; i + 1 < Size && source[i]; ++i) target[i] = source[i];
        }

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
        bool readInvisibleTexture(const RE::BSTriShape* shape, unsigned& stage, ReadTrace* trace)
        {
            __try {
                stage = 1;
                if (!plausible(shape)) return false;
                const auto& properties = shape->GetRuntimeData().properties;
                for (unsigned index = 0; index < 2; ++index) {
                    auto* property = properties[index].get();
                    if (trace) trace->properties[index] = reinterpret_cast<std::uintptr_t>(property);
                    if (!property) continue;
                    stage = 2;
                    if (!plausible(property)) return false;
                    auto* rtti = property->GetRTTI();
                    bool lighting = false;
                    for (unsigned depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
                        if (!plausible(rtti)) return false;
                        const auto* name = rtti->GetName();
                        if (trace && depth == 0) copyTraceText(trace->propertyTypes[index], name);
                        if (name && std::strcmp(name, "BSLightingShaderProperty") == 0) {
                            lighting = true;
                            break;
                        }
                    }
                    if (!lighting) continue;
                    if (trace) {
                        trace->lighting = true;
                        trace->shaderFlags = *reinterpret_cast<const std::uint64_t*>(reinterpret_cast<const char*>(property) + 0x30);
                    }
                    stage = 3;
                    const auto* material = *reinterpret_cast<const char* const*>(reinterpret_cast<const char*>(property) + 0x58);
                    if (trace) trace->material = reinterpret_cast<std::uintptr_t>(material);
                    if (!plausible(material)) return false;
                    // Native alpha setter/getter: 1427A7080 / 1427A6FA0.
                    if (trace) trace->materialAlpha = *reinterpret_cast<const float*>(material + 0x70);
                    stage = 4;
                    const auto* texture = *reinterpret_cast<const RE::NiTexture* const*>(material + 0x38);
                    if (trace) trace->diffuse = reinterpret_cast<std::uintptr_t>(texture);
                    if (!plausible(texture)) return false;
                    stage = 5;
                    const char* path = texture->name.c_str();
                    if (!path || reinterpret_cast<std::uintptr_t>(path) < 0x10000) return false;
                    if (trace) copyTraceText(trace->path, path);
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

        MaterialRead inspect(const RE::BSTriShape* shape, ReadTrace* trace = nullptr)
        {
            unsigned stage = 0;
            const bool hidden = readInvisibleTexture(shape, stage, trace);
            if (trace) trace->stage = stage;
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

    bool State::update(std::span<RE::NiAVObject* const> roots, std::uint32_t weaponFormID)
    {
        std::uint64_t rootSignature = 1469598103934665603ull ^ weaponFormID;
        for (const auto* root : roots) {
            rootSignature ^= reinterpret_cast<std::uintptr_t>(root);
            rootSignature *= 1099511628211ull;
        }
        if (rootSignature != _traceRoots) {
            _traceRoots = rootSignature;
            _traceFrame = 0;
            _traceCount = 0;
        }
        if (_traceFrame < 121) ++_traceFrame;
        if (_traceFrame == 120) _traceCount = 0;
        if (_traceFrame == 1 || _traceFrame == 120) {
            ROCK_LOG_INFO(Weapon, "Weapon material trace begin revision=2 form={:08X} roots={} rootKey={:016X} pass={}",
                weaponFormID, roots.size(), rootSignature, _traceFrame == 1 ? "first" : "settled");
        }
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
                const auto shapeIdentity = reinterpret_cast<std::uintptr_t>(shape);
                bool traceShape = _traceFrame <= 120 && _traceCount < _tracedShapes.size();
                if (traceShape) {
                    for (std::size_t i = 0; i < _traceCount; ++i) {
                        if (_tracedShapes[i] == shapeIdentity) { traceShape = false; break; }
                    }
                }
                ReadTrace trace{};
                const auto material = inspect(shape, traceShape ? &trace : nullptr);
                if (traceShape) {
                    _tracedShapes[_traceCount++] = shapeIdentity;
                    ROCK_LOG_INFO(Weapon,
                        "Weapon material trace shape='{}' node={:X} properties=({:X},'{}';{:X},'{}') lighting={} material={:X} diffuse={:X} path='{}' shaderFlags={:016X} materialAlpha={} stage={} available={} excluded={} appCulled={} ownedCull={}",
                        node->name.c_str(), shapeIdentity, trace.properties[0], trace.propertyTypes[0],
                        trace.properties[1], trace.propertyTypes[1], trace.lighting, trace.material, trace.diffuse,
                        trace.path, trace.shaderFlags, trace.materialAlpha, trace.stage, material.available,
                        material.hidden, node->GetAppCulled(), owned);
                }
                if (owned && !node->GetAppCulled()) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "Weapon material cull overwritten: shape='{}' node={:X} available={} invisibleMaterial={}",
                        node->name.c_str(), shapeIdentity, material.available, material.hidden);
                }
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
        _traceCount = 0;
        _traceRoots = 0;
        _traceFrame = 0;
    }
}
