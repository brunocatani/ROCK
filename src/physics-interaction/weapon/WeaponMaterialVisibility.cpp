#include "physics-interaction/weapon/WeaponMaterialVisibility.h"
#include "physics-interaction/weapon/WeaponMaterialVisibilityPolicy.h"
#include "physics-interaction/weapon/WeaponSceneTraversal.h"
#include "physics-interaction/weapon/WeaponTextureAlphaCache.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"

#include "RE/Bethesda/BSGeometry.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTexture.h"

#include <cstdint>
#include <cstring>
#include <cmath>

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
            char sourcePath[260]{};
            AlphaState alpha{};
            texture_alpha::Result coverage = texture_alpha::Result::Unavailable;
            bool lighting = false;
            unsigned stage = 0;
        };

        template <std::size_t Size>
        bool copyTraceText(char (&target)[Size], const char* source)
        {
            if (!source || reinterpret_cast<std::uintptr_t>(source) < 0x10000) return false;
            std::size_t i = 0;
            for (; i + 1 < Size && source[i]; ++i) target[i] = source[i];
            return i != 0 && source[i] == 0;
        }

        bool plausible(const void* pointer)
        {
            const auto address = reinterpret_cast<std::uintptr_t>(pointer);
            return address >= 0x10000 && address < 0x0000800000000000 && (address & 7) == 0;
        }

        void describeDiffuse(const char* material, ReadTrace& out)
        {
            // Renderer diagnostics cannot change the source-material verdict.
            __try {
                const auto* texture = *reinterpret_cast<const RE::NiTexture* const*>(material + 0x38);
                out.diffuse = reinterpret_cast<std::uintptr_t>(texture);
                if (plausible(texture)) copyTraceText(out.path, texture->name.c_str());
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                copyTraceText(out.path, "<unavailable>");
            }
        }

        // FO4VR 1.2.72 raw witnesses: property material +58 is set by
        // 14278EF20 and read by 1427A6E20; diffuse +38 is read by 1427A6E20
        // and retained/released by 14280B8B0/14280B9F0. NiTexture name +10
        // is constructed/destroyed by 141C24D30/141C24180. The shared lighting
        // material header's +40 diffuse field is NOT the VR layout.
        bool readMaterial(const RE::BSTriShape* shape, ReadTrace& out, bool trace)
        {
            __try {
                out.stage = 1;
                if (!plausible(shape)) return false;
                const auto& properties = shape->GetRuntimeData().properties;
                const char* shader = nullptr;
                for (unsigned index = 0; index < 2; ++index) {
                    auto* property = properties[index].get();
                    if (trace) out.properties[index] = reinterpret_cast<std::uintptr_t>(property);
                    if (!property) continue;
                    out.stage = 2;
                    if (!plausible(property)) return false;
                    auto* rtti = property->GetRTTI();
                    for (unsigned depth = 0; rtti && depth < 16; ++depth, rtti = rtti->GetBaseRTTI()) {
                        if (!plausible(rtti)) return false;
                        const auto* name = rtti->GetName();
                        if (trace && depth == 0) copyTraceText(out.propertyTypes[index], name);
                        if (name && std::strcmp(name, "BSLightingShaderProperty") == 0) {
                            shader = reinterpret_cast<const char*>(property);
                            break;
                        }
                        if (name && std::strcmp(name, "NiAlphaProperty") == 0) {
                            // 1401DA7A0 initializes +28/+2A; alpha setters at
                            // 1401DB5D0 / 1401DB770 update these packed flags.
                            const auto* alpha = reinterpret_cast<const char*>(property);
                            out.alpha = { *reinterpret_cast<const std::uint16_t*>(alpha + 0x28),
                                *reinterpret_cast<const std::uint8_t*>(alpha + 0x2A), true };
                            break;
                        }
                    }
                }
                out.lighting = shader != nullptr;
                if (!shader) { out.stage = 0; return true; }
                out.shaderFlags = *reinterpret_cast<const std::uint64_t*>(shader + 0x30);
                out.stage = 3;
                const auto* material = *reinterpret_cast<const char* const*>(shader + 0x58);
                out.material = reinterpret_cast<std::uintptr_t>(material);
                if (!plausible(material)) return false;
                // Native alpha setter/getter: 1427A7080 / 1427A6FA0.
                out.materialAlpha = *reinterpret_cast<const float*>(material + 0x70);
                if (!std::isfinite(out.materialAlpha) || out.materialAlpha < 0.0f) return false;
                if (trace) describeDiffuse(material, out);
                if (!zeroAlphaIsInvisible(out.alpha) || out.materialAlpha == 0.0f) {
                    out.stage = 0;
                    return true;
                }
                out.stage = 6;
                // +68 retains the active texture set even when VR substitutes
                // an opaque default texture for the invisible DDS. Witnesses:
                // 14280C3D0 assigns it; 14280B9F0 releases it. Diffuse filename
                // +10 is constructed at 1404AD450 and read at 1427918A0/8C0.
                const auto* set = *reinterpret_cast<const RE::NiObject* const*>(material + 0x68);
                if (!plausible(set)) return false;
                const auto* rtti = set->GetRTTI();
                if (!plausible(rtti) || !rtti->GetName() || std::strcmp(rtti->GetName(), "BSShaderTextureSet") != 0) return false;
                out.stage = 7;
                const auto* filename = reinterpret_cast<const RE::BSFixedString*>(reinterpret_cast<const char*>(set) + 0x10);
                if (!copyTraceText(out.sourcePath, filename->c_str())) return false;
                out.stage = 0;
                return true;
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
            ReadTrace local{};
            auto& data = trace ? *trace : local;
            const bool available = readMaterial(shape, data, trace != nullptr);
            if (!available) {
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "Weapon material visibility unavailable: shape={:X} stage={} (shape/property/material/diffuse/path/texture-set/source-path)",
                    reinterpret_cast<std::uintptr_t>(shape), data.stage);
                return { false, false };
            }
            if (!data.lighting || !zeroAlphaIsInvisible(data.alpha)) return { false, true };
            if (data.materialAlpha == 0.0f) return { true, true };
            data.coverage = weapon_texture_alpha::query(data.sourcePath);
            return { data.coverage == texture_alpha::Result::Transparent, true };
        }
    }

    bool isHidden(const RE::BSTriShape* shape)
    {
        const auto material = inspect(shape);
        // Missing evidence is not permission to remove installed geometry.
        return material.hidden;
    }

    bool State::update(std::span<RE::NiAVObject* const> roots, std::uint32_t weaponFormID, bool traceDetails)
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
        if (traceDetails && (_traceFrame == 1 || _traceFrame == 120)) {
            ROCK_LOG_INFO(Weapon, "Weapon material trace begin revision=4 form={:08X} roots={} rootKey={:016X} pass={}",
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
                bool traceShape = traceDetails && _traceFrame <= 120 && _traceCount < _tracedShapes.size();
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
                        "Weapon material trace shape='{}' node={:X} properties=({:X},'{}';{:X},'{}') lighting={} material={:X} diffuse={:X} path='{}' source='{}' shaderFlags={:016X} materialAlpha={} alphaFlags={:04X} alphaRef={} coverage={} stage={} available={} excluded={} appCulled={} ownedCull={}",
                        node->name.c_str(), shapeIdentity, trace.properties[0], trace.propertyTypes[0],
                        trace.properties[1], trace.propertyTypes[1], trace.lighting, trace.material, trace.diffuse,
                        trace.path, trace.sourcePath, trace.shaderFlags, trace.materialAlpha, trace.alpha.flags,
                        trace.alpha.threshold, static_cast<unsigned>(trace.coverage), trace.stage, material.available,
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
