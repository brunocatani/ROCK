#pragma once

#include <cstdint>

namespace rock::collision_system_owner
{
    // VR constructors 141E07732/141E0B0EE, 141E0A0A3/141E2A178,
    // 141E0A3D3 and 142996C15/142996D2E install these exact tables.
    inline constexpr std::uintptr_t kNativeBody = 0x2E835D8;
    inline constexpr std::uintptr_t kUnlinkedBody = 0x2E838F8;
    inline constexpr std::uintptr_t kProxy = 0x2E83A98;
    inline constexpr std::uintptr_t kSceneOnly = 0x30C5AA8;

    enum class Stage { Native, Proxy, SceneOnly, SourceUnreadable, SourceType, ProxyTarget, ProxyTargetType };
    struct Resolution
    {
        std::uintptr_t owner{}, sourceVtable{}, targetVtable{};
        Stage stage{ Stage::SourceUnreadable };
        bool usable() const { return owner && (stage == Stage::Native || stage == Stage::Proxy); }
    };

    inline constexpr bool nativeBody(std::uintptr_t table) { return table == kNativeBody || table == kUnlinkedBody; }

    // The native proxy constructor receives a body collision object, not a
    // physics system (141DFC5FB..601). UpdateWorldData independently follows
    // proxy+20 -> body+20 -> system at 141E0A43C..452 and 141E0A495..4BE.
    // Never reinterpret an unknown/proxy target as a bhkPhysicsSystem.
    template<class ReadVtable, class ReadTarget>
    Resolution resolve(std::uintptr_t source, ReadVtable&& readVtable, ReadTarget&& readTarget)
    {
        Resolution result{};
        if (!source || !readVtable(source, result.sourceVtable)) return result;
        if (nativeBody(result.sourceVtable)) { result.owner = source; result.stage = Stage::Native; return result; }
        if (result.sourceVtable == kSceneOnly) { result.stage = Stage::SceneOnly; return result; }
        if (result.sourceVtable != kProxy) { result.stage = Stage::SourceType; return result; }
        std::uintptr_t target{};
        result.stage = Stage::ProxyTarget;
        if (!readTarget(source, target) || !target || target == source || !readVtable(target, result.targetVtable)) return result;
        result.stage = Stage::ProxyTargetType;
        if (!nativeBody(result.targetVtable)) return result;
        result.owner = target;
        result.stage = Stage::Proxy;
        return result;
    }
}
