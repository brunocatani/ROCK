#pragma once
#include <cstdint>
#include <type_traits>

// Private experiment contract shared only by ROCK Parallel 2 and PAPER Parallel.
// Independent V1/size negotiation; no released ROCK/PAPER SDK record is changed.
// Copy values on the ROCK animation owner thread, for the requested frame only.
// Right/left poses below are the authored anatomical hands, before role mapping.
namespace loose_reload_experiment
{
    inline constexpr char kQueryExport[] = "ROCK_LooseReloadExperiment_QueryV1";
    struct Point { float x{}, y{}, z{}; };
    struct Rotation { float entry[3][3]{}; };
    struct Transform { Rotation rotate{}; Point translate{}; float scale{1}; };
    struct HandPose
    {
        Transform baseline{}, current{};
        Transform fingers[15]{};
        std::uint32_t fingerMask{};
        std::uint32_t valid{};
    };
    struct Snapshot
    {
        std::uint32_t size{sizeof(Snapshot)}, version{1};
        std::uint64_t frame{}, session{}, binding{}, reload{};
        std::uint32_t reference{}, weapon{}, firingHand{}; // 0 right, 1 left.
        std::uint32_t active{};
        float time{}, duration{};
        Transform weaponWorld{}, firingSeat{};
        HandPose hands[2]{};
    };
    using Query = bool (*)(std::uint64_t frame, Snapshot* output) noexcept;

    template<class T> Transform pack(const T& source) noexcept
    {
        Transform result;
        for (unsigned r=0;r<3;++r) for (unsigned c=0;c<3;++c)
            result.rotate.entry[r][c]=source.rotate.entry[r][c];
        result.translate={source.translate.x,source.translate.y,source.translate.z};
        result.scale=source.scale;
        return result;
    }
    template<class T> T unpack(const Transform& source) noexcept
    {
        T result{};
        for (unsigned r=0;r<3;++r) for (unsigned c=0;c<3;++c)
            result.rotate.entry[r][c]=source.rotate.entry[r][c];
        result.translate={source.translate.x,source.translate.y,source.translate.z};
        result.scale=source.scale;
        return result;
    }
    inline bool current(const Snapshot& value, std::uint64_t frame) noexcept
    {
        return value.size==sizeof(Snapshot) && value.version==1 &&
            value.active==1 && frame && value.frame==frame && value.session &&
            value.binding && value.reload && value.reference && value.weapon && value.firingHand<2;
    }
    static_assert(sizeof(Transform)==52 && sizeof(HandPose)==892 && sizeof(Snapshot)==1952);
    static_assert(std::is_standard_layout_v<Snapshot> && std::is_trivially_copyable_v<Snapshot>);
}
