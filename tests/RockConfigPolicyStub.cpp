#define NOMMNOSOUND
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "F4SE/Impl/PCH.h"

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif
#ifdef MEM_RELEASE
#undef MEM_RELEASE
#endif
#ifdef MEM_COMMIT
#undef MEM_COMMIT
#endif
#ifdef MEM_RESERVE
#undef MEM_RESERVE
#endif
#ifdef PAGE_EXECUTE_READ
#undef PAGE_EXECUTE_READ
#endif
#ifdef PAGE_EXECUTE_READWRITE
#undef PAGE_EXECUTE_READWRITE
#endif
#ifdef MAX_PATH
#undef MAX_PATH
#endif

#include "RockConfig.h"

#include <cmath>
#include <cstddef>

#include "RE/NetImmerse/NiMatrix3.h"

namespace RE
{
    const NiPoint2 NiPoint2::ZERO{ 0.0f, 0.0f };
    const NiPoint3 NiPoint3::ZERO{ 0.0f, 0.0f, 0.0f };
    const NiPoint3A NiPoint3A::ZERO{ 0.0f, 0.0f, 0.0f };
    const NiPoint4 NiPoint4::ZERO{ 0.0f, 0.0f, 0.0f, 0.0f };
    const NiPoint4 NiPoint4::IDENTITY0{ 1.0f, 0.0f, 0.0f, 0.0f };
    const NiPoint4 NiPoint4::IDENTITY1{ 0.0f, 1.0f, 0.0f, 0.0f };
    const NiPoint4 NiPoint4::IDENTITY2{ 0.0f, 0.0f, 1.0f, 0.0f };
    const NiPoint4 NiPoint4::IDENTITY3{ 0.0f, 0.0f, 0.0f, 1.0f };
    const NiMatrix3 NiMatrix3::ZERO{ NiPoint4::ZERO, NiPoint4::ZERO, NiPoint4::ZERO };
    const NiMatrix3 NiMatrix3::IDENTITY{ NiPoint4::IDENTITY0, NiPoint4::IDENTITY1, NiPoint4::IDENTITY2 };

    NiPoint2::NiPoint2(float a_x, float a_y) noexcept :
        x(a_x), y(a_y)
    {}

    float& NiPoint2::operator[](std::size_t a_pos) noexcept { return (&x)[a_pos]; }
    const float& NiPoint2::operator[](std::size_t a_pos) const noexcept { return (&x)[a_pos]; }
    bool NiPoint2::operator==(const NiPoint2& a_rhs) const noexcept { return x == a_rhs.x && y == a_rhs.y; }
    bool NiPoint2::operator!=(const NiPoint2& a_rhs) const noexcept { return !(*this == a_rhs); }
    bool NiPoint2::operator<(const NiPoint2& a_rhs) const noexcept { return x < a_rhs.x && y < a_rhs.y; }
    bool NiPoint2::operator>(const NiPoint2& a_rhs) const noexcept { return x > a_rhs.x && y > a_rhs.y; }
    NiPoint2 NiPoint2::operator+(const NiPoint2& a_rhs) const noexcept { return { x + a_rhs.x, y + a_rhs.y }; }
    NiPoint2 NiPoint2::operator-(const NiPoint2& a_rhs) const noexcept { return { x - a_rhs.x, y - a_rhs.y }; }
    NiPoint2 NiPoint2::operator*(const NiPoint2& a_rhs) const noexcept { return { x * a_rhs.x, y * a_rhs.y }; }
    NiPoint2 NiPoint2::operator/(const NiPoint2& a_rhs) const noexcept { return { x / a_rhs.x, y / a_rhs.y }; }
    NiPoint2 NiPoint2::operator*(float a_scalar) const noexcept { return { x * a_scalar, y * a_scalar }; }
    NiPoint2 NiPoint2::operator/(float a_scalar) const noexcept { return { x / a_scalar, y / a_scalar }; }
    NiPoint2 NiPoint2::operator-() const noexcept { return { -x, -y }; }
    NiPoint2& NiPoint2::operator+=(const NiPoint2& a_rhs) noexcept { x += a_rhs.x; y += a_rhs.y; return *this; }
    NiPoint2& NiPoint2::operator-=(const NiPoint2& a_rhs) noexcept { x -= a_rhs.x; y -= a_rhs.y; return *this; }
    NiPoint2& NiPoint2::operator*=(const NiPoint2& a_rhs) noexcept { x *= a_rhs.x; y *= a_rhs.y; return *this; }
    NiPoint2& NiPoint2::operator/=(const NiPoint2& a_rhs) noexcept { x /= a_rhs.x; y /= a_rhs.y; return *this; }
    NiPoint2& NiPoint2::operator+=(float a_scalar) noexcept { x += a_scalar; y += a_scalar; return *this; }
    NiPoint2& NiPoint2::operator-=(float a_scalar) noexcept { x -= a_scalar; y -= a_scalar; return *this; }
    NiPoint2& NiPoint2::operator*=(float a_scalar) noexcept { x *= a_scalar; y *= a_scalar; return *this; }
    NiPoint2& NiPoint2::operator/=(float a_scalar) noexcept { x /= a_scalar; y /= a_scalar; return *this; }

    NiPoint3::NiPoint3(const NiPoint2& a_point) noexcept :
        x(a_point.x), y(a_point.y), z(0.0f)
    {}

    NiPoint3::NiPoint3(float a_x, float a_y, float a_z) noexcept :
        x(a_x), y(a_y), z(a_z)
    {}

    float& NiPoint3::operator[](std::size_t a_pos) noexcept { return (&x)[a_pos]; }
    const float& NiPoint3::operator[](std::size_t a_pos) const noexcept { return (&x)[a_pos]; }
    bool NiPoint3::operator==(const NiPoint3& a_rhs) const noexcept { return x == a_rhs.x && y == a_rhs.y && z == a_rhs.z; }
    bool NiPoint3::operator!=(const NiPoint3& a_rhs) const noexcept { return !(*this == a_rhs); }
    bool NiPoint3::operator<(const NiPoint3& a_rhs) const noexcept { return x < a_rhs.x && y < a_rhs.y && z < a_rhs.z; }
    bool NiPoint3::operator>(const NiPoint3& a_rhs) const noexcept { return x > a_rhs.x && y > a_rhs.y && z > a_rhs.z; }
    NiPoint3 NiPoint3::operator+(const NiPoint3& a_rhs) const noexcept { return { x + a_rhs.x, y + a_rhs.y, z + a_rhs.z }; }
    NiPoint3 NiPoint3::operator-(const NiPoint3& a_rhs) const noexcept { return { x - a_rhs.x, y - a_rhs.y, z - a_rhs.z }; }
    NiPoint3 NiPoint3::operator*(const NiPoint3& a_rhs) const noexcept { return { x * a_rhs.x, y * a_rhs.y, z * a_rhs.z }; }
    NiPoint3 NiPoint3::operator/(const NiPoint3& a_rhs) const noexcept { return { x / a_rhs.x, y / a_rhs.y, z / a_rhs.z }; }
    NiPoint3 NiPoint3::operator*(float a_scalar) const noexcept { return { x * a_scalar, y * a_scalar, z * a_scalar }; }
    NiPoint3 NiPoint3::operator/(float a_scalar) const noexcept { return { x / a_scalar, y / a_scalar, z / a_scalar }; }
    NiPoint3 NiPoint3::operator-() const noexcept { return { -x, -y, -z }; }
    NiPoint3& NiPoint3::operator+=(const NiPoint3& a_rhs) noexcept { x += a_rhs.x; y += a_rhs.y; z += a_rhs.z; return *this; }
    NiPoint3& NiPoint3::operator-=(const NiPoint3& a_rhs) noexcept { x -= a_rhs.x; y -= a_rhs.y; z -= a_rhs.z; return *this; }
    NiPoint3& NiPoint3::operator*=(const NiPoint3& a_rhs) noexcept { x *= a_rhs.x; y *= a_rhs.y; z *= a_rhs.z; return *this; }
    NiPoint3& NiPoint3::operator/=(const NiPoint3& a_rhs) noexcept { x /= a_rhs.x; y /= a_rhs.y; z /= a_rhs.z; return *this; }
    NiPoint3& NiPoint3::operator+=(float a_scalar) noexcept { x += a_scalar; y += a_scalar; z += a_scalar; return *this; }
    NiPoint3& NiPoint3::operator-=(float a_scalar) noexcept { x -= a_scalar; y -= a_scalar; z -= a_scalar; return *this; }
    NiPoint3& NiPoint3::operator*=(float a_scalar) noexcept { x *= a_scalar; y *= a_scalar; z *= a_scalar; return *this; }
    NiPoint3& NiPoint3::operator/=(float a_scalar) noexcept { x /= a_scalar; y /= a_scalar; z /= a_scalar; return *this; }
    NiPoint3 NiPoint3::Cross(const NiPoint3& a_point) const noexcept { return { y * a_point.z - z * a_point.y, z * a_point.x - x * a_point.z, x * a_point.y - y * a_point.x }; }
    float NiPoint3::Dot(const NiPoint3& a_point) const noexcept { return x * a_point.x + y * a_point.y + z * a_point.z; }
    float NiPoint3::GetDistance(const NiPoint3& a_point) const noexcept { return (*this - a_point).Length(); }
    float NiPoint3::GetSquaredDistance(const NiPoint3& a_point) const noexcept { return (*this - a_point).SqrLength(); }
    float NiPoint3::GetZAngleFromVector() const { return std::atan2(y, x); }
    float NiPoint3::Length() const noexcept { return std::sqrt(SqrLength()); }
    float NiPoint3::SqrLength() const noexcept { return Dot(*this); }
    NiPoint3 NiPoint3::UnitCross(const NiPoint3& a_point) const noexcept
    {
        auto result = Cross(a_point);
        result.Unitize();
        return result;
    }
    float NiPoint3::Unitize() noexcept
    {
        const float length = Length();
        if (length > 0.000001f) {
            *this /= length;
        }
        return length;
    }

    NiPoint4::NiPoint4(const NiPoint2& a_point) noexcept :
        x(a_point.x), y(a_point.y), z(0.0f), w(0.0f)
    {}

    NiPoint4::NiPoint4(const NiPoint3& a_point) noexcept :
        x(a_point.x), y(a_point.y), z(a_point.z), w(0.0f)
    {}

    NiPoint4::NiPoint4(float a_x, float a_y, float a_z, float a_w) noexcept :
        x(a_x), y(a_y), z(a_z), w(a_w)
    {}

    float& NiPoint4::operator[](std::size_t a_pos) noexcept { return (&x)[a_pos]; }
    const float& NiPoint4::operator[](std::size_t a_pos) const noexcept { return (&x)[a_pos]; }
    bool NiPoint4::operator==(const NiPoint4& a_rhs) const noexcept { return x == a_rhs.x && y == a_rhs.y && z == a_rhs.z && w == a_rhs.w; }
    bool NiPoint4::operator!=(const NiPoint4& a_rhs) const noexcept { return !(*this == a_rhs); }
    bool NiPoint4::operator<(const NiPoint4& a_rhs) const noexcept { return x < a_rhs.x && y < a_rhs.y && z < a_rhs.z && w < a_rhs.w; }
    bool NiPoint4::operator>(const NiPoint4& a_rhs) const noexcept { return x > a_rhs.x && y > a_rhs.y && z > a_rhs.z && w > a_rhs.w; }
    NiPoint4 NiPoint4::operator+(const NiPoint4& a_rhs) const noexcept { return { x + a_rhs.x, y + a_rhs.y, z + a_rhs.z, w + a_rhs.w }; }
    NiPoint4 NiPoint4::operator-(const NiPoint4& a_rhs) const noexcept { return { x - a_rhs.x, y - a_rhs.y, z - a_rhs.z, w - a_rhs.w }; }
    NiPoint4 NiPoint4::operator*(const NiPoint4& a_rhs) const noexcept { return { x * a_rhs.x, y * a_rhs.y, z * a_rhs.z, w * a_rhs.w }; }
    NiPoint4 NiPoint4::operator/(const NiPoint4& a_rhs) const noexcept { return { x / a_rhs.x, y / a_rhs.y, z / a_rhs.z, w / a_rhs.w }; }
    NiPoint4 NiPoint4::operator*(float a_scalar) const noexcept { return { x * a_scalar, y * a_scalar, z * a_scalar, w * a_scalar }; }
    NiPoint4 NiPoint4::operator/(float a_scalar) const noexcept { return { x / a_scalar, y / a_scalar, z / a_scalar, w / a_scalar }; }
    NiPoint4 NiPoint4::operator-() const noexcept { return { -x, -y, -z, -w }; }
    NiPoint4& NiPoint4::operator+=(const NiPoint4& a_rhs) noexcept { x += a_rhs.x; y += a_rhs.y; z += a_rhs.z; w += a_rhs.w; return *this; }
    NiPoint4& NiPoint4::operator-=(const NiPoint4& a_rhs) noexcept { x -= a_rhs.x; y -= a_rhs.y; z -= a_rhs.z; w -= a_rhs.w; return *this; }
    NiPoint4& NiPoint4::operator*=(const NiPoint4& a_rhs) noexcept { x *= a_rhs.x; y *= a_rhs.y; z *= a_rhs.z; w *= a_rhs.w; return *this; }
    NiPoint4& NiPoint4::operator/=(const NiPoint4& a_rhs) noexcept { x /= a_rhs.x; y /= a_rhs.y; z /= a_rhs.z; w /= a_rhs.w; return *this; }
    NiPoint4& NiPoint4::operator+=(float a_scalar) noexcept { x += a_scalar; y += a_scalar; z += a_scalar; w += a_scalar; return *this; }
    NiPoint4& NiPoint4::operator-=(float a_scalar) noexcept { x -= a_scalar; y -= a_scalar; z -= a_scalar; w -= a_scalar; return *this; }
    NiPoint4& NiPoint4::operator*=(float a_scalar) noexcept { x *= a_scalar; y *= a_scalar; z *= a_scalar; w *= a_scalar; return *this; }
    NiPoint4& NiPoint4::operator/=(float a_scalar) noexcept { x /= a_scalar; y /= a_scalar; z /= a_scalar; w /= a_scalar; return *this; }

    NiMatrix3::NiMatrix3(const NiPoint4& a_point0, const NiPoint4& a_point1, const NiPoint4& a_point2) noexcept :
        entry{ a_point0, a_point1, a_point2 }
    {}

    NiMatrix3::NiMatrix3(
        float a_x0, float a_y0, float a_z0, float a_w0,
        float a_x1, float a_y1, float a_z1, float a_w1,
        float a_x2, float a_y2, float a_z2, float a_w2) noexcept :
        entry{ NiPoint4(a_x0, a_y0, a_z0, a_w0), NiPoint4(a_x1, a_y1, a_z1, a_w1), NiPoint4(a_x2, a_y2, a_z2, a_w2) }
    {}

    NiPoint4& NiMatrix3::operator[](std::size_t a_pos) noexcept { return entry[a_pos]; }
    const NiPoint4& NiMatrix3::operator[](std::size_t a_pos) const noexcept { return entry[a_pos]; }
    bool NiMatrix3::operator==(const NiMatrix3& a_rhs) const noexcept { return entry[0] == a_rhs.entry[0] && entry[1] == a_rhs.entry[1] && entry[2] == a_rhs.entry[2]; }
    bool NiMatrix3::operator!=(const NiMatrix3& a_rhs) const noexcept { return !(*this == a_rhs); }
    NiMatrix3 NiMatrix3::operator*(float a_scalar) const noexcept { return { entry[0] * a_scalar, entry[1] * a_scalar, entry[2] * a_scalar }; }
    NiPoint3 NiMatrix3::operator*(const NiPoint3& a_rhs) const noexcept
    {
        return {
            entry[0].x * a_rhs.x + entry[0].y * a_rhs.y + entry[0].z * a_rhs.z,
            entry[1].x * a_rhs.x + entry[1].y * a_rhs.y + entry[1].z * a_rhs.z,
            entry[2].x * a_rhs.x + entry[2].y * a_rhs.y + entry[2].z * a_rhs.z
        };
    }
    NiMatrix3 NiMatrix3::operator*(const NiMatrix3& a_rhs) const noexcept
    {
        NiMatrix3 result{};
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 3; ++col) {
                result.entry[row][col] = entry[row].x * a_rhs.entry[0][col] + entry[row].y * a_rhs.entry[1][col] + entry[row].z * a_rhs.entry[2][col];
            }
        }
        return result;
    }
    void NiMatrix3::MakeIdentity() noexcept { *this = IDENTITY; }
    NiMatrix3 NiMatrix3::Transpose() const noexcept
    {
        return {
            entry[0].x, entry[1].x, entry[2].x, 0.0f,
            entry[0].y, entry[1].y, entry[2].y, 0.0f,
            entry[0].z, entry[1].z, entry[2].z, 0.0f
        };
    }
}

namespace REL
{
    /*
     * Policy test executables validate pure ROCK math and state contracts. They
     * intentionally do not run inside Fallout 4 VR, so CommonLib relocation
     * singletons must not scan process modules or load the VR address library.
     * Production code still uses the real CommonLib definitions through the DLL
     * target; these inert definitions are linked only into standalone tests.
     */
    Module::Module() :
        _filename(L"Fallout4VR.exe"),
        _filePath(L"Fallout4VR.exe"),
        _runtime(Runtime::VR)
    {}

    IDDB::IDDB() = default;

    std::size_t IDDB::id2offset(std::uint64_t) const
    {
        return 0;
    }

#ifdef ENABLE_FALLOUT_VR
    bool IDDB::IsVRAddressLibraryAtLeastVersion(const char*, bool) const
    {
        return true;
    }
#endif
}

namespace rock
{
    /*
     * Policy executables exercise header-only math and state decisions without
     * loading the runtime INI watcher. Refactored grouped headers can still
     * instantiate the inline global config for default values, so tests provide
     * only the file-watch shutdown hook needed by RockConfig's destructor.
     */
    void RockConfig::stopFileWatch() {}
}
