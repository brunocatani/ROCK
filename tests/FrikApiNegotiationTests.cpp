#include <format>
#include "api/FRIKApiV2.h"
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{
    std::uint32_t advertisedVersion{};
    std::uint32_t version() { return advertisedVersion; }
}

int main()
{
    using Api = frik::api::FRIKApiV2;
    // Published byte sizes on Windows x64. A new client must not require its
    // full current table to use entries that existed in a released prefix.
    static_assert(Api::tableSizeForVersion(1) == 248);
    static_assert(Api::tableSizeForVersion(2) == 296);
    static_assert(Api::tableSizeForVersion(3) == 368);
    static_assert(Api::tableSizeForVersion(4) == 368);
    static_assert(offsetof(Api, blockSecondaryWeaponNodeOwnership) == 368);
    static_assert(sizeof(Api) == 376);
    Api provider{};
    provider.getVersion = &version;
    assert(!Api::supportsVersion(3));
    Api::inst = &provider;
    for (advertisedVersion = 1; advertisedVersion <= 6; ++advertisedVersion) {
        for (std::uint32_t size : {0u,247u,248u,295u,296u,367u,368u,375u,376u,384u}) {
            Api::negotiatedTableSize = size;
            assert(Api::supportsVersion(1) == (size >= 248));
            assert(Api::supportsVersion(2) == (advertisedVersion >= 2 && size >= 296));
            assert(Api::supportsVersion(3) == (advertisedVersion >= 3 && size >= 368));
            assert(Api::supportsVersion(4) == (advertisedVersion >= 4 && size >= 368));
            assert(Api::supportsVersion(5) == (advertisedVersion >= 5 && size >= 376));
            assert(!Api::supportsVersion(0) && !Api::supportsVersion(6));
        }
    }
    Api::inst = nullptr;
    Api::negotiatedTableSize = 0;
}
