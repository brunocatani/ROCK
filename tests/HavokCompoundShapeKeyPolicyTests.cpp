#include "physics-interaction/native/HavokCompoundShapeBuilder.h"

#include <cassert>
#include <cstdint>

int main()
{
    using namespace rock::havok_compound_shape_builder;

    static_assert(compoundShapeKeyBitCount(0) == 1);
    static_assert(compoundShapeKeyBitCount(1) == 1);
    static_assert(compoundShapeKeyBitCount(2) == 2);
    static_assert(compoundShapeKeyBitCount(16) == 5);
    static_assert(compoundShapeKeyBitCount(17) == 5);

    constexpr std::uint8_t handCompoundBits =
        compoundShapeKeyBitCount(17);
    for (std::uint32_t child = 0; child < 17; ++child) {
        const std::uint32_t nestedLeafBits = 0x05A5'A5A5u &
            ((1u << (32u - handCompoundBits)) - 1u);
        const std::uint32_t shapeKey =
            (child << (32u - handCompoundBits)) | nestedLeafBits;
        const auto decoded = decodeTopLevelCompoundInstanceId(
            shapeKey,
            handCompoundBits);
        assert(decoded.has_value());
        assert(*decoded == child);
    }

    assert(!decodeTopLevelCompoundInstanceId(
        0xFFFF'FFFFu,
        handCompoundBits));
    assert(!decodeTopLevelCompoundInstanceId(0u, 0u));
    assert(!decodeTopLevelCompoundInstanceId(0u, 32u));
    return 0;
}
