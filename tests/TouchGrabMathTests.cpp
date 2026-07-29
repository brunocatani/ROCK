#include "physics-interaction/grab/TouchGrabMath.h"

#include <cmath>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

int main()
{
    using namespace rock::touch_grab_math;
    constexpr float halfPi = 1.57079632679f;

    Vector3 witness{};
    assert(makePerpendicularWitness({ 0.0f, 0.0f, 1.0f }, witness));
    assert(std::abs(dot(witness, { 0.0f, 0.0f, 1.0f })) <
           1.0e-6f);

    const float positive = hingeCoordinate(
        0.25f,
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f });
    assert(std::abs(positive - (0.25f + halfPi)) < 1.0e-5f);

    const float negative = hingeCoordinate(
        0.25f,
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f });
    assert(std::abs(negative - (0.25f - halfPi)) < 1.0e-5f);

    const float slider = prismaticCoordinate(
        2.0f,
        { 5.0f, 6.0f, 7.0f },
        { 5.0f, 1.0f, 9.0f },
        { 0.0f, -1.0f, 0.0f });
    assert(std::abs(slider - 7.0f) < 1.0e-6f);
    return 0;
}
