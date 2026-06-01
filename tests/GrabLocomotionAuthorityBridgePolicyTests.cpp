#include "physics-interaction/grab/GrabLocomotionAuthorityBridge.h"

#include <cmath>
#include <cstdio>

namespace
{
    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::grab_locomotion_authority_bridge;

    bool ok = true;

    {
        State state{};
        const Config config{};

        const auto invalidPlayerSpace = update(state,
            Input{
                .config = config,
                .playerSpaceValid = false,
                .playerMoving = true,
                .heldObjectActive = true,
                .playerDeltaGameUnits = Vec3{ 1.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });
        ok &= expectTrue("invalid player space resets bridge", invalidPlayerSpace.reset);
        ok &= expectNear("invalid player space has no offset", invalidPlayerSpace.offsetGameUnits.x, 0.0f, 0.0001f);

        const auto noHeldObject = update(state,
            Input{
                .config = config,
                .playerSpaceValid = true,
                .playerMoving = true,
                .heldObjectActive = false,
                .playerDeltaGameUnits = Vec3{ 1.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });
        ok &= expectTrue("no held object resets bridge", noHeldObject.reset);
        ok &= expectNear("no held object has no offset", noHeldObject.offsetGameUnits.x, 0.0f, 0.0001f);

        const auto notMoving = update(state,
            Input{
                .config = config,
                .playerSpaceValid = true,
                .playerMoving = false,
                .heldObjectActive = true,
                .playerDeltaGameUnits = Vec3{ 0.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });
        ok &= expectFalse("not moving is not active", notMoving.active);
        ok &= expectNear("not moving has no offset", notMoving.offsetGameUnits.x, 0.0f, 0.0001f);
    }

    {
        State state{};
        const auto steady = update(state,
            Input{
                .config = Config{},
                .playerSpaceValid = true,
                .playerMoving = true,
                .heldObjectActive = true,
                .playerPositionGame = Vec3{ 1.0f, 0.0f, 0.0f },
                .playerDeltaGameUnits = Vec3{ 1.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });

        ok &= expectTrue("steady stick locomotion activates bridge", steady.active);
        ok &= expectNear("steady stick locomotion estimates root velocity", steady.velocityGameUnitsPerSecond.x, 90.0f, 0.001f);
        ok &= expectNear("steady stick locomotion predicts bounded lead", steady.offsetGameUnits.x, 1.08f, 0.001f);
        ok &= expectNear("steady stick locomotion only translates root axis", steady.offsetGameUnits.y, 0.0f, 0.001f);
    }

    {
        State state{};
        const auto capped = update(state,
            Input{
                .config = Config{},
                .playerSpaceValid = true,
                .playerMoving = true,
                .heldObjectActive = true,
                .playerPositionGame = Vec3{ 10.0f, 0.0f, 0.0f },
                .playerDeltaGameUnits = Vec3{ 10.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });

        ok &= expectTrue("fast stick locomotion still activates bridge", capped.active);
        ok &= expectNear("fast stick locomotion offset is capped", capped.offsetGameUnits.x, 4.0f, 0.001f);
    }

    {
        State state{};
        (void)update(state,
            Input{
                .config = Config{},
                .playerSpaceValid = true,
                .playerMoving = true,
                .heldObjectActive = true,
                .playerPositionGame = Vec3{ 1.0f, 0.0f, 0.0f },
                .playerDeltaGameUnits = Vec3{ 1.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });

        const auto rootJump = update(state,
            Input{
                .config = Config{},
                .playerSpaceValid = true,
                .playerMoving = true,
                .heldObjectActive = true,
                .playerPositionGame = Vec3{ 40.0f, 0.0f, 0.0f },
                .playerDeltaGameUnits = Vec3{ 36.0f, 0.0f, 0.0f },
                .deltaSeconds = 1.0f / 90.0f,
            });

        ok &= expectTrue("large root jump resets bridge", rootJump.reset);
        ok &= expectFalse("large root jump suppresses bridge activity", rootJump.active);
        ok &= expectNear("large root jump has no offset", rootJump.offsetGameUnits.x, 0.0f, 0.0001f);
    }

    return ok ? 0 : 1;
}
