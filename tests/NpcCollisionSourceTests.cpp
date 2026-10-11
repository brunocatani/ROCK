#include "physics-interaction/actor/NpcCollisionSource.h"

#include <cstdio>

namespace
{
    bool expect(bool condition, const char* message)
    {
        if (!condition) std::printf("%s\n", message);
        return condition;
    }
}

int main()
{
    using namespace rock::npc_collision;
    // 2026-09-21 20:57:11, actor 00002F8F, body 5572: readable and
    // animated, but absent from physical collision. Pointer values below are
    // opaque identity tokens; filter/flags/presence/body/motion are captured.
    SourceState missing{
        .identity = {5572, 2787, 0x10000, 0x20000, 0x30000, 0x051E},
        .filter = 0x051E0208,
        .flags = 0x00011486,
        .presence = -1,
    };
    const auto expected = missing.identity;
    bool ok = expect(missing.canDrive(expected), "captured inactive animated limb must be eligible");

    // Native re-insertion must immediately revoke supplemental contact even
    // when every identity field and the animated pose are unchanged.
    auto live = missing;
    live.presence = 1073741840;
    ok &= expect(!live.canDrive(expected), "native collision return must revoke the duplicate");
    live = missing;
    live.flags |= 0x40000;
    ok &= expect(!live.canDrive(expected), "native pending insertion must also revoke the duplicate");
    live = missing;
    live.flags &= ~4u;
    ok &= expect(!live.canDrive(expected), "ragdoll motion ownership must revoke supplemental driving");

    // Captured pose-bound body 5571 exists separately from the limb bodies.
    auto poseBound = missing;
    poseBound.identity.bodyId = 5571;
    poseBound.identity.motionId = 2786;
    poseBound.filter = 0x051E0008;
    poseBound.flags = 0x000C010E;
    poseBound.presence = -1610612735;
    ok &= expect(!poseBound.needsSupplement(), "pose-bound body must not become an anatomical obstacle");

    // Recycled slots cannot inherit a pending collision publication. Each
    // independent identity witness must still match immediately before drive.
    for (int field = 0; field != 6; ++field) {
        auto recycled = missing;
        switch (field) {
        case 0: ++recycled.identity.bodyId; break;
        case 1: ++recycled.identity.motionId; break;
        case 2: ++recycled.identity.collisionObject; break;
        case 3: ++recycled.identity.ownerNode; break;
        case 4: ++recycled.identity.shape; break;
        case 5: ++recycled.identity.group; break;
        }
        ok &= expect(!recycled.canDrive(expected), "a recycled source identity must fail closed");
    }
    auto disabled = missing;
    disabled.filter |= 0x4000;
    ok &= expect(!disabled.canDrive(expected), "explicit native no-collision ownership must be preserved");
    ok &= expect(!SourceState{}.canDrive(expected), "missing source must not publish collision");
    return ok ? 0 : 1;
}
