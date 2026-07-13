#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <cmath>
#include <cstdint>

#include <windows.h>

namespace rock::character_controller_runtime
{
    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept
    {
        RE::bhkCharacterController* controller = nullptr;

        __try {
            if (actor && actor->currentProcess && actor->currentProcess->middleHigh) {
                controller = actor->currentProcess->middleHigh->charController.get();
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            controller = nullptr;
        }

        return controller;
    }

    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept
    {
        return tryGetActorCharacterController(RE::PlayerCharacter::GetSingleton());
    }

    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept
    {
        outVelocityGameUnits = RE::NiPoint3{};
        bool ok = false;

        __try {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                // Verified FO4VR offsets only -- do not substitute CommonLibF4VR struct members here.
                const auto* playerBytes = reinterpret_cast<const std::uint8_t*>(player);
                const void* currentProcess = *reinterpret_cast<void* const*>(playerBytes + 0x300);  // Actor::currentProcess
                if (currentProcess) {
                    const auto* processBytes = reinterpret_cast<const std::uint8_t*>(currentProcess);
                    const void* middleHigh = *reinterpret_cast<void* const*>(processBytes + 0x08);  // AIProcess::middleHigh
                    if (middleHigh) {
                        const auto* middleHighBytes = reinterpret_cast<const std::uint8_t*>(middleHigh);
                        const void* charController = *reinterpret_cast<void* const*>(middleHighBytes + 0x3E8);  // VR-verified (CommonLib 0x3E0 is wrong)
                        if (charController) {
                            const auto* ccBytes = reinterpret_cast<const std::uint8_t*>(charController);
                            const float vx = *reinterpret_cast<const float*>(ccBytes + 0x250);  // cachedLinearVelocity.x (game units)
                            const float vy = *reinterpret_cast<const float*>(ccBytes + 0x254);
                            const float vz = *reinterpret_cast<const float*>(ccBytes + 0x258);
                            if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
                                outVelocityGameUnits.x = vx;
                                outVelocityGameUnits.y = vy;
                                outVelocityGameUnits.z = vz;
                                ok = true;
                            }
                        }
                    }
                }
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            ok = false;
        }

        return ok;
    }
}
