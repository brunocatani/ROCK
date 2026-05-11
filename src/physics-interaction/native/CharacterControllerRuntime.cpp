#include "physics-interaction/native/CharacterControllerRuntime.h"

#include "RE/Bethesda/PlayerCharacter.h"

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
}
