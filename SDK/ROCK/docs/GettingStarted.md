# Getting started

ROCK SDK V1 is an in-process C++ ABI for FO4VR F4SE plugins. Consumers dynamically discover `ROCK.dll`; they do not link against a ROCK import library and must not include private ROCK source.

## 1. Add the header

Copy or include `SDK/ROCK/include/ROCKProviderApi.h`. `ROCKApi.h` is a compatibility alias for the same V1 table.

Your plugin still links against its own verified FO4VR/CommonLibF4VR stack. ROCK is discovered with `GetModuleHandleA` and `GetProcAddress` inside the SDK header.

## 2. Initialize after ROCK has loaded

Request both V1 and the byte extent containing the newest function your mod will call:

```cpp
using namespace rock::provider;

const int result = RockProviderApi::initialize(
    ROCK_PROVIDER_API_VERSION,
    ROCK_PROVIDER_API_V1_TOUCH_GRAB_TARGETS_TABLE_BYTES);
if (result != 0 || !RockProviderApi::inst) {
    return false;
}
```

Initialization result codes are stable:

| Code | Meaning |
| --- | --- |
| 0 | Initialized, requested API version and table extent are available. |
| 1 | `ROCK.dll` is not loaded. |
| 2 | The legacy `ROCKAPI_GetProviderApi` export is unavailable. |
| 3 | The legacy export returned no table. |
| 4 | The provider API version is older than requested. |
| 5 | The negotiated V1 table is shorter than `minProviderApiByteSize`. |
| 6 | The safe descriptor is malformed, or a table extent was requested from a provider without descriptor support. |

Do not retry in a per-frame loop. Connect on an F4SE game-data/session event, and reconnect only after a meaningful lifecycle transition.

## 3. Register a consumer

```cpp
RockProviderConsumerRegistrationV1 registration{};
std::snprintf(
    registration.modName,
    sizeof(registration.modName),
    "%s",
    "MyPlugin");
registration.requestedCapabilities =
    static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::FrameSnapshots) |
    static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ProviderEvents);

RockProviderConsumerHandleV1 handle{};
const auto registerResult =
    RockProviderApi::inst->registerConsumerV1(&registration, &handle);
if (registerResult != RockProviderResultV1::Ok ||
    handle.ownerToken == 0 ||
    (handle.grantedCapabilities & registration.requestedCapabilities) !=
        registration.requestedCapabilities) {
    return false;
}
```

The returned owner token is the identity for all stateful calls. Never invent, share, persist, or reuse a token across registrations.

## 4. Register an owner callback

Frame-sensitive reads and all stateful writes belong inside ROCK's owner callback:

```cpp
void ROCK_PROVIDER_CALL onRockFrame(
    const RockProviderFrameSnapshot* snapshot,
    void*)
{
    if (!snapshot ||
        !hasLifecycleFlag(
            snapshot->lifecycleFlags,
            RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
        return;
    }

    // Read snapshots or refresh bounded publications here.
}

std::uint64_t callbackToken{};
const auto callbackResult =
    RockProviderApi::inst->registerFrameCallbackForOwnerV1(
        handle.ownerToken,
        &onRockFrame,
        nullptr,
        &callbackToken);
```

## 5. Teardown deterministically

```cpp
(void)RockProviderApi::inst->unregisterFrameCallbackForOwnerV1(
    handle.ownerToken,
    callbackToken);
(void)RockProviderApi::inst->unregisterConsumerV1(handle.ownerToken);
```

Unregistering revokes the owner's callbacks, publications, registrations, reservations, queued commands, and other stateful resources. Still clear long-lived surfaces explicitly when your feature turns off; teardown is a final safety net, not ordinary control flow.

## Build complete examples

See `examples/README.md`. The example CMake project produces four independent F4SE DLLs and enforces the FO4VR Query/Load contract at configure time.
