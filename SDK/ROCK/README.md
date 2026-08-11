# ROCK SDK

ROCK SDK V1 is the public in-process C++ ABI for FO4VR F4SE plugins that need coherent access to ROCK's hand, collision, weapon, input, animation, interaction, and diagnostic systems.

The current append-only V1 table contains 90 function slots. It supports read-only observation, owner-scoped registration, queued interaction commands, bounded control authorities, scoped publications, semantic contact streams, touch mechanisms, raycasts, and shared VR diagnostics without exposing private ROCK runtime classes.

## Start here

1. Read `docs/GettingStarted.md` and integrate `include/ROCKProviderApi.h`.
2. Read `docs/RuntimeContract.md` before calling any frame-sensitive or stateful surface.
3. Choose only the capabilities your mod needs from `docs/DiscoveryAndCapabilities.md`.
4. Use `docs/ApiIndex.md` for the exact 90-slot map and `docs/FeatureGuide.md` for task-oriented guidance.
5. Build or adapt one of the generic plugins under `examples/`.

## Package map

| Path | Contents |
| --- | --- |
| `include/ROCKProviderApi.h` | Canonical public V1 declarations, values, table guards, discovery helper, and support helpers. |
| `include/ROCKApi.h` | Alias header for the same provider table/version. |
| `docs/PublicApi.md` | Architectural overview and API-family map. |
| `docs/GettingStarted.md` | Initialization, result codes, registration, callbacks, and teardown. |
| `docs/RuntimeContract.md` | Threading, lifecycle, generation, pointer, lease, cursor, and command rules. |
| `docs/DiscoveryAndCapabilities.md` | Descriptor, table extent, 27 capabilities, 62 feature bits, and limits. |
| `docs/ApiIndex.md` | Every V1 function pointer in ABI slot order. |
| `docs/FeatureGuide.md` | What users can build with each subsystem. |
| `docs/Recipes.md` | Focused call patterns for common integrations. |
| `docs/VersionMatrix.md` | API/provider compatibility policy. |
| `examples/` | Four complete buildable F4SE DLLs plus a compact integration fragment. |

## ABI contract

The boundary is POD/value-oriented and C-exported from `ROCK.dll`. Consumers include only SDK headers, discover the table dynamically, and never link or reach into ROCK's private implementation.

V1 evolves by appending function pointers and extending structures through sized prefixes. `ROCK_PROVIDER_API_VERSION` therefore remains `1` while the table grows. Compatibility requires API version, table byte extent, feature support, and owner capability grant—not the version number alone.

The authoritative signatures and enum values are in `ROCKProviderApi.h`. Documentation and examples are mechanically checked against that header, and example plugins are compiled in ROCK's test configuration.

## Ownership in one paragraph

Register with `registerConsumerV1`, retain the returned owner token for the current registration lifetime, and use it for every stateful call. Refresh leases only while active, clear feature state when it stops, unregister callbacks, then unregister the consumer. ROCK revokes remaining owner resources on unregister/provider loss, but deterministic cleanup is still the consumer's responsibility.
