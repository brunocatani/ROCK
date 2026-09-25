# Changelog

## v0.92.2

### Fixed

- Prevented repeated dynamic-hand resets while walking with one hand pressed
  against the other hand or an equipped weapon. Self-contact limits now account
  for shared player movement; world contact retains its stationary safety limit.
- Added sustained surface-contact observations for held loose objects, so a
  bottle resting against a surface can qualify for placement without an impact.
- Validated placement against a fresh, bounded body scan rather than historical
  grab-cache completeness. Valid static parts, such as the Timberwolf's fixed
  component, remain static while movable parts are anchored. Invalid collision
  layers and incomplete body sets remain rejected with diagnostic details.

### Added

- Grab API 1.1 exposes held-object/contact observations, one-frame placement
  requests, cancellation and completion results. Requests expire on lost renewal
  and are cancelled when their owner or required runtime state disappears.
- Input API 1.1 exposes placement-click reservation through the complete press
  and release, allowing input consumers to avoid reusing that click.
- Native placement coordinates release of grab constraints, keyframing and the
  game's position, rotation and Havok save state. Ordinary grab and pull-to-grab
  retain their existing unlock behavior; no decoration co-save registry is added.

### Feature ownership and compatibility

- Decoration activation and gesture handling belong to the optional **ROCK Pimp
  my House** add-on. ROCK has no built-in decoration mode or decoration INI key.
- Released Grab 1.0 and Input 1.0 contracts, table prefixes and Core requirements
  remain unchanged. Consumers explicitly negotiate the new minor versions.
- Builds require [RPS SDK df02e89 or newer](https://github.com/brunocatani/RPS_SDK/commit/df02e89),
  containing `ROCK/GrabV1_1.h` and `ROCK/InputV1_1.h`.

### Validation

- Release builds passed for ROCK, ROCK Pimp my House, RobCo PALM and the SDK
  examples. Fourteen targeted tests passed across those projects during
  implementation, including ABI/negotiation, placement policy and input handling.
- The moving self-contact fix and native anchoring were tested in game before
  extraction. The extracted add-on's first load and save/reload/cell-return
  persistence still require a correlated runtime test.
