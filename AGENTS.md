# REPOSITORY DEFINITION

# ROCK — Realistic Overengineered Character Kinetics

This is the project folder for ROCK, a physics-based hand interaction system for Fallout 4 VR. ROCK began as a port of HIGGS (Skyrim VR) concepts, functionality, and interaction logic into FO4VR, but it has evolved into its own FO4VR-native system with FRIK as the body/skeleton provider and several systems that now exceed HIGGS behavior.

HIGGS source location: `E:\fo4dev\skirymvr_mods\source_codes\higgs`.
HIGGS WAS THE ORIGINAL BASELINE, NOT THE CURRENT AUTHORITY. It is historical/reference material for understanding the first-stage port, old interaction categories, constraints, and failure cases.
DO NOT use HIGGS as the default baseline for new ROCK implementation, behavior comparison, or correctness checks unless the user explicitly asks for HIGGS comparison/porting guidance or explicitly approves HIGGS as a base for the current task.
DO NOT force ROCK toward 1:1 HIGGS behavior. ROCK is now more advanced in several areas, especially detailed weapon collisions and richer weapon/body interaction logic that HIGGS does not have.
When HIGGS conflicts with current ROCK architecture, current FO4VR design wins unless the user explicitly says otherwise.

# Main Guidelines

# ABSOLUTE RULE: QUALITY MANDATE — NO EXCEPTIONS
# ALL implementations MUST be proper, high-quality, long-term, production-grade solutions.
# NO quick fixes. NO "just to test." NO "for now" workarounds. NO barely functional code.
# NO "easy wins." NO "simple fix." NO shortcuts of ANY kind. NEVER suggest them.
# NO skipping features because they're complex. Find the correct FO4VR-native architecture,
# use approved local references when they are relevant, and ask when the right approach is uncertain.
# The goal is to build something GOOD — not preschool-level code that needs to be redone.
# Every line of code must be written as if it will be there forever.
# Previous agents introduced bugs and shortcuts that compounded into major issues.
# NEVER repeat that. Map everything first, discuss with the user, THEN implement.
# If unsure about the right approach, ASK before writing a single line of code.
# Quality over speed, ALWAYS. Time is not a constraint — correctness is.
# If you find yourself about to say "quick", "simple", "easy", "for now", or "just" —
# STOP and reconsider. Those words mean you're about to propose something wrong.

# CRITICAL: NEVER TELL THE USER WHAT THE PRIORITY IS
# The user decides what to work on and in what order. NEVER say "current priority is X"
# or "optimize later" or suggest next steps. Do the work, report results, wait for direction.

# CRITICAL: SYSTEM-LEVEL THINKING
# Features and fixes are INTERCONNECTED. NEVER propose or implement isolated fixes.
# ALWAYS think about the WHOLE system: what depends on this, what this depends on, what
# will need to change when the next feature is added. Design as a COMPLETE COHERENT SYSTEM.

# CRITICAL: FO4VR-NATIVE REFERENCE DISCIPLINE
# For EVERY modification, BEFORE writing any code:
# 1. Map the current ROCK architecture, local docs, tests, and existing subsystem behavior first.
# 2. Use HIGGS at `E:\fo4dev\skirymvr_mods\source_codes\higgs` only when the user explicitly asks
#    for HIGGS guidance or explicitly approves HIGGS as the current base.
# 3. Cross-check reverse-engineering, layout, offset, and runtime behavior claims against the FO4VR binary
#    via Ghidra MCP only after explaining why and receiving user approval.
# 4. DO NOT implement from memory, assumptions, or "general knowledge"
# 5. If the current architecture and approved references are insufficient, ASK the user before writing code.

# CRITICAL: GHIDRA USAGE PROTOCOL
# Ghidra MCP is a shared resource. Before EVERY Ghidra operation:
# 1. EXPLAIN to the user WHY you need to use Ghidra
# 2. WAIT for the user to approve

# CRITICAL: BLIND VERIFICATION
# Use /blind-verification-audit for ALL reverse engineering claims, struct layouts,
# and offset assumptions. Never trust documentation without binary verification.
# CommonLibF4VR headers may be wrong for VR — always cross-check with Ghidra.

DO NOT DEFAULT TO HIGGS AS THE REFERENCE BASELINE FOR NEW FEATURES. HIGGS is available at `E:\fo4dev\skirymvr_mods\source_codes\higgs`, but use it as a base only when the user explicitly permits it for the current work.
When HIGGS is approved for a task, study it in depth, then document what applies to FO4VR, what does not apply, and why the chosen ROCK approach differs if it differs.
TAKE PERIODIC NOTES USING .MD FILES OF THE FINDINGS.

THINGS OF IMPORTANCE ADDED MANUALLY

HAND BODY IN HIGGS - 56 = OUR HAND COLLIDER ON LAYER 43
WEAPONBODY IN HIGGS  56 = (NOT IMPLEMENTED YET) OUR WEAPON COLLIDER, WHEN WE RE-ENABLE PHYSICS FOR WEAPONS
HELD OBJECT KEYFRAMED IN HIGGS = OUR HELD OBJECT, OBJECT KEEPS IN THE SAME LAYER AS IS
HELD OBJECT DYNAMIC IN HIGGS (CONSTRAINT ADDED) = OUR IMPLEMENTATION OF DYNAMIC HELD OBJECT

Prefer complete, well implemented FO4VR-native approaches. Existing reference mods such as HIGGS, active ragdoll, and others may be useful context when the user allows them, but they are not behavior targets unless explicitly approved for the current task.

* Local builds for fast iteration during development. GitHub Actions for "release" builds only after everything is tested locally.
* All code must be developed with a focus on quality, logging, testing and longevity, trying to match the specification and objectives as close as possible as this project requires complex integrations and logic
* When in doubt, first map the current ROCK implementation and ask the user which reference sources are allowed. Use local source from projects with similar mechanisms only when it is permitted and clearly relevant.
* BETHESDA MAY HAVE STRIPPED MANY FUNCTIONS FROM FALLOUT 4 VR — when necessary for confirmation, use Ghidra to study and map everything. If something is found, save to a relevant skill or reference file. Also check the FO4 flat binary for answers.
* FO4 Mods have almost 11 years of community development — always ask or refer to available source code for implementations and mod building
* Production native layout reads, raw `REL::Offset`/`REL::ID`/`REL::Relocation` use, and direct Havok/Bethesda memory boundary logic belong in `src/physics-interaction/native/` behind named helpers. Debug-only renderer/overlay reads may stay isolated in `src/physics-interaction/debug/` when source-boundary tests explicitly allow them.
* Native/Havok function placement is strict. If code calls or wraps a raw hknp/bhk/Bethesda physics function, reads/writes a native layout offset, builds a native ABI object, registers native data, touches a Havok world/body/motion/material/constraint table, patches a collision filter matrix, or mutates body state through a verified address, the implementation belongs in `src/physics-interaction/native/`. Domain modules (`hand/`, `grab/`, `weapon/`, `collision/`, `contact/`, `core/`) should call a named native helper and keep only ROCK policy, state transitions, target selection, math, telemetry shaping, and lifecycle decisions.
* Do not leave orphan Havok wrappers in domain files. Examples that belong in native helpers include hknp motion-property library reads/writes and leases, `SetBodyMotionProperties`, custom constraint data/vtables/atoms/motors, `CreateConstraint`/`DestroyConstraints`, body/motion raw field reads, material library registration, `bhkNPCollisionObject` impulse/velocity/transform/mass wrappers, `bhkWorld` recursive wrappers, collision filter matrix access, physics-system instance/body-id enumeration, Havok allocator/TLS calls, shape casts, generated body creation/teardown, and step-listener registration. If a domain needs one of these operations, create or extend a purpose-named native file such as `HavokRuntime`, `HavokMotionProperties`, `HavokGrabConstraint`, `BethesdaPhysicsBody`, `PhysicsShapeCast`, `PhysicsRecursiveWrappers`, or another native helper with the same ownership pattern.
* Raw address constants belong in `src/physics-interaction/native/HavokOffsets.h` when they are Havok/Bethesda physics addresses. Non-Havok hook, input, UI, or renderer addresses may stay in their focused runtime/hook/debug files only when they do not own physics memory-boundary behavior and the placement is covered by a source-boundary test or an explicit local rationale.
* Domain-facing native helpers must be named for what ROCK does, not for where the address came from. Prefer APIs such as `acquireBodyMotionDampingLease`, `setBodyTransformDeferred`, `registerGeneratedBodyMaterial`, `createGrabConstraintDrive`, or `forEachPhysicsSystemBodyId` over exposing offsets, raw pointers, or native function signatures to domain code.
* When moving or adding a native boundary rule, add or update a focused `tests/*SourceTests.ps1` guard. Source-boundary tests should reject the raw offset/function use outside `native/`/approved `debug/` locations and require the named helper that domains are expected to call.
* Config authority: the real production INI at `C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini` is the law. Packaged `data/config/ROCK.ini`, embedded resource defaults, compiled defaults, and parity tests are often stale and must not be treated as authoritative unless the user explicitly says they manually updated them. Always update the prod INI in place; never replace it.
* Public API/provider structs are binary contracts. Keep layout/size/static-assert coverage when changing anything in `src/api/ROCKApi.h` or `src/api/ROCKProviderApi.h`.


## Implementations, approaches and plans must always focus a proper implementation of the desired features, no matter how many sessions it takes. Use multiple .md files with plans, specs and approaches as needed to scope, map and study everything necessary.

## Every implementation must begin with a short paragraph explaining WHY it's being done this way — what problem it solves, what alternatives were considered, and why this approach was chosen. This goes as a comment block at the top of the new code or as a note in the plan before work starts. Without this, future sessions have no context for the decisions made.

## Code comments, identifiers, logs, and source-boundary tests must describe ROCK behavior directly. DO NOT explain a function by saying how HIGGS, Skyrim VR, Planck, FRIK, CommonLib, or another project does it unless the current task explicitly requires a comparison. Inline comments should say what ROCK owns, what the function reads/writes, what FO4VR/Havok/native boundary is being protected, and why the chosen ROCK behavior exists. Avoid phrases like "HIGGS-style", "mirrors HIGGS", "ported from", "Skyrim-style", "FRIK-style", or "baseline did X" in code, config comments, test messages, debug overlay labels, enum names, and telemetry names. If external reference history is useful, keep it in explicitly approved `.md` analysis/planning notes, not in production code comments or test assertions.

## HIGGS and Planck (Active Ragdoll) reference documents are useful context when explicitly approved for the current task, but they are not current design authority. FO4VR has its own peculiarities — different Havok version (2014 hknp vs 2012 hkp), stripped functions, different memory layouts, different solver behavior. NEVER blindly copy from HIGGS/Planck documentation or force 1:1 behavior. ALWAYS cross-check approved reference claims against:
## 1. The Havok library at `libraries_and_tools/havok` (confirmed addresses, Ghidra findings)
## 2. Ghidra MCP on the FO4VR binary (reverse engineer actual runtime behavior)
## 3. The FO4 flat binary when VR-specific answers are needed
## When in doubt, verify in Ghidra BEFORE implementing. When something differs from the reference documentation, EXPLAIN WHY — what is different in FO4VR, what the documentation says vs what you found, and what approach you chose and why. Never silently deviate.

## Always look for potential improvements to the logic and novel approaches

## When in doubt ask the user for feedback and guidance

## If new relevant information is found, save it to relevant skills or files — especially new discoveries that improve old code, implementation or logic, as those are a treasure trove



# Build System

- ROCK builds standalone with CMake + VS2022 + vcpkg
- CommonLibF4VR and F4VR-CommonFramework are in `libraries_and_tools/`
- Build command: `cd ROCK && VCPKG_ROOT="C:/vcpkg" cmake --build build --config Release`
- Output: `ROCK/build/Release/ROCK.dll`
- CMakeUserPresets.json points to local library paths
- Focused tests are controlled by `BUILD_ROCK_TESTS`. C++ regression tests and PowerShell `tests/*SourceTests.ps1` source-boundary tests are registered through CTest when the required runner is available.
- Source-boundary tests are part of the contract. When adding a new architecture rule, add or update a focused source test instead of relying on review memory.
- If local Linux tooling lacks CMake, Visual Studio, or PowerShell, still run source scans that prove the touched boundary, then clearly report which build/test commands could not be executed.

# Available External Tools

Must confirm with the user before using:

* Ghidra MCP — reverse engineer .exe and .dll (FO4VR, FO4, other mods without source code)
* FO4 Mods MCP — extract Bethesda files (esp, esl, esm)

# CRITICAL: SCAFFOLD-FIRST ARCHITECTURE
# If a function, state, pattern, or interface can be implemented NOW to structure the
# architecture better or eliminate the need for future alterations, it MUST be suggested and done.
# Separate the cost of STRUCTURE (enum values, state transitions, polling loops, guard patterns,
# function signatures, collision layers, listener registrations) from the cost of SYSTEMS
# (physics computation, item spawning, complex algorithms).
# If the structure is cheap and the system is expensive, implement the structure NOW with
# explicit stubs/timeouts as placeholders — NOT invisible omissions.
# The question to always ask: "will adding this feature later require changing code that already works?"
# If yes → add the scaffolding now.
# States are the skeleton, supporting systems are the muscles. Build the skeleton first.
# Retrofitting state machines, adding transitions to running systems, and forcing features into
# wrong states is exactly how previous agents introduced compounding errors.
# NEVER dismiss missing scaffolding as "FUTURE" just because its supporting system isn't built yet.
