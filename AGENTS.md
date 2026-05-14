# REPOSITORY DEFINITION

# ROCK — Realistic Overengineered Character Kinetics

This is the project folder for ROCK, a physics-based hand interaction system for Fallout 4 VR. ROCK began as a port of HIGGS (Skyrim VR) concepts, functionality, and interaction logic into FO4VR, but it has evolved into its own FO4VR-native system with FRIK as the body/skeleton provider and several systems that now exceed HIGGS behavior.

# FRIK Source Authority
- Active FRIK source for ROCK integration is `E:\fo4dev\PROJECT_ROCK_V2\hFRIK`. This is our maintained FRIK fork/version and the provider source ROCK should use for API, skeleton, deployment, and behavior checks.
- Upstream/original FRIK source is `E:\fo4dev\PROJECT_ROCK_V2\Fallout-4-VR-Body`. This checkout is for upstream parity review and PR preparation only.
- Do not use `Fallout-4-VR-Body` as the active ROCK provider baseline, implementation reference, or deployed-source assumption unless the user explicitly asks for upstream parity or PR comparison work.
- When both trees contain similar files or APIs, prefer `hFRIK` for current ROCK correctness. Treat differences from `Fallout-4-VR-Body` as parity findings to document, not as automatic regressions to copy back into ROCK.

# Branching And Commit Policy
- NEVER switch branches, check out another branch, create a branch, delete a branch, merge, rebase, pull, push, or perform any other branch-changing/history-sharing operation unless the user explicitly gives permission for that exact operation. Mapping work and implementation must stay on the currently checked-out branch unless the user orders otherwise.
- Do not infer that work should start from, move to, or return to `develop`, `main`, or any other branch. The current checked-out branch is the working branch until the user explicitly orders a different branch operation.
- `main` is release-only. NEVER push, force-push, merge, or commit to `main` unless the user gives that explicit ROCK command.
- `feature/<scope>` and `fix/<scope>` are commit-message classes, NOT branch names and NOT instructions to create or switch branches.
- Commit messages must identify the change class and scope: `feature/<scope>: <specific change>` or `fix/<scope>: <specific correction>`.
- After finishing any modification or fix, commit the completed work before handing it back unless the user explicitly says not to commit. The commit message must describe what was changed and why.
- Keep feature and fix work separated into separate commits. Do not mix unrelated features, fixes, build changes, docs, and cleanup in one commit.
- Release flow is manual: the user explicitly instructs any required merge, push, tag, and local release build/deploy steps.
- Builds are local-only. Do not add GitHub Actions build or release jobs unless the user explicitly asks.

# Cleanup And Rollback Discipline
- This rule exists because stale fallback paths hide regressions, split behavior across multiple implementations, and make rollback harder than a clean git commit.
- When replacing or redesigning behavior, remove the superseded implementation, obsolete config, dead tests, stale docs, temporary scaffolding, and unused helpers in the same coherent change. Do not keep the old implementation as a fallback, shadow path, compatibility branch, dormant feature flag, or "backup" unless the user explicitly orders that fallback to remain.
- If a fallback is explicitly requested, document why it exists, how it is selected, and when it should be removed. Unexplained duplicate behavior is a bug.
- Always check the relevant repo's git status before changing files. Commit each completed coherent feature, fix, cleanup, or docs update before handing work back unless the user explicitly says not to commit.
- Commits are the rollback mechanism. Keep commits focused, include all required cleanup in the same commit as the behavior change, and never mix unrelated user changes into an agent commit.
- If unrelated dirty files or parallel agent edits prevent a safe commit, stop before committing, report the exact conflicting files, and ask how to proceed.

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
All agent-created research, plans, audits, implementation notes, and Markdown artifacts must be placed under the repo-root `docs/` folder. Do not create root-level `.md` files or scatter research Markdown elsewhere; `README.md` is the only project documentation exception unless the user explicitly instructs otherwise.

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
- Use explicit Windows presets instead of the old shared `build/` folder for normal work.
- Fast local plugin build and deploy: `cd ROCK && cmake --preset custom-fast && cmake --build build-fast --config Release --target ROCK -- /m`
- Fast output: `ROCK/build-fast/Release/ROCK.dll` and `ROCK/build-fast/Release/ROCK.pdb`; `custom-fast` copies both to `D:\FO4\mods\ROCK\F4SE\Plugins` through `COPY_PLUGIN_BASE_PATH=D:/FO4/mods/ROCK`.
- Test build: `cd ROCK && cmake --preset custom-tests && cmake --build build-tests --config Release --target ROCKPolicyTestBinaries -- /m`
- Full test run: `cd ROCK && ctest --test-dir build-tests -C Release --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- Source-boundary-only test run: `cd ROCK && ctest --test-dir build-tests -C Release -L source-boundary --output-on-failure -j %NUMBER_OF_PROCESSORS%`
- Release package build: `cd ROCK && cmake --preset custom-release && cmake --build build-release --config Release --target ROCK -- /m`
- `BUILD_ROCK_TESTS` controls C++ regression tests and PowerShell `tests/*SourceTests.ps1` source-boundary tests. C++ tests are labeled `policy`; PowerShell boundary tests are labeled `source-boundary`.
- `ROCK_PACKAGE_RELEASE` controls the `.7z` packaging post-build step. It is disabled for fast/test presets and enabled for release preset.
- Build folders are generated and can be deleted to recover space, especially the old `build/` folder after the new preset folders are verified. Do not auto-delete active preset build folders after each build because that destroys MSVC incremental/PCH state and makes the next build slower.
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
