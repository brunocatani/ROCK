# REPOSITORY DEFINITION

# ROCK — Realistic Overengineered Character Kinetics

This is the project folder for ROCK, a physics-based hand interaction system for Fallout 4 VR. It ports HIGGS (Skyrim VR) to FO4VR, building into FRIK as the body/skeleton provider.

The HIGGS mod has been completely mapped and the codebase is present in the HIGGS folder.

# Git
- Repository: `https://github.com/brunocatani/ROCK` (private)
- Branch: `main`
- Do NOT add `Co-Authored-By` lines or mention Claude/AI in commit messages.

# Main Guidelines

# ABSOLUTE RULE: QUALITY MANDATE — NO EXCEPTIONS
# ALL implementations MUST be proper, high-quality, long-term, production-grade solutions.
# NO quick fixes. NO "just to test." NO "for now" workarounds. NO barely functional code.
# NO "easy wins." NO "simple fix." NO shortcuts of ANY kind. NEVER suggest them.
# NO skipping features because they're complex — HIGGS found a way, so will we.
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
# Every function depends on other functions. Fixing tau means nothing without the per-frame
# update formula, which means nothing without the collision listener, which needs per-body
# registration, which ties into player movement compensation. They are LAYERS of ONE system.
# ALWAYS think about the WHOLE system: what depends on this, what this depends on, what
# will need to change when the next feature is added. Design as a COMPLETE COHERENT SYSTEM.

# CRITICAL: REFERENCE-DRIVEN IMPLEMENTATION
# For EVERY modification, BEFORE writing any code:
# 1. Check HIGGS source code at E:\fo4dev\skirymvr_mods\source_codes\higgs
# 3. Cross-check against the FO4VR binary via Ghidra MCP
# 4. DO NOT implement from memory, assumptions, or "general knowledge"
# 5. If the reference doesn't exist, ASK the user for it.

# CRITICAL: GHIDRA USAGE PROTOCOL
# Ghidra MCP is a shared resource. Before EVERY Ghidra operation:
# 1. EXPLAIN to the user WHY you need to use Ghidra
# 2. WAIT for the user to approve

# CRITICAL: BLIND VERIFICATION
# Use /blind-verification-audit for ALL reverse engineering claims, struct layouts,
# and offset assumptions. Never trust documentation without binary verification.
# CommonLibF4VR headers may be wrong for VR — always cross-check with Ghidra.

ALWAYS USE HIGGS AS REFERENCE IN IMPLEMENTATION OF FEATURES, SPECIALLY ON THE LOGIC AND APPROACH.
ALWAYS DO AN IN DEPTH ANALYSIS OF THE HIGGS CODEBASE WHEN CONFUSED.
TAKE PERIODIC NOTES USING .MD FILES OF THE FINDINGS.

THINGS OF IMPORTANCE ADDED MANUALLY

HAND BODY IN HIGGS - 56 = OUR HAND COLLIDER ON LAYER 43
WEAPONBODY IN HIGGS  56 = (NOT IMPLEMENTED YET) OUR WEAPON COLLIDER, WHEN WE RE-ENABLE PHYSICS FOR WEAPONS
HELD OBJECT KEYFRAMED IN HIGGS = OUR HELD OBJECT, OBJECT KEEPS IN THE SAME LAYER AS IS
HELD OBJECT DYNAMIC IN HIGGS (CONSTRAINT ADDED) = OUR IMPLEMENTATION OF DYNAMIC HELD OBJECT

Prefer complex well implemented approaches, using existing reference mods like HIGGS, active ragdoll and others when available for implementations that do the same thing but on skyrim

* Local builds for fast iteration during development. GitHub Actions for "release" builds only after everything is tested locally.
* All code must be developed with a focus on quality, logging, testing and longevity, trying to match the specification and objectives as close as possible as this project requires complex integrations and logic
* When in doubt, always use mods/source_code from projects that already have or have similar mechanisms and features as the desired
* BETHESDA MAY HAVE STRIPPED MANY FUNCTIONS FROM FALLOUT 4 VR — when necessary for confirmation, use Ghidra to study and map everything. If something is found, save to a relevant skill or reference file. Also check the FO4 flat binary for answers.
* FO4 Mods have almost 11 years of community development — always ask or refer to available source code for implementations and mod building


## Implementations, approaches and plans must always focus a proper implementation of the desired features, no matter how many sessions it takes. Use multiple .md files with plans, specs and approaches as needed to scope, map and study everything necessary.

## Every implementation must begin with a short paragraph explaining WHY it's being done this way — what problem it solves, what alternatives were considered, and why this approach was chosen. This goes as a comment block at the top of the new code or as a note in the plan before work starts. Without this, future sessions have no context for the decisions made.

## HIGGS and Planck (Active Ragdoll) reference documents are incredibly in-depth, but FO4VR has its own peculiarities — different Havok version (2014 hknp vs 2012 hkp), stripped functions, different memory layouts, different solver behavior. NEVER blindly copy from HIGGS/Planck documentation. ALWAYS cross-check against:
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
