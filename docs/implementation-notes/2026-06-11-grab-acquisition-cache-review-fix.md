# Grab Acquisition Cache Review Fix

Date: 2026-06-11

Project: ROCK

Source used: local ROCK source, local tests, implementation review of commit `4f19e20`.

Confidence: high for source-level lifecycle and hot-path issues; runtime profiling still required in FO4VR for final hitch verification.

## Problem

The first grab-acquisition cache pass removed one grab-start tree scan when selection had a ready cache, but it reused pre-prep cached body IDs as if they proved post-prep completeness. That weakened the old lifecycle invariant where bodies discovered only after active prep were marked late and forced incomplete-scan restore fallback.

The same pass also ran full recursive cache capture synchronously from selection update. That can move the hitch from the grab edge to hover/selection frames, especially for multipart loose weapons and dead/body targets.

## Fix Strategy

- Treat pre-prep cache data as a body-ID seed, not as post-prep completeness proof.
- Use cached pre-prep evidence for immediate mechanical startup only when the cache is complete for the current selected ref/root/world/body key.
- When prepared body records are rebuilt from pre-prep cached IDs after active prep, mark the lifecycle scan incomplete. This preserves root restore fallback if active prep exposed additional native bodies that were not in the pre-prep cache.
- Keep a direct post-prep scan fallback only when no safe cache is available or the cached prepared replay produces no accepted body. That preserves functionality while selection prewarm finishes.
- Move selection prewarm to a bounded cursor so `updateSelection` advances a limited amount of selected-tree discovery each frame instead of recursing the entire tree at once.
- Retain cached scene nodes with `NiPointer` and validate cached entries against the current root/collision object before replaying body IDs.

## Current Scope

This fix does not add a full post-prep background reconciliation/adoption system. Cached prepared replay is therefore explicitly incomplete unless a future post-prep cache stage is added. The important invariant is that cached replay can avoid the grab-frame scan for ready selections without pretending late-body discovery was proven complete.

## Validation Targets

- Source-boundary tests must reject direct prepared-cache replay that is treated as complete.
- Policy tests must prove pre-prep cache replay requires incomplete lifecycle handling unless post-prep proof exists.
- `custom-tests`, `ROCKPolicyTestBinaries`, full `ctest`, and `custom-fast` must pass.
