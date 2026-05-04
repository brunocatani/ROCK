# Collider Planet Regression Review

## Progress

- [x] Read current ROCK logs for generated collider source and provider wording.
- [x] Trace hand collider transform source from code.
- [ ] Add failing source tests for first-person skeleton source and no FRIK-source hand collider drive.
- [ ] Patch source selection and log/source naming.
- [ ] Build and run focused verification.
- [ ] Final code review.

## Evidence

- `ROCK.log` shows `HandBoneCache resolved fpSkeleton=...`, then hand collider creation reads `source=FrikRootFlattenedBoneTree` with a different skeleton/tree address.
- `HandBoneColliderSet::captureBoneLookup` hardcodes `DebugSkeletonBoneSource::FrikRootFlattenedBoneTree`.
- `DirectSkeletonBoneReader::resolveTreeSource(FrikRootFlattenedBoneTree)` reads `f4vr::getFlattenedBoneTree()` / `f4vr::getRootNode()`, while `HandBoneCache` reads `f4vr::getFirstPersonSkeleton()`.
- The hand collider path therefore mixes first-person hand frame assumptions with root flattened-tree bone transforms.
