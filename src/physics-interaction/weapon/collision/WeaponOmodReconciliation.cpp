#include "physics-interaction/weapon/WeaponCollisionInternal.h"
#include "physics-interaction/weapon/WeaponOmodCollisionPolicy.h"

namespace rock
{
    namespace
    {
        namespace policy = weapon_omod_collision_policy;
        constexpr std::string_view kOwnedPrefix = "ROCK-OMOD-Collision-";

        bool metadataLayoutValidated()
        {
            static const bool valid = []() {
                if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
                const auto matches = [](std::uintptr_t offset, const std::array<std::uint8_t, 12>& expected) {
                    std::array<std::uint8_t, 12> actual{};
                    return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(offset).address()),
                        actual.data(), actual.size()) && actual == expected;
                };
                const bool result = matches(0x1DEF090, { 0x40, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x83, 0x3D, 0xE3, 0xF7, 0xB3, 0x01 }) &&
                    matches(0x1DF1200, { 0x40, 0x55, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24 }) &&
                    matches(0x1DEF8A0, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x18, 0x55, 0x57 });
                if (!result) ROCK_LOG_ERROR(Weapon, "OMOD-COLLISION stage=metadata-layout-validation recovery=disabled");
                return result;
            }();
            return valid;
        }

        bool readMetadataName(const RE::BSFixedString& name, std::string& out)
        {
            out.clear();
            const char* source = name.c_str();
            if (!source) return false;
            for (std::size_t i = 0; i < 256; ++i) {
                char value = 0;
                if (!native_memory::tryReadValue(source + i, value)) return false;
                if (!value) return true;
                out.push_back(value);
            }
            return false;
        }

        struct Census
        {
            std::vector<RE::NiAVObject*> nodes;
            std::unordered_set<RE::NiAVObject*> visited;
            bool complete{ true };
            std::size_t flattenedWitnesses{ 0 };
        };

        void scan(Census& census, RE::NiAVObject* object, std::size_t limit, int depth = 0)
        {
            if (!object || census.visited.contains(object)) return;
            if (depth > 32 || census.nodes.size() >= limit) {
                census.complete = false;
                return;
            }
            census.visited.insert(object);
            // Owned collision geometry is never evidence that the engine has
            // registered the authored model. Ordinary render scans also stop
            // at these app-culled containers.
            if (std::string_view(safeNodeName(object)).starts_with(kOwnedPrefix)) return;
            census.nodes.push_back(object);
            if (auto* node = object->IsNode()) {
                for (const auto& child : node->children) {
                    scan(census, child.get(), limit, depth + 1);
                    if (!census.complete) break;
                }
            }
        }

        bool below(const RE::NiAVObject* root, const RE::NiAVObject* object)
        {
            RE::NiTransform local{};
            return root && object && tryResolveDescendantLocalTransform(root, object, local);
        }

        bool cloneIdentityMatches(const Census& original, const Census& copied)
        {
            if (!original.complete || !copied.complete) return false;
            std::vector<std::string_view> originalNames, copiedNames;
            originalNames.reserve(original.nodes.size());
            copiedNames.reserve(copied.nodes.size());
            for (const auto* node : original.nodes) originalNames.emplace_back(safeNodeName(node));
            for (const auto* node : copied.nodes) copiedNames.emplace_back(safeNodeName(node));
            return policy::cloneNamesPreserved(originalNames, copiedNames);
        }

        Census equippedCensus(RE::NiAVObject* weaponRoot)
        {
            Census result;
            result.nodes.reserve(512);
            result.visited.reserve(512);
            visitGeneratedWeaponMeshRootCandidates(weaponRoot, [&](const WeaponMeshRootCandidate& candidate) {
                scan(result, candidate.root, policy::kMaximumSceneNodes);
            });
            // The animation tree is an additional discovery witness, not a
            // replacement geometry inventory. Only logical Weapon descendants
            // from the current first-person tree may contribute references.
            auto* tree = f4vr::getFirstPersonBoneTree();
            if (weaponAnimFlattenedTreeValid(tree)) {
                for (int i = 0; i < tree->numTransforms; ++i) {
                    int cursor = i;
                    bool weaponOwned = false;
                    for (int hop = 0; hop < tree->numTransforms && cursor >= 0 && cursor < tree->numTransforms; ++hop) {
                        const auto& bone = tree->transforms[cursor];
                        if (_stricmp(weaponAnimFlattenedTransformName(bone), "Weapon") == 0) {
                            weaponOwned = true;
                            break;
                        }
                        cursor = bone.parPos;
                    }
                    if (weaponOwned && tree->transforms[i].refNode) {
                        ++result.flattenedWitnesses;
                        scan(result, tree->transforms[i].refNode, policy::kMaximumSceneNodes);
                    }
                }
            }
            return result;
        }

        RE::NiNode* uniqueParent(const Census& census, RE::NiAVObject* weaponRoot, std::string_view name,
            std::uint32_t index, std::size_t& count)
        {
            count = 0;
            RE::NiNode* result = nullptr;
            const auto resolved = policy::instanceName(name, index);
            for (auto* object : census.nodes) {
                if (_stricmp(safeNodeName(object), resolved.c_str()) != 0 || !below(weaponRoot, object)) continue;
                if (auto* node = object->IsNode()) {
                    result = node;
                    ++count;
                }
            }
            return count == 1 ? result : nullptr;
        }

        bool activeBelow(RE::NiAVObject* root, RE::NiAVObject* object)
        {
            // Equip-time root culling is a presentation transition, not an
            // instruction to delete the weapon's physical attachments.
            for (int hop = 0; object && hop < 64; ++hop, object = object->parent) {
                if (object == root) return true;
                if (!weaponVisualNodeVisible(object)) return false;
            }
            return false;
        }

        struct NativeConnectPointParentLayout
        {
            std::uint64_t referenceState;
            RE::BSFixedString parentNodeName;
            RE::BSFixedString connectPointName;
            RE::NiQuaternion rotation;
            RE::NiPoint3 translation;
            float scale;
        };
        static_assert(offsetof(NativeConnectPointParentLayout, parentNodeName) == 0x08);
        static_assert(offsetof(NativeConnectPointParentLayout, connectPointName) == 0x10);
        static_assert(offsetof(NativeConnectPointParentLayout, rotation) == 0x18);
        static_assert(offsetof(NativeConnectPointParentLayout, translation) == 0x28);
        static_assert(sizeof(NativeConnectPointParentLayout) == 0x38);

        struct NativeConnectPointParentArrayLayout
        {
            NativeConnectPointParentLayout** entries;
            std::uint32_t capacity;
            std::uint32_t pad0C;
            std::uint32_t count;
            std::uint32_t pad14;
        };
        static_assert(sizeof(NativeConnectPointParentArrayLayout) == 0x18);


        struct Model
        {
            std::uint32_t formId{};
            std::uint32_t attachPoint{};
            std::uint32_t index{};
            std::string path;
            RE::NiPointer<RE::NiNode> root;
            Census census;
        };

        struct Socket
        {
            RE::NiNode* parent{};
            RE::NiTransform local{};
            std::string point;
        };

        // VR Children: ctor 141DEF090 initializes the array at +18/count
        // +28 and skinned byte +30; DoAttach 141DF1200 reads the same array,
        // count and byte. Parents is corroborated by 141DEF8A0 and the
        // existing F4SEVR definition. Copy only bounded, readable fields.
        bool childPoints(RE::NiNode* root, std::vector<std::string>& points, bool& skinned)
        {
            auto* extra = root ? root->GetExtraData(RE::BSFixedString("CPT")) : nullptr;
            if (!extra || !niObjectRttiChainContains(extra, "BSConnectPoint::Children")) return false;
            RE::BSFixedString* names = nullptr;
            std::uint32_t count = 0;
            std::uint8_t skin = 0;
            if (!native_memory::tryReadField(extra, 0x18, names) ||
                !native_memory::tryReadField(extra, 0x28, count) ||
                !native_memory::tryReadField(extra, 0x30, skin) ||
                count == 0 || count > 32 || skin > 1 ||
                !native_memory::pointerRangeLooksReadable(names, count * sizeof(*names))) return false;
            skinned = skin != 0;
            for (std::uint32_t i = 0; i < count; ++i) {
                std::string text;
                if (!readMetadataName(names[i], text)) return false;
                const auto point = policy::parentPointName(text);
                if (point.empty()) return false;
                points.push_back(point);
            }
            return true;
        }

        bool finiteTransform(const RE::NiTransform& transform)
        {
            return weaponTransformFinite(transform) && transform.scale > 0.0001f;
        }

        bool sameTransform(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            if (!finiteTransform(a) || !finiteTransform(b) ||
                std::abs(a.scale - b.scale) > 0.0001f ||
                (a.translate - b.translate).Length() > 0.05f) return false;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    if (std::abs(a.rotate.entry[r][c] - b.rotate.entry[r][c]) > 0.001f) return false;
            return true;
        }

        RE::NiTransform socketTransform(const NativeConnectPointParentLayout& point)
        {
            const float q[4]{ point.rotation.x, point.rotation.y, point.rotation.z, point.rotation.w };
            // NiQuaternion::ToRotation (1401AA810), called by ConnectChild
            // (141DEF8A0), writes the transpose of the Havok row convention.
            RE::NiTransform result{};
            result.rotate = transform_math::transposeRotation(transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(q));
            result.translate = point.translation;
            result.scale = point.scale;
            const float norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
            if (!std::isfinite(norm) || std::abs(norm - 1.0f) > 0.01f) result.scale = 0.0f;
            return result;
        }

        void findSockets(const Census& metadata, const Census& live, RE::NiAVObject* weaponRoot,
            const std::vector<std::string>& points, std::uint32_t index, std::vector<Socket>& result, bool& complete)
        {
            for (auto* object : metadata.nodes) {
                auto* node = object->IsNode();
                auto* extra = node ? node->GetExtraData(RE::BSFixedString("CPA")) : nullptr;
                if (!extra || !niObjectRttiChainContains(extra, "BSConnectPoint::Parents")) continue;
                NativeConnectPointParentArrayLayout array{};
                if (!native_memory::guardedCopyFromMemory(
                        reinterpret_cast<const char*>(extra) + sizeof(RE::NiExtraData), &array, sizeof(array)) ||
                    array.count > array.capacity || array.count > 64 ||
                    (array.count && !native_memory::pointerRangeLooksReadable(array.entries, array.count * sizeof(*array.entries)))) {
                    complete = false;
                    continue;
                }
                for (std::uint32_t i = 0; i < array.count; ++i) {
                    NativeConnectPointParentLayout* point = nullptr;
                    if (!native_memory::tryReadValue(array.entries + i, point) ||
                        !native_memory::pointerRangeLooksReadable(point, sizeof(*point))) {
                        complete = false;
                        continue;
                    }
                    std::string pointName, parentName;
                    if (!readMetadataName(point->connectPointName, pointName) ||
                        !readMetadataName(point->parentNodeName, parentName)) { complete = false; continue; }
                    const auto end = pointName.find_first_of(",|");
                    const std::string pointBase(pointName.substr(0, end));
                    if (std::ranges::none_of(points, [&](const auto& name) { return _stricmp(name.c_str(), pointBase.c_str()) == 0; })) continue;
                    const char* name = parentName.c_str();
                    std::size_t matches = 0;
                    RE::NiNode* parent = name && *name ? uniqueParent(live, weaponRoot, name, index, matches) : nullptr;
                    if (!name || !*name) {
                        // Empty parent means the metadata-owning installed
                        // root. A template root name cannot establish it.
                        parent = below(weaponRoot, node) ? node : nullptr;
                        matches = parent ? 1 : 0;
                    }
                    auto local = socketTransform(*point);
                    if (!parent && matches == 0 && name && *name) {
                        // Preserve authored static paths omitted by native
                        // assembly (e.g. CROSSBarrelOffsetNode). Fold their
                        // exact locals into the collision binding instead of
                        // inserting missing parents into the rendered graph.
                        RE::NiAVObject* authored = nullptr;
                        std::size_t authoredCount = 0;
                        for (auto* candidate : metadata.nodes) {
                            if (_stricmp(safeNodeName(candidate), name) == 0 && candidate->IsNode()) {
                                authored = candidate;
                                ++authoredCount;
                            }
                        }
                        if (authoredCount == 1) {
                            auto path = transform_math::makeIdentityTransform<RE::NiTransform>();
                            for (int hop = 0; authored && hop < 32; ++hop, authored = authored->parent) {
                                std::size_t ancestorMatches = 0;
                                auto* ancestor = uniqueParent(live, weaponRoot, safeNodeName(authored), index, ancestorMatches);
                                if (ancestorMatches > 1) { complete = false; break; }
                                if (ancestor) {
                                    parent = ancestor;
                                    local = transform_math::composeTransforms(path, local);
                                    break;
                                }
                                if (authored->controllers || !weaponVisualNodeVisible(authored) || !finiteTransform(authored->local)) break;
                                path = transform_math::composeTransforms(authored->local, path);
                            }
                        }
                    }
                    if (matches > 1 || !finiteTransform(local)) { complete = false; continue; }
                    if (!parent) continue;
                    const auto duplicate = std::ranges::find_if(result, [&](const Socket& other) {
                        return other.parent == parent && other.point == pointBase && sameTransform(other.local, local);
                    });
                    if (duplicate == result.end()) result.push_back({ parent, local, pointBase });
                }
            }
            complete = complete && metadata.complete;
        }

        struct Binding
        {
            RE::NiNode* parent{};
            RE::NiTransform local{};
            bool active{};
            const char* reason{ "parent-unresolved" };
        };

        Binding bindShape(RE::NiAVObject* shape, RE::NiNode* templateRoot, const Socket* socket,
            const Census& live, RE::NiAVObject* weaponRoot, std::uint32_t index, bool rawGeometry, bool skinConnection)
        {
            Binding result;
            RE::NiTransform local = shape->local;
            if (!finiteTransform(local)) { result.reason = "invalid-shape-transform"; return result; }
            auto* parent = shape->parent;
            auto* top = shape;
            for (int hop = 0; parent && parent != templateRoot && hop < 32; ++hop) {
                std::size_t matches = 0;
                auto* liveParent = uniqueParent(live, weaponRoot, safeNodeName(parent), index, matches);
                if (matches > 1) { result.reason = "animated-parent-ambiguous"; return result; }
                if (liveParent) {
                    result.parent = liveParent;
                    result.local = local;
                    result.active = activeBelow(weaponRoot, liveParent) && weaponVisualNodeVisible(shape);
                    result.reason = "live-animation-parent";
                    return result;
                }
                // A missing controller branch cannot be reconstructed by
                // freezing its reference pose or guessing from a clip name.
                if (parent->controllers) { result.reason = "animated-parent-missing"; return result; }
                if (!weaponVisualNodeVisible(parent)) { result.reason = "inactive-template-branch"; return result; }
                local = transform_math::composeTransforms(parent->local, local);
                top = parent;
                parent = parent->parent;
            }
            if (parent != templateRoot || !socket) return result;
            // ConnectChild 141DEF8A0 uses the CPT skinned flag to place the
            // socket on each immediate child (141DF2600), then identities
            // the root. That flag is distinct from a shape's skin instance.
            // Rigid geometry can still use the skinned connection convention.
            if (skinConnection && !rawGeometry) {
                if (!tryResolveDescendantLocalTransform(top, shape, local)) return result;
            }
            result.parent = socket->parent;
            result.local = transform_math::composeTransforms(socket->local, local);
            result.active = activeBelow(weaponRoot, result.parent) && weaponVisualNodeVisible(shape);
            result.reason = rawGeometry ? "raw-authored-socket" : "authored-socket";
            return result;
        }

        bool matchingGeometry(RE::BSTriShape* a, RE::BSTriShape* b,
            const RE::NiTransform& aInParent, const RE::NiTransform& bInParent)
        {
            std::uint32_t aTriangles = 0, bTriangles = 0;
            std::uint16_t aVertices = 0, bVertices = 0;
            if (!native_memory::tryReadField(a, VROffset::numTriangles, aTriangles) ||
                !native_memory::tryReadField(b, VROffset::numTriangles, bTriangles) ||
                !native_memory::tryReadField(a, VROffset::numVertices, aVertices) ||
                !native_memory::tryReadField(b, VROffset::numVertices, bVertices) ||
                aTriangles != bTriangles || aVertices != bVertices || aTriangles == 0) return false;
            std::vector<TriangleData> worldA, localA, worldB, localB;
            if (extractTrianglesFromTriShape(a, worldA, nullptr, &localA) <= 0 ||
                extractTrianglesFromTriShape(b, worldB, nullptr, &localB) <= 0 ||
                localA.empty() || localA.size() != localB.size()) return false;
            bool sameLocal = true;
            bool samePlaced = true;
            for (std::size_t i = 0; i < localA.size(); ++i) {
                sameLocal = sameLocal && (localA[i].v0 - localB[i].v0).Length() <= 0.01f &&
                    (localA[i].v1 - localB[i].v1).Length() <= 0.01f &&
                    (localA[i].v2 - localB[i].v2).Length() <= 0.01f;
                const auto matchesPlaced = [&](const auto& p, const auto& q) {
                    return (transform_math::localPointToWorld(aInParent, p) -
                        transform_math::localPointToWorld(bInParent, q)).Length() <= 0.01f;
                };
                samePlaced = samePlaced && matchesPlaced(localA[i].v0, localB[i].v0) &&
                    matchesPlaced(localA[i].v1, localB[i].v1) && matchesPlaced(localA[i].v2, localB[i].v2);
                if (!sameLocal && !samePlaced) return false;
            }
            return sameLocal || samePlaced;
        }

        RE::BSTriShape* findNativeShape(const Census& census, const Binding& binding, RE::BSTriShape* shape,
            std::string_view name, std::uint32_t index)
        {
            const auto resolved = policy::instanceName(name, index);
            for (auto* object : census.nodes) {
                if (_stricmp(safeNodeName(object), resolved.c_str()) != 0 ||
                    !activeBelow(binding.parent, object)) continue;
                auto* native = object->IsTriShape();
                RE::NiTransform local{};
                if (native && tryResolveDescendantLocalTransform(binding.parent, native, local) &&
                    matchingGeometry(shape, native, binding.local, local)) return native;
            }
            return nullptr;
        }

        bool physicalShape(RE::NiAVObject* object)
        {
            auto* shape = object->IsTriShape();
            return shape && classifyGeneratedWeaponEffectGeometry(shape) == weapon_effect_geometry_policy::ExclusionReason::None;
        }
        bool applyEquippedOmodModelCustomization(
            RE::BGSMod::Attachment::Mod* omod,
            RE::NiNode* clonedRoot,
            RE::TBO_InstanceData* instanceData)
        {
            if (!omod || !clonedRoot) {
                return false;
            }

            static const bool entryValidated = []() {
                constexpr std::array<std::uint8_t, 12> kExpectedPrefix{
                    0x48, 0x85, 0xC9, 0x0F, 0x84, 0x16, 0x01, 0x00, 0x00, 0x48, 0x8B, 0xC4
                };
                const auto address = REL::Offset(offsets::kFunc_ApplyOmodModelCustomization).address();
                std::array<std::uint8_t, kExpectedPrefix.size()> actual{};
                const bool valid = REL::Module::IsVR() &&
                    REL::Module::get().version() == F4SE::RUNTIME_VR_1_2_72 &&
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) &&
                    actual == kExpectedPrefix;
                if (!valid) {
                    ROCK_LOG_ERROR(Weapon, "OMOD physical enrichment disabled: model-customization entry validation failed at 0x{:X}", address);
                }
                return valid;
            }();
            if (!entryValidated) {
                return false;
            }

            using ApplyModelCustomization = void (*)(RE::NiAVObject*, RE::BGSModelMaterialSwap*, void*, RE::TBO_InstanceData*, void*);
            const auto applyCustomization = reinterpret_cast<ApplyModelCustomization>(
                REL::Offset(offsets::kFunc_ApplyOmodModelCustomization).address());
            applyCustomization(clonedRoot, static_cast<RE::BGSModelMaterialSwap*>(omod), nullptr, instanceData, nullptr);
            return true;
        }


    }

    void WeaponCollision::clearOmodCollisionSources()
    {
        // Both endpoints are retained. Never dereference a remembered raw
        // engine parent after an equip/world transition.
        for (auto& source : _omod.collisionSources) {
            if (source.parent && source.container &&
                source.container->parent == source.parent.get()) {
                source.parent->DetachChild(source.container.get());
            }
        }
        _omod.collisionSources.clear();
        _omod.sourceEquippedKey = 0;
        _omod.nativeCensusKey = 0;
        _omod.diagnosticFrameCounter = 0;
    }

    WeaponCollision::OmodReconciliationResult WeaponCollision::maybeRunWeaponOmodReconciliation(
        RE::NiAVObject* weaponNode, std::uint64_t auditedEquippedKey, bool forceBeforeInitialBuild)
    {
        OmodReconciliationResult result{};
        auto* root = weaponNode ? weaponNode->IsNode() : nullptr;
        if (!root || !auditedEquippedKey || !metadataLayoutValidated()) return result;
        if (!forceBeforeInitialBuild) {
            if (_omod.reconciledBodySetKey != _identity.cachedBodySetKey) {
                _omod.reconciledBodySetKey = _identity.cachedBodySetKey;
                _omod.frameCounter = 0;
            }
            // A bounded census also allows a later native assembly to take
            // ownership back from a reconstruction. No render repair runs.
            if (_omod.diagnosticFrameCounter < (std::numeric_limits<int>::max)()) ++_omod.diagnosticFrameCounter;
            if (++_omod.frameCounter < 90) return result;
        }
        _omod.frameCounter = 0;
        const bool emitCoverageDiagnostics = g_rockConfig.rockDebugWeaponOmodCoverageAudit &&
            (forceBeforeInitialBuild || _omod.diagnosticFrameCounter >=
                (std::max)(30, g_rockConfig.rockDebugWeaponOmodCoverageAuditIntervalFrames));
        if (emitCoverageDiagnostics) _omod.diagnosticFrameCounter = 0;
        result.ran = true;
        const bool changedOwner = _omod.sourceEquippedKey != auditedEquippedKey;
        if (changedOwner && !_omod.collisionSources.empty()) {
            retireActiveWeaponBodiesForSceneTransition(_cachedWorld, "omod-source-owner-change");
            clearPendingGeneratedWeaponBuild(_cachedWorld, true);
            clearGeneratedSourceCache();
            clearOmodCollisionSources();
            result.sceneEnriched = true;
        }
        _omod.sourceEquippedKey = auditedEquippedKey;
        if (changedOwner) _omod.nativeCensusKey = 0;

        auto live = equippedCensus(root);
        if (!live.complete) {
            ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                "OMOD-COLLISION key={:016X} stage=scene-census-incomplete nodes={} limit={} recovery=withheld",
                auditedEquippedKey, live.nodes.size(), policy::kMaximumSceneNodes);
            return result;
        }
        std::uint64_t censusKey = auditedEquippedKey;
        for (auto* node : live.nodes) {
            mixWeaponVisualKey(censusKey, reinterpret_cast<std::uintptr_t>(node));
            mixWeaponVisualKey(censusKey, reinterpret_cast<std::uintptr_t>(node->parent));
            mixWeaponVisualKey(censusKey, weaponVisualNodeVisible(node) ? 1 : 0);
            weapon_visual_composition_policy::mixString(censusKey, safeNodeName(node));
        }
        if (_omod.nativeCensusKey == censusKey) {
            if (emitCoverageDiagnostics) ROCK_LOG_INFO(Weapon,
                "OMOD-COLLISION key={:016X} census=unchanged owned={} nodes={} flatWitnesses={}",
                auditedEquippedKey, _omod.collisionSources.size(), live.nodes.size(), live.flattenedWitnesses);
            return result;
        }
        auto* player = f4vr::getPlayer();
        auto* equip = f4vr::getEquippedWeaponItem();
        auto* weapon = equip ? equip->item.object : nullptr;
        auto* instance = equip ? equip->item.instanceData.get() : nullptr;
        const auto* extra = weapon ? findEquippedWeaponObjectInstanceExtra(player, weapon, instance) : nullptr;
        if (!extra || !extra->values) return result;
        const auto mods = extra->GetIndexData();
        if (mods.size() > 128) {
            ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                "OMOD-COLLISION key={:016X} stage=record-cap records={} recovery=withheld", auditedEquippedKey, mods.size());
            return result;
        }
        std::vector<Model> models;
        bool modelsComplete = true;
        models.reserve(mods.size());
        for (const auto& entry : mods) {
            if (entry.disabled) continue;
            auto* mod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(entry.objectID);
            if (!mod || !mod->model.c_str() || !*mod->model.c_str()) continue;
            auto* keyword = RE::BGSKeyword::GetTypedKeywordByIndex(RE::KeywordType::kAttachPoint, mod->attachPoint.keywordIndex);
            Model model{};
            model.formId = mod->formID;
            model.attachPoint = keyword ? keyword->formID : 0;
            model.index = entry.index;
            model.path = mod->model.c_str();
            model.root = loadCompleteOmodModelTemplate(model.path);
            if (!model.root) {
                modelsComplete = false;
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "OMOD-COLLISION omod={:08X} stage=model-unavailable path='{}'", model.formId, model.path);
                continue;
            }
            scan(model.census, model.root.get(), policy::kMaximumTemplateNodes);
            models.push_back(std::move(model));
        }

        bool retired = false;
        auto prepareMutation = [&]() {
            if (!retired) {
                retireActiveWeaponBodiesForSceneTransition(_cachedWorld, "omod-collision-source-change");
                clearPendingGeneratedWeaponBuild(_cachedWorld, true);
                clearGeneratedSourceCache();
                retired = true;
            }
            result.sceneEnriched = true;
        };
        // Remove obsolete or detached sources before releasing their strong
        // scene references. Shape identity includes the installed entry index.
        for (auto it = _omod.collisionSources.begin(); it != _omod.collisionSources.end();) {
            const bool installed = std::ranges::any_of(models, [&](const Model& model) {
                return model.formId == it->omodFormId && model.index == it->modIndex;
            });
            if (!installed || !below(root, it->container.get())) {
                prepareMutation();
                if (it->container->parent == it->parent.get()) it->parent->DetachChild(it->container.get());
                it = _omod.collisionSources.erase(it);
            } else ++it;
        }

        std::size_t added = 0, nativeCount = 0, removed = 0, unresolved = 0;
        for (auto& model : models) {
            const auto unresolvedBeforeModel = unresolved;
            const auto addedBeforeModel = added;
            const auto nativeBeforeModel = nativeCount;
            if (!model.census.complete) {
                ++unresolved;
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "OMOD-COLLISION omod={:08X} stage=template-census-incomplete recovery=withheld", model.formId);
                continue;
            }
            std::vector<std::string> pointNames;
            bool skinConnection = false;
            const bool hasChildPoints = childPoints(model.root.get(), pointNames, skinConnection);
            std::vector<Socket> sockets;
            bool socketsComplete = true;
            if (hasChildPoints) {
                findSockets(live, live, root, pointNames, model.index, sockets, socketsComplete);
                for (const auto& provider : models) {
                    if (provider.formId != model.formId)
                        findSockets(provider.census, live, root, pointNames, model.index, sockets, socketsComplete);
                }
            }
            const Socket* socket = socketsComplete && sockets.size() == 1 ? &sockets.front() : nullptr;

            // Keep the known receiver postprocessor exception, selected only
            // when its raw physical signature strictly contains the normal
            // one and it carries native collision. It never drives visuals.
            RE::NiPointer<RE::NiNode> rawRoot;
            Census rawCensus;
            bool rawGeometry = false;
            if (model.attachPoint == weapon_part_record_identity_policy::kAttachPointReceiver) {
                rawRoot = loadGeometryInspectionOmodModelTemplate(model.path);
                if (rawRoot) {
                    scan(rawCensus, rawRoot.get(), policy::kMaximumTemplateNodes);
                    const auto physicalCount = [](const Census& c) {
                        return std::ranges::count_if(c.nodes, physicalShape);
                    };
                    const bool collision = std::ranges::any_of(rawCensus.nodes, [](auto* node) {
                        return node->collisionObject && niObjectRttiChainContains(node->collisionObject.get(), "bhkNPCollisionObject");
                    });
                    const bool superset = std::ranges::all_of(model.census.nodes, [&](auto* node) {
                        return !physicalShape(node) || std::ranges::any_of(rawCensus.nodes, [&](auto* raw) {
                            return physicalShape(raw) && _stricmp(safeNodeName(node), safeNodeName(raw)) == 0;
                        });
                    });
                    rawGeometry = rawCensus.complete && collision && superset && physicalCount(rawCensus) > physicalCount(model.census);
                }
            }
            auto* physicalRoot = rawGeometry ? rawRoot.get() : model.root.get();

            RE::NiPointer<RE::NiNode> clone;
            clone.reset(f4vr::cloneNode(physicalRoot));
            auto* mod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(model.formId);
            if (!clone) {
                modelsComplete = false;
                ++unresolved;
                continue;
            }
            const auto& sourceCensus = rawGeometry ? rawCensus : model.census;
            Census copied;
            scan(copied, clone.get(), policy::kMaximumTemplateNodes);
            const bool namesBeforeCustomization = cloneIdentityMatches(sourceCensus, copied);
            if (!namesBeforeCustomization || !applyEquippedOmodModelCustomization(mod, clone.get(), instance)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "OMOD-COLLISION omod={:08X} stage=clone-identity sourceNodes={} cloneNodes={} namesBeforeCustomization={} recovery=withheld",
                    model.formId, sourceCensus.nodes.size(), copied.nodes.size(), namesBeforeCustomization);
                modelsComplete = false;
                ++unresolved;
                continue;
            }
            Census physical;
            scan(physical, clone.get(), policy::kMaximumTemplateNodes);
            const bool namesAfterCustomization = cloneIdentityMatches(sourceCensus, physical);
            if (emitCoverageDiagnostics || !namesAfterCustomization) {
                ROCK_LOG_INFO(Weapon,
                    "OMOD-COLLISION clone omod={:08X} sourceNodes={} clonedNodes={} namesBeforeCustomization={} namesAfterCustomization={}",
                    model.formId, sourceCensus.nodes.size(), physical.nodes.size(), namesBeforeCustomization, namesAfterCustomization);
            }
            if (!namesAfterCustomization) { ++unresolved; continue; }
            std::uint32_t shapeIndex = 0;
            for (auto* object : physical.nodes) {
                if (!physicalShape(object)) continue;
                const auto thisShape = shapeIndex++;
                auto* shape = object->IsTriShape();
                void* skin = nullptr;
                const bool skinKnown = native_memory::tryReadField(shape, VROffset::skinInstance, skin);
                const bool skinned = !skinKnown || skin;
                auto binding = bindShape(shape, clone.get(), socket, live, root, model.index, rawGeometry, skinConnection);
                if (!binding.parent || !finiteTransform(binding.local)) {
                    ++unresolved;
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "OMOD-COLLISION key={:016X} omod={:08X} index={} shape='{}' stage={} sockets={} socketComplete={} flatWitnesses={} recovery=withheld",
                        auditedEquippedKey, model.formId, model.index, safeNodeName(shape), binding.reason,
                        sockets.size(), socketsComplete, live.flattenedWitnesses);
                    continue;
                }
                auto owned = std::ranges::find_if(_omod.collisionSources, [&](const OmodCollisionSource& source) {
                    return source.omodFormId == model.formId && source.modIndex == model.index && source.shapeIndex == thisShape;
                });
                auto* native = !skinned ? findNativeShape(live, binding, shape, safeNodeName(shape), model.index) : nullptr;
                const auto decision = policy::decide(physical.complete, 1, binding.active, skinned, native != nullptr,
                    owned != _omod.collisionSources.end());
                if (decision == policy::Coverage::NativeGeometry || decision == policy::Coverage::InactiveBranch) {
                    if (native) ++nativeCount;
                    if (owned != _omod.collisionSources.end()) {
                        prepareMutation();
                        if (owned->container->parent == owned->parent.get()) owned->parent->DetachChild(owned->container.get());
                        _omod.collisionSources.erase(owned);
                        ++removed;
                    }
                    continue;
                }
                if (decision == policy::Coverage::OwnedGeometry) {
                    // Rebinding is an atomic body-set transition, never an
                    // in-place move under a published collider.
                    if (owned->parent.get() == binding.parent && sameTransform(owned->container->local, binding.local)) continue;
                    prepareMutation();
                    if (owned->container->parent == owned->parent.get()) owned->parent->DetachChild(owned->container.get());
                    _omod.collisionSources.erase(owned);
                } else if (decision != policy::Coverage::Recoverable) {
                    ++unresolved;
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "OMOD-COLLISION omod={:08X} shape='{}' stage=unsupported-skin recovery=withheld", model.formId, safeNodeName(shape));
                    continue;
                }
                if (_omod.collisionSources.size() >= policy::kMaximumSources) {
                    ++unresolved;
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "OMOD-COLLISION key={:016X} stage=source-cap recovery=withheld", auditedEquippedKey);
                    break;
                }
                if (shape->controllers) {
                    ++unresolved;
                    ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                        "OMOD-COLLISION omod={:08X} shape='{}' stage=shape-controller-unmapped recovery=withheld", model.formId, safeNodeName(shape));
                    continue;
                }
                // Validate extractability before attaching any owned source.
                std::vector<TriangleData> triangles, locals;
                if (extractTrianglesFromTriShape(shape, triangles, nullptr, &locals) <= 0 || locals.empty()) {
                    ++unresolved;
                    continue;
                }
                auto* clonedParent = shape->parent ? shape->parent->IsNode() : nullptr;
                if (!clonedParent) { ++unresolved; continue; }
                RE::NiPointer<RE::NiAVObject> leaf;
                clonedParent->DetachChild(shape, leaf);
                if (!leaf) { ++unresolved; continue; }
                auto container = native_scene::createEngineNiNode(1);
                if (!container) { ++unresolved; continue; }
                const auto name = fmt::format("{}{:08X}-{}-{}", kOwnedPrefix, model.formId, model.index, thisShape);
                container->name = name.c_str();
                container->flags.flags |= 1u;
                container->local = binding.local;
                const std::string authoredName = safeNodeName(leaf.get());
                leaf->name = (name + "-shape").c_str();
                leaf->local = transform_math::makeIdentityTransform<RE::NiTransform>();
                // The selected leaf is rigid. Clone-owned native physics and
                // controllers must never acquire a second runtime registration.
                leaf->collisionObject = nullptr;
                leaf->controllers = nullptr;
                leaf->userData = 0;
                container->AttachChild(leaf.get(), true);
                prepareMutation();
                binding.parent->AttachChild(container.get(), true);
                if (container->parent != binding.parent || leaf->parent != container.get()) {
                    binding.parent->DetachChild(container.get());
                    ++unresolved;
                    continue;
                }
                OmodCollisionSource source{};
                source.container = container;
                source.parent.reset(binding.parent);
                source.shape = leaf.get();
                source.omodFormId = model.formId;
                source.attachPointFormId = model.attachPoint;
                source.modIndex = model.index;
                source.shapeIndex = thisShape;
                source.authoredName = authoredName;
                source.connectPoint = socket ? socket->point : "";
                _omod.collisionSources.push_back(std::move(source));
                ++added;
                ROCK_LOG_INFO(Weapon,
                    "OMOD-COLLISION key={:016X} omod={:08X} index={} model='{}' shape='{}' nativeMatch=no parent='{}' point='{}' binding={} local=({:.3f},{:.3f},{:.3f}) scale={:.4f} triangles={} render=hidden source=owned",
                    auditedEquippedKey, model.formId, model.index, model.path, authoredName,
                    safeNodeName(binding.parent), socket ? socket->point : "", binding.reason,
                    binding.local.translate.x, binding.local.translate.y, binding.local.translate.z, binding.local.scale, locals.size());
            }
            if (emitCoverageDiagnostics || unresolved != unresolvedBeforeModel) {
                ROCK_LOG_INFO(Weapon,
                    "OMOD-COLLISION model omod={:08X} index={} attachPoint={:08X} path='{}' sockets={} socketComplete={} childMetadata={} skinConnection={} physicalShapes={} native={} added={} unresolved={}",
                    model.formId, model.index, model.attachPoint, model.path, sockets.size(), socketsComplete,
                    hasChildPoints, skinConnection, shapeIndex, nativeCount - nativeBeforeModel,
                    added - addedBeforeModel, unresolved - unresolvedBeforeModel);
            }
        }
        if (added || removed || unresolved || emitCoverageDiagnostics) {
            ROCK_LOG_INFO(Weapon,
                "OMOD-COLLISION key={:016X} records={} native={} added={} removed={} owned={} unresolved={} sceneNodes={} flatWitnesses={}",
                auditedEquippedKey, models.size(), nativeCount, added, removed, _omod.collisionSources.size(), unresolved,
                live.nodes.size(), live.flattenedWitnesses);
        }
        if (result.sceneEnriched) requestWorkbenchExitRebuild();
        _omod.nativeCensusKey = modelsComplete ? censusKey : 0;
        return result;
    }
}
