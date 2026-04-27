#pragma once

#include "BethesdaPhysicsBody.h"
#include "GrabConstraint.h"
#include "HandCollisionSuppressionMath.h"
#include "ObjectDetection.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"
#include "RockConfig.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/BSTempEffect.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpWorld.h"

#include <array>

namespace frik::rock
{

    constexpr std::uint32_t ROCK_HAND_LAYER = 43;

    constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

    RE::hknpShape* CreateBoxShape(float hx, float hy, float hz, float convexRadius = 0.0f);

    RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world);

    enum class HandState : std::uint8_t
    {
        Idle,
        SelectedClose,
        SelectedFar,
        SelectionLocked,
        HeldInit,
        HeldBody,
        Pulled,
        PreGrabItem,
        GrabFromOtherHand,
        SelectedTwoHand,
        HeldTwoHanded,
    };

    struct GrabPivotDebugSnapshot
    {
        RE::NiPoint3 handPivotWorld{};
        RE::NiPoint3 objectPivotWorld{};
        RE::NiPoint3 handBodyWorld{};
        RE::NiPoint3 objectBodyWorld{};
        float pivotErrorGameUnits = 0.0f;
    };

    class Hand
    {
    public:
        explicit Hand(bool isLeft) : _isLeft(isLeft) {}

        bool isLeft() const { return _isLeft; }
        HandState getState() const { return _state; }
        const char* handName() const { return _isLeft ? "Left" : "Right"; }

        void reset();

        void collectHeldBodyIds(RE::TESObjectREFR* refr);

    private:
        void collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth = 10);
        void suppressHandCollisionForGrab(RE::hknpWorld* world);
        void restoreHandCollisionAfterGrab(RE::hknpWorld* world);
        void clearGrabHandCollisionSuppressionState();
        bool computeAdjustedHandTransformTarget(RE::NiTransform& outTransform) const;

    public:
        const std::vector<std::uint32_t>& getHeldBodyIds() const { return _heldBodyIds; }

        bool isHeldBodyId(std::uint32_t bodyId) const
        {
            int count = _heldBodyIdsCount.load(std::memory_order_acquire);
            for (int i = 0; i < count; i++) {
                if (_heldBodyIdsSnapshot[i] == bodyId)
                    return true;
            }
            return false;
        }

        bool isHoldingAtomic() const { return _isHoldingFlag.load(std::memory_order_acquire); }

        const SelectedObject& getSelection() const { return _currentSelection; }
        bool hasSelection() const { return _currentSelection.isValid(); }

        bool isTouching() const { return _touchActiveFrames < 5; }
        RE::TESObjectREFR* getLastTouchedRef() const { return _lastTouchedRef; }
        std::uint32_t getLastTouchedFormID() const { return _lastTouchedFormID; }
        std::uint32_t getLastTouchedLayer() const { return _lastTouchedLayer; }

        void setTouchState(RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
        {
            _lastTouchedRef = refr;
            _lastTouchedFormID = formID;
            _lastTouchedLayer = layer;
            _touchActiveFrames = 0;
        }

        bool isHolding() const { return _state == HandState::HeldInit || _state == HandState::HeldBody; }
        RE::TESObjectREFR* getHeldRef() const { return _savedObjectState.refr; }
        const ActiveConstraint& getActiveConstraint() const { return _activeConstraint; }
        const SavedObjectState& getSavedObjectState() const { return _savedObjectState; }
        bool getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const;

        bool getAdjustedHandTransform(RE::NiTransform& outTransform) const;
        bool getGrabFingerProbeDebug(std::array<RE::NiPoint3, 5>& outStart, std::array<RE::NiPoint3, 5>& outEnd) const
        {
            if (!_hasGrabFingerProbeDebug)
                return false;
            outStart = _grabFingerProbeStart;
            outEnd = _grabFingerProbeEnd;
            return true;
        }

        bool grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
            float constantRecovery);

        void updateHeldObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime, float forceFadeInTime, float tauMin, float tauMax,
            float tauIncrement, float tauDecrement, float closeThreshold, float farThreshold);

        void releaseGrabbedObject(RE::hknpWorld* world);

        void tickTouchState() { _touchActiveFrames++; }

        void updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& palmNormal,
            const RE::NiPoint3& pointingDirection, float nearRange, float farRange, RE::TESObjectREFR* otherHandRef);

        RE::hknpBodyId getCollisionBodyId() const { return _handBody.getBodyId(); }
        bool hasCollisionBody() const { return _handBody.isValid(); }
        BethesdaPhysicsBody& getHandBody() { return _handBody; }
        const BethesdaPhysicsBody& getHandBody() const { return _handBody; }

        bool createCollision(RE::hknpWorld* world, void* bhkWorld, float halfExtentX, float halfExtentY, float halfExtentZ);

        void destroyCollision(void* bhkWorld);

        void updateCollisionTransform(RE::hknpWorld* world, const RE::NiTransform& handTransform, float deltaTime);

    private:
        bool _isLeft;
        HandState _state = HandState::Idle;
        HandState _prevState = HandState::Idle;

        bool _idleDesired = false;
        bool _grabRequested = false;
        bool _releaseRequested = false;

        BethesdaPhysicsBody _handBody;

        RE::NiNode* _debugHandOriginVis = nullptr;
        RE::NiNode* _debugPalmCenterVis = nullptr;
        RE::NiNode* _debugAxisXVis = nullptr;
        RE::NiNode* _debugAxisYVis = nullptr;
        RE::NiNode* _debugAxisZVis = nullptr;
        RE::NiNode* _debugBasisVisParent = nullptr;

        SelectedObject _currentSelection;
        SelectedObject _cachedFarCandidate;
        int _farDetectCounter = 0;
        int _selectionHoldFrames = 0;
        int _deselectCooldown = 0;
        RE::TESObjectREFR* _lastDeselectedRef = nullptr;

        RE::TESObjectREFR* _lastTouchedRef = nullptr;
        std::uint32_t _lastTouchedFormID = 0;
        std::uint32_t _lastTouchedLayer = 0;
        int _touchActiveFrames = 100;

        std::atomic<int> _heldBodyContactFrame{ 100 };

        hand_collision_suppression_math::SuppressionState _grabHandCollisionSuppression{};
        bool _grabHandCollisionBroadPhaseSuppressed = false;

    public:
        bool isHeldBodyColliding() const { return _heldBodyContactFrame.load(std::memory_order_acquire) < 5; }

        void notifyHeldBodyContact() { _heldBodyContactFrame.store(0, std::memory_order_release); }

        void tickHeldBodyContact()
        {
            int current = _heldBodyContactFrame.load(std::memory_order_acquire);
            if (current < 100) {
                _heldBodyContactFrame.compare_exchange_weak(current, current + 1, std::memory_order_release, std::memory_order_relaxed);
            }
        }

    private:
        ActiveConstraint _activeConstraint;
        SavedObjectState _savedObjectState;
        float _grabStartTime = 0.0f;
        int _heldLogCounter = 0;
        int _transformWitnessLogCounter = 0;
        int _notifCounter = 0;

        RE::NiTransform _grabHandSpace;
        RE::NiTransform _adjustedHandTransform;
        bool _hasAdjustedHandTransform = false;
        float _grabVisualLerpElapsed = 0.0f;
        std::array<RE::NiPoint3, 5> _grabFingerProbeStart{};
        std::array<RE::NiPoint3, 5> _grabFingerProbeEnd{};
        bool _hasGrabFingerProbeDebug = false;

        RE::NiTransform _grabConstraintHandSpace;

        RE::NiTransform _grabBodyLocalTransform;

        RE::NiTransform _grabRootBodyLocalTransform;

        RE::NiTransform _grabOwnerBodyLocalTransform;

        RE::NiAVObject* _heldNode = nullptr;

        std::vector<std::uint32_t> _heldBodyIds;

        static constexpr int MAX_HELD_BODIES = 64;
        std::uint32_t _heldBodyIdsSnapshot[MAX_HELD_BODIES] = {};
        std::atomic<int> _heldBodyIdsCount{ 0 };

        std::atomic<bool> _isHoldingFlag{ false };

        RE::TESObjectREFR* _highlightedRef = nullptr;
        RE::ShaderReferenceEffect* _highlightEffect = nullptr;

        RE::TESEffectShader* _cachedHighlightShader = nullptr;
        std::uint32_t _cachedHighlightFormID = 0;

    public:
        void playSelectionHighlight(RE::TESObjectREFR* refr)
        {
            if (!refr || refr == _highlightedRef)
                return;
            if (!g_rockConfig.rockHighlightEnabled || g_rockConfig.rockHighlightShaderFormID == 0)
                return;

            stopSelectionHighlight();

            if (_cachedHighlightFormID != g_rockConfig.rockHighlightShaderFormID) {
                _cachedHighlightFormID = g_rockConfig.rockHighlightShaderFormID;
                _cachedHighlightShader = RE::TESForm::GetFormByID<RE::TESEffectShader>(_cachedHighlightFormID);
                if (_cachedHighlightShader) {
                    ROCK_LOG_INFO(Hand, "Highlight shader loaded: FormID 0x{:08X}", _cachedHighlightFormID);
                } else {
                    ROCK_LOG_WARN(Hand, "Highlight shader FormID 0x{:08X} not found — disabling highlight", _cachedHighlightFormID);
                }
            }

            if (!_cachedHighlightShader)
                return;

            _highlightEffect = refr->ApplyEffectShader(_cachedHighlightShader, 60.0f);
            _highlightedRef = refr;
        }

        void stopSelectionHighlight()
        {
            if (_highlightEffect) {
                _highlightEffect->finished = true;
                _highlightEffect->effectShaderAge = 999.0f;
                _highlightEffect->lifetime = 0.0f;
                _highlightEffect = nullptr;
            }
            _highlightedRef = nullptr;
        }

        void updateDebugBasisVis(const RE::NiTransform& colliderTransform, const RE::NiPoint3& grabAnchorWorld, bool show, RE::NiNode* parentNode);

        void destroyDebugBasisVis();
    };
}
