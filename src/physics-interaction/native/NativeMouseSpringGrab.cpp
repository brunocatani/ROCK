#include "physics-interaction/native/NativeMouseSpringGrab.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include <REL/Relocation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace rock
{
    namespace
    {
        constexpr std::size_t kMouseSpringActionSize = 0xE0;
        constexpr std::size_t kMouseSpringCinfoSize = 0xB0;
        constexpr std::size_t kCinfoInitialTransformOffset = 0x00;
        constexpr std::size_t kCinfoTargetTransformOffset = 0x40;
        constexpr std::size_t kCinfoTargetPositionOffset = 0x70;
        constexpr std::size_t kCinfoLocalGrabPointOffset = 0x80;
        constexpr std::size_t kCinfoBodyIdOffset = 0x90;
        constexpr std::size_t kCinfoAngularClampOffset = 0x94;
        constexpr std::size_t kCinfoLinearResponseOffset = 0x98;
        constexpr std::size_t kCinfoAngularResponseOffset = 0x9C;
        constexpr std::size_t kCinfoVelocityCarryOffset = 0xA0;
        constexpr std::size_t kCinfoBodyFlagsOffset = 0xA4;

        using MouseSpringCtor_t = void* (*)(void*, const void*);
        using MouseSpringUpdate_t = std::uint64_t (*)(void*, RE::hknpWorld*, float);
        using MouseSpringSetTargetPosition_t = void (*)(void*, const float*);
        using MouseSpringSetTargetTransform_t = void (*)(void*, const float*);

        struct alignas(16) MouseSpringCinfo
        {
            std::array<std::byte, kMouseSpringCinfoSize> bytes{};
        };

        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        bool isFiniteMatrix(const RE::NiMatrix3& matrix)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(matrix.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            return isFinitePoint(transform.translate) && isFiniteMatrix(transform.rotate) && std::isfinite(transform.scale) && std::fabs(transform.scale) > 0.000001f;
        }

        float readNativeSpringFloat(std::uintptr_t offset, float fallback)
        {
            REL::Relocation<float*> value{ REL::Offset(offset) };
            if (value.address() && std::isfinite(*value)) {
                return *value;
            }
            return fallback;
        }

        template <class T>
        void writeCinfo(MouseSpringCinfo& cinfo, std::size_t offset, const T& value)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            if (offset + sizeof(T) <= cinfo.bytes.size()) {
                std::memcpy(cinfo.bytes.data() + offset, &value, sizeof(T));
            }
        }

        void writeVector4(MouseSpringCinfo& cinfo, std::size_t offset, const float vector[4])
        {
            if (offset + sizeof(float) * 4 <= cinfo.bytes.size()) {
                std::memcpy(cinfo.bytes.data() + offset, vector, sizeof(float) * 4);
            }
        }

        RE::NiMatrix3 makeMouseSpringTargetRotation(const RE::NiMatrix3& targetBodyWorldRotation)
        {
            /*
             * This native action boundary uses the transposed Ni target basis.
             * Higher-level grab relations stay in root-flattened hand space and
             * native BODY object space. Keep the conversion isolated here so
             * hand/object math does not start carrying native packing details
             * through the rest of the grab pipeline.
             */
            return transform_math::transposeRotation(targetBodyWorldRotation);
        }

        void writeTransformColumns(MouseSpringCinfo& cinfo, std::size_t offset, const RE::NiMatrix3& rotation)
        {
            const auto columns = transform_math::niRowsToHavokColumns(makeMouseSpringTargetRotation(rotation));
            for (int column = 0; column < 3; ++column) {
                const float vector[4]{
                    columns.entry[column][0],
                    columns.entry[column][1],
                    columns.entry[column][2],
                    0.0f,
                };
                writeVector4(cinfo, offset + static_cast<std::size_t>(column) * 0x10, vector);
            }
        }

        void makeHavokPoint(const RE::NiPoint3& gamePoint, float outPoint[4])
        {
            const float scale = gameToHavokScale();
            outPoint[0] = gamePoint.x * scale;
            outPoint[1] = gamePoint.y * scale;
            outPoint[2] = gamePoint.z * scale;
            outPoint[3] = 0.0f;
        }

        MouseSpringCinfo makeCinfo(
            RE::hknpBodyId bodyId,
            const RE::NiTransform& targetBodyWorldGame,
            const RE::NiPoint3& targetPointWorldGame,
            const RE::NiPoint3& localGrabPointBodyGame,
            const native_mouse_spring_grab::Tuning& tuning)
        {
            MouseSpringCinfo cinfo{};

            float targetPointHavok[4]{};
            float localGrabPointHavok[4]{};
            makeHavokPoint(targetPointWorldGame, targetPointHavok);
            makeHavokPoint(localGrabPointBodyGame, localGrabPointHavok);

            writeTransformColumns(cinfo, kCinfoInitialTransformOffset, targetBodyWorldGame.rotate);
            writeVector4(cinfo, kCinfoInitialTransformOffset + 0x30, targetPointHavok);
            writeTransformColumns(cinfo, kCinfoTargetTransformOffset, targetBodyWorldGame.rotate);
            writeVector4(cinfo, kCinfoTargetPositionOffset, targetPointHavok);
            writeVector4(cinfo, kCinfoLocalGrabPointOffset, localGrabPointHavok);
            writeCinfo(cinfo, kCinfoBodyIdOffset, bodyId.value);

            const float angularClamp = readNativeSpringFloat(offsets::kData_MouseSpringAngularClampBase, 1.0f) *
                                       native_mouse_spring_grab::sanitizeTuningScale(tuning.angularClampScale);
            const float linearResponse = readNativeSpringFloat(offsets::kData_MouseSpringLinearResponseBase, 1.0f) *
                                         native_mouse_spring_grab::sanitizeTuningScale(tuning.linearResponseScale);
            const float angularResponse = readNativeSpringFloat(offsets::kData_MouseSpringAngularResponseBase, 1.0f) *
                                          native_mouse_spring_grab::sanitizeTuningScale(tuning.angularResponseScale);
            const float velocityCarry = readNativeSpringFloat(offsets::kData_MouseSpringVelocityCarryBase, 1.0f);
            writeCinfo(cinfo, kCinfoAngularClampOffset, angularClamp);
            writeCinfo(cinfo, kCinfoLinearResponseOffset, linearResponse);
            writeCinfo(cinfo, kCinfoAngularResponseOffset, angularResponse);
            writeCinfo(cinfo, kCinfoVelocityCarryOffset, velocityCarry);
            writeCinfo(cinfo, kCinfoBodyFlagsOffset, tuning.bodyFlags);

            return cinfo;
        }

        void writeTargetTransform(const RE::NiTransform& targetBodyWorldGame, float outTransform[12])
        {
            const auto columns = transform_math::niRowsToHavokColumns(makeMouseSpringTargetRotation(targetBodyWorldGame.rotate));
            for (int column = 0; column < 3; ++column) {
                outTransform[column * 4 + 0] = columns.entry[column][0];
                outTransform[column * 4 + 1] = columns.entry[column][1];
                outTransform[column * 4 + 2] = columns.entry[column][2];
                outTransform[column * 4 + 3] = 0.0f;
            }
        }
    }

    namespace native_mouse_spring_grab
    {
        RE::NiPoint3 computeTargetPointWorldGame(const RE::NiTransform& targetBodyWorldGame, const RE::NiPoint3& localGrabPointBodyGame)
        {
            return transform_math::localPointToWorld(targetBodyWorldGame, localGrabPointBodyGame);
        }
    }

    NativeMouseSpringGrab::~NativeMouseSpringGrab()
    {
        destroy(nullptr);
    }

    bool NativeMouseSpringGrab::create(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        const RE::NiTransform& initialTargetBodyWorldGame,
        const RE::NiPoint3& localGrabPointBodyGame,
        const native_mouse_spring_grab::Tuning& tuning)
    {
        const RE::NiPoint3 initialTargetPointWorldGame =
            native_mouse_spring_grab::computeTargetPointWorldGame(initialTargetBodyWorldGame, localGrabPointBodyGame);

        std::scoped_lock lock(_mutex);
        destroyUnlocked(world, true);

        if (!world || bodyId.value == native_mouse_spring_grab::kInvalidBodyId || !isFiniteTransform(initialTargetBodyWorldGame) ||
            !isFinitePoint(initialTargetPointWorldGame) ||
            !isFinitePoint(localGrabPointBodyGame)) {
            ROCK_LOG_WARN(Hand,
                "Native mouse-spring grab create rejected: world={:p} bodyId={} finiteTargetTransform={} finiteTargetPoint={} finitePivot={}",
                static_cast<void*>(world),
                bodyId.value,
                isFiniteTransform(initialTargetBodyWorldGame) ? "yes" : "no",
                isFinitePoint(initialTargetPointWorldGame) ? "yes" : "no",
                isFinitePoint(localGrabPointBodyGame) ? "yes" : "no");
            return false;
        }

        if (!havok_runtime::getBody(world, bodyId) || !havok_runtime::getBodyMotion(world, bodyId)) {
            ROCK_LOG_WARN(Hand, "Native mouse-spring grab create rejected unreadable dynamic bodyId={}", bodyId.value);
            return false;
        }

        void* memory = havok_runtime::allocateHavok(kMouseSpringActionSize);
        if (!memory) {
            ROCK_LOG_ERROR(Hand, "Native mouse-spring grab allocation failed size=0x{:X} bodyId={}", kMouseSpringActionSize, bodyId.value);
            return false;
        }

        const auto cinfo = makeCinfo(bodyId, initialTargetBodyWorldGame, initialTargetPointWorldGame, localGrabPointBodyGame, tuning);
        static REL::Relocation<MouseSpringCtor_t> constructAction{ REL::Offset(offsets::kFunc_MouseSpringAction_Ctor) };
        void* action = constructAction(memory, cinfo.bytes.data());
        if (!action) {
            havok_runtime::freeHavok(memory, kMouseSpringActionSize);
            ROCK_LOG_ERROR(Hand, "Native mouse-spring grab constructor returned null bodyId={}", bodyId.value);
            return false;
        }

        _action = action;
        _bodyId = bodyId;
        _targetBodyWorldGame = initialTargetBodyWorldGame;
        _localGrabPointBodyGame = localGrabPointBodyGame;
        _targetPointWorldGame = initialTargetPointWorldGame;
        _bodyFlags = tuning.bodyFlags;
        _lastFlushDeltaSeconds = 0.0f;
        _queuedTargets = 1;
        _flushedTargets = 0;
        _failedFlushes = 0;
        _hasTarget = true;

        if (_bodyFlags != 0) {
            havok_runtime::acquireBodyFlagLease(world, bodyId.value, _bodyFlags, 1, reinterpret_cast<std::uintptr_t>(this));
        }

        ROCK_LOG_DEBUG(Hand,
            "Native mouse-spring grab created action={:p} bodyId={} targetPoint=({:.2f},{:.2f},{:.2f}) pivotLocal=({:.2f},{:.2f},{:.2f}) flags=0x{:08X}",
            _action,
            _bodyId.value,
            _targetPointWorldGame.x,
            _targetPointWorldGame.y,
            _targetPointWorldGame.z,
            _localGrabPointBodyGame.x,
            _localGrabPointBodyGame.y,
            _localGrabPointBodyGame.z,
            _bodyFlags);
        return true;
    }

    void NativeMouseSpringGrab::destroy(RE::hknpWorld* world, bool restoreBodyFlags)
    {
        std::scoped_lock lock(_mutex);
        destroyUnlocked(world, restoreBodyFlags);
    }

    void NativeMouseSpringGrab::destroyUnlocked(RE::hknpWorld* world, bool restoreBodyFlags)
    {
        if (_action) {
            if (_bodyId.value != native_mouse_spring_grab::kInvalidBodyId && _bodyFlags != 0) {
                havok_runtime::releaseBodyFlagLease(
                    world,
                    _bodyId.value,
                    _bodyFlags,
                    1,
                    reinterpret_cast<std::uintptr_t>(this),
                    restoreBodyFlags);
            }
            havok_runtime::freeHavok(_action, kMouseSpringActionSize);
        }
        clearUnlocked();
    }

    void NativeMouseSpringGrab::clear()
    {
        std::scoped_lock lock(_mutex);
        clearUnlocked();
    }

    void NativeMouseSpringGrab::clearUnlocked()
    {
        _action = nullptr;
        _bodyId = RE::hknpBodyId{ native_mouse_spring_grab::kInvalidBodyId };
        _targetBodyWorldGame = RE::NiTransform{};
        _targetPointWorldGame = RE::NiPoint3{};
        _localGrabPointBodyGame = RE::NiPoint3{};
        _bodyFlags = 0;
        _lastFlushDeltaSeconds = 0.0f;
        _queuedTargets = 0;
        _flushedTargets = 0;
        _failedFlushes = 0;
        _hasTarget = false;
    }

    bool NativeMouseSpringGrab::isValidUnlocked() const
    {
        return _action != nullptr && _bodyId.value != native_mouse_spring_grab::kInvalidBodyId;
    }

    bool NativeMouseSpringGrab::isValid() const
    {
        std::scoped_lock lock(_mutex);
        return isValidUnlocked();
    }

    RE::hknpBodyId NativeMouseSpringGrab::bodyId() const
    {
        std::scoped_lock lock(_mutex);
        return _bodyId;
    }

    void* NativeMouseSpringGrab::action() const
    {
        std::scoped_lock lock(_mutex);
        return _action;
    }

    bool NativeMouseSpringGrab::hasTarget() const
    {
        std::scoped_lock lock(_mutex);
        return _hasTarget;
    }

    RE::NiPoint3 NativeMouseSpringGrab::targetPointWorldGame() const
    {
        std::scoped_lock lock(_mutex);
        return _targetPointWorldGame;
    }

    bool NativeMouseSpringGrab::queueTarget(const RE::NiTransform& targetBodyWorldGame)
    {
        std::scoped_lock lock(_mutex);
        const RE::NiPoint3 targetPointWorldGame =
            native_mouse_spring_grab::computeTargetPointWorldGame(targetBodyWorldGame, _localGrabPointBodyGame);
        if (!isValidUnlocked() || !isFiniteTransform(targetBodyWorldGame) || !isFinitePoint(targetPointWorldGame)) {
            return false;
        }

        _targetBodyWorldGame = targetBodyWorldGame;
        _targetPointWorldGame = targetPointWorldGame;
        _hasTarget = true;
        ++_queuedTargets;
        return true;
    }

    bool NativeMouseSpringGrab::flush(RE::hknpWorld* world, float deltaSeconds)
    {
        std::scoped_lock lock(_mutex);
        if (!isValidUnlocked() || !world || !_hasTarget) {
            return false;
        }

        const float driveDelta = havok_physics_timing::isUsableDelta(deltaSeconds) ? deltaSeconds : havok_physics_timing::kFallbackPhysicsDeltaSeconds;
        if (!havok_runtime::getBody(world, _bodyId) || !havok_runtime::getBodyMotion(world, _bodyId)) {
            ++_failedFlushes;
            return false;
        }

        alignas(16) float targetPosition[4]{};
        alignas(16) float targetTransform[12]{};
        makeHavokPoint(_targetPointWorldGame, targetPosition);
        writeTargetTransform(_targetBodyWorldGame, targetTransform);

        static REL::Relocation<MouseSpringSetTargetPosition_t> setTargetPosition{ REL::Offset(offsets::kFunc_MouseSpringAction_SetTargetPosition) };
        static REL::Relocation<MouseSpringSetTargetTransform_t> setTargetTransform{ REL::Offset(offsets::kFunc_MouseSpringAction_SetTargetTransform) };
        static REL::Relocation<MouseSpringUpdate_t> updateAction{ REL::Offset(offsets::kFunc_MouseSpringAction_Update) };

        setTargetPosition(_action, targetPosition);
        setTargetTransform(_action, targetTransform);
        updateAction(_action, world, driveDelta);

        _lastFlushDeltaSeconds = driveDelta;
        ++_flushedTargets;
        return true;
    }

    native_mouse_spring_grab::DebugState NativeMouseSpringGrab::debugState() const
    {
        std::scoped_lock lock(_mutex);
        native_mouse_spring_grab::DebugState state{};
        state.action = _action;
        state.bodyId = _bodyId;
        state.targetBodyWorldGame = _targetBodyWorldGame;
        state.targetPointWorldGame = _targetPointWorldGame;
        state.localGrabPointBodyGame = _localGrabPointBodyGame;
        state.lastFlushDeltaSeconds = _lastFlushDeltaSeconds;
        state.queuedTargets = _queuedTargets;
        state.flushedTargets = _flushedTargets;
        state.failedFlushes = _failedFlushes;
        state.hasTarget = _hasTarget;
        return state;
    }
}
