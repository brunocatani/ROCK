#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

namespace rock
{
    /*
     * Main-thread structural mutations must not overlap native physics-step
     * callbacks that traverse ROCK-owned body banks. The gate starts paused;
     * registration publishes the callback owner and opens it only after the
     * next-step listener has been installed.
     *
     * Contract: pauseForMutation() is used by the single game/update thread.
     * Callback leases may be acquired concurrently by native physics threads.
     * No callback may attempt a structural mutation because waiting for its own
     * lease would deadlock.
     */
    class PhysicsCallbackQuiescenceGate
    {
    public:
        class CallbackLease
        {
        public:
            CallbackLease() = default;
            ~CallbackLease() { release(); }

            CallbackLease(const CallbackLease&) = delete;
            CallbackLease& operator=(const CallbackLease&) = delete;

            CallbackLease(CallbackLease&& other) noexcept : _gate(std::exchange(other._gate, nullptr)) {}
            CallbackLease& operator=(CallbackLease&& other) noexcept
            {
                if (this != &other) {
                    release();
                    _gate = std::exchange(other._gate, nullptr);
                }
                return *this;
            }

            [[nodiscard]] explicit operator bool() const { return _gate != nullptr; }

        private:
            friend class PhysicsCallbackQuiescenceGate;
            explicit CallbackLease(PhysicsCallbackQuiescenceGate* gate) : _gate(gate) {}

            void release()
            {
                if (_gate) {
                    _gate->leaveCallback();
                    _gate = nullptr;
                }
            }

            PhysicsCallbackQuiescenceGate* _gate = nullptr;
        };

        class MutationLease
        {
        public:
            MutationLease() = default;
            ~MutationLease()
            {
                if (_resumeOnExit && _gate) {
                    _gate->resumeCallbacks();
                }
            }

            MutationLease(const MutationLease&) = delete;
            MutationLease& operator=(const MutationLease&) = delete;

            MutationLease(MutationLease&& other) noexcept :
                _gate(std::exchange(other._gate, nullptr)),
                _resumeOnExit(std::exchange(other._resumeOnExit, false))
            {}
            MutationLease& operator=(MutationLease&& other) noexcept
            {
                if (this != &other) {
                    if (_resumeOnExit && _gate) {
                        _gate->resumeCallbacks();
                    }
                    _gate = std::exchange(other._gate, nullptr);
                    _resumeOnExit = std::exchange(other._resumeOnExit, false);
                }
                return *this;
            }

        private:
            friend class PhysicsCallbackQuiescenceGate;
            MutationLease(PhysicsCallbackQuiescenceGate* gate, bool resumeOnExit) :
                _gate(gate),
                _resumeOnExit(resumeOnExit)
            {}

            PhysicsCallbackQuiescenceGate* _gate = nullptr;
            bool _resumeOnExit = false;
        };

        PhysicsCallbackQuiescenceGate() = default;
        PhysicsCallbackQuiescenceGate(const PhysicsCallbackQuiescenceGate&) = delete;
        PhysicsCallbackQuiescenceGate& operator=(const PhysicsCallbackQuiescenceGate&) = delete;

        [[nodiscard]] CallbackLease tryEnterCallback()
        {
            if (_callbacksPaused.load(std::memory_order_acquire)) {
                return {};
            }

            _callbacksInFlight.fetch_add(1, std::memory_order_acq_rel);
            if (_callbacksPaused.load(std::memory_order_acquire)) {
                leaveCallback();
                return {};
            }
            return CallbackLease(this);
        }

        [[nodiscard]] MutationLease pauseForMutation()
        {
            const bool wasPaused = _callbacksPaused.exchange(true, std::memory_order_acq_rel);
            waitForCallbacks();
            return MutationLease(this, !wasPaused);
        }

        void pauseAndWait()
        {
            _callbacksPaused.store(true, std::memory_order_release);
            waitForCallbacks();
        }

        void resumeCallbacks()
        {
            _callbacksPaused.store(false, std::memory_order_release);
        }

        [[nodiscard]] bool callbacksPaused() const
        {
            return _callbacksPaused.load(std::memory_order_acquire);
        }

    private:
        void waitForCallbacks()
        {
            std::uint32_t active = _callbacksInFlight.load(std::memory_order_acquire);
            while (active != 0) {
                _callbacksInFlight.wait(active, std::memory_order_acquire);
                active = _callbacksInFlight.load(std::memory_order_acquire);
            }
        }

        void leaveCallback()
        {
            if (_callbacksInFlight.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                _callbacksInFlight.notify_all();
            }
        }

        std::atomic<bool> _callbacksPaused{ true };
        std::atomic<std::uint32_t> _callbacksInFlight{ 0 };
    };
}
