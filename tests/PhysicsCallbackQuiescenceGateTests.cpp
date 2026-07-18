#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

using rock::PhysicsCallbackQuiescenceGate;

int main()
{
    using namespace std::chrono_literals;

    PhysicsCallbackQuiescenceGate gate;
    assert(gate.callbacksPaused());
    assert(!gate.tryEnterCallback());

    gate.resumeCallbacks();
    auto activeCallback = gate.tryEnterCallback();
    assert(activeCallback);

    std::atomic<bool> mutationEntered{ false };
    std::atomic<bool> releaseMutation{ false };
    std::thread mutationThread([&] {
        auto mutation = gate.pauseForMutation();
        mutationEntered.store(true, std::memory_order_release);
        mutationEntered.notify_all();
        while (!releaseMutation.load(std::memory_order_acquire)) {
            releaseMutation.wait(false, std::memory_order_acquire);
        }
    });

    std::this_thread::sleep_for(10ms);
    assert(!mutationEntered.load(std::memory_order_acquire));
    activeCallback = {};
    mutationEntered.wait(false, std::memory_order_acquire);
    assert(mutationEntered.load(std::memory_order_acquire));
    assert(!gate.tryEnterCallback());

    releaseMutation.store(true, std::memory_order_release);
    releaseMutation.notify_all();
    mutationThread.join();
    assert(!gate.callbacksPaused());
    assert(gate.tryEnterCallback());

    {
        auto outerMutation = gate.pauseForMutation();
        assert(gate.callbacksPaused());
        {
            auto nestedMutation = gate.pauseForMutation();
            assert(gate.callbacksPaused());
        }
        assert(gate.callbacksPaused());
    }
    assert(!gate.callbacksPaused());

    gate.pauseAndWait();
    assert(gate.callbacksPaused());
    assert(!gate.tryEnterCallback());
    return 0;
}
