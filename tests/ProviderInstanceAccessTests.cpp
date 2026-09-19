#include "api/ProviderInstanceAccess.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <memory>

namespace rock
{
    // This test needs only the lifetime of the published object, not the engine.
    class PhysicsInteraction
    {
    public:
        explicit PhysicsInteraction(std::atomic<bool>& destroyed) : _destroyed(destroyed) {}
        ~PhysicsInteraction() { _destroyed = true; }
        int value = 42;
    private:
        std::atomic<bool>& _destroyed;
    };
}

int main()
{
    using namespace std::chrono_literals;
    rock::provider::ProviderInstanceAccess access;
    assert(!access.borrow().get());
    std::atomic<bool> destroyed{false};
    auto instance = std::make_unique<rock::PhysicsInteraction>(destroyed);
    access.publish(instance.get());
    std::future<void> retire;
    {
        const auto read = access.borrow();
        assert(read.get() == instance.get());
        std::promise<void> started;
        auto ready = started.get_future();
        retire = std::async(std::launch::async, [&] {
            started.set_value();
            access.publish(nullptr);
            instance.reset();
        });
        ready.wait();
        assert(retire.wait_for(20ms) == std::future_status::timeout);
        assert(!destroyed && read.get()->value == 42);
    }
    retire.get();
    assert(destroyed && !access.borrow().get());

    // A replacement instance becomes available only through a new publication.
    destroyed = false;
    instance = std::make_unique<rock::PhysicsInteraction>(destroyed);
    access.publish(instance.get());
    assert(access.borrow().get() == instance.get());
    access.publish(nullptr);
    instance.reset();
    assert(destroyed);
}
