#pragma once

#include <mutex>
#include <shared_mutex>

namespace rock
{
    class PhysicsInteraction;
}

namespace rock::provider
{
    // The game thread owns the instance. API readers borrow it only for one
    // bounded call; publishing nullptr drains those readers before deletion.
    // Read admission never waits for the writer; teardown returns unavailable.
    // No consumer callback or nested borrow may run while a Read is held.
    class ProviderInstanceAccess
    {
    public:
        class Read
        {
        public:
            explicit Read(const ProviderInstanceAccess& access) :
                _lock(access._mutex, std::try_to_lock),
                _instance(_lock.owns_lock() ? access._instance : nullptr)
            {}

            [[nodiscard]] PhysicsInteraction* get() const noexcept { return _instance; }

        private:
            std::shared_lock<std::shared_mutex> _lock;
            PhysicsInteraction* _instance;
        };

        [[nodiscard]] Read borrow() const { return Read(*this); }

        void publish(PhysicsInteraction* instance)
        {
            std::unique_lock lock(_mutex);
            _instance = instance;
        }

    private:
        mutable std::shared_mutex _mutex;
        PhysicsInteraction* _instance{ nullptr }; // Owned by ROCKMain.
    };
}
