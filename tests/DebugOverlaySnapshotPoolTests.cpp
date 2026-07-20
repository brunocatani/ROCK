#include <cassert>
#include <memory>

#include "physics-interaction/debug/DebugOverlaySnapshotPool.h"

namespace
{
    struct Snapshot
    {
        int value{ 0 };
    };
}

int main()
{
    using Pool = rock::debug_overlay_snapshot::SnapshotPool<Snapshot, 2>;
    Pool pool;

    auto first = pool.acquire();
    assert(first);
    assert(pool.size() == 1);
    first->value = 17;

    std::shared_ptr<const Snapshot> published = first;
    first.reset();

    auto second = pool.acquire();
    assert(second);
    assert(second.get() != published.get());
    assert(pool.size() == 2);

    std::shared_ptr<const Snapshot> consumed = second;
    second.reset();
    assert(!pool.acquire());

    published.reset();
    auto reused = pool.acquire();
    assert(reused);
    assert(reused->value == 17);
    assert(pool.size() == 2);

    consumed.reset();
    reused.reset();
    return 0;
}
