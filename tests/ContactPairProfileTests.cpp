#include "physics-interaction/performance/ContactPairProfile.h"

#include <cassert>
#include <barrier>
#include <thread>
#include <vector>

using namespace rock::performance_profiler;

int main()
{
    ContactPairTable<16> table;
    ContactPair pair{ .world = 123, .bodyA = 700, .bodyB = 42, .shapeA = 0x12000000,
        .shapeB = 0x34000000, .layerA = 51, .layerB = 5 };
    assert(table.record(pair, ContactStage::Manifold, 7, 1));
    std::swap(pair.bodyA, pair.bodyB);
    std::swap(pair.shapeA, pair.shapeB);
    std::swap(pair.layerA, pair.layerB);
    assert(table.record(pair, ContactStage::Manifold, 8, 1));
    assert(table.record(pair, ContactStage::PlayerMeleeDropped, 6, 1));
    auto snapshot = table.take(1);
    assert(snapshot.size == 1 && snapshot.dropped == 0 && snapshot.carriedBuckets == 0);
    const auto& row = snapshot.entries[0];
    assert(row.pair.bodyA == 42 && row.pair.bodyB == 700);
    assert(row.pair.shapeA == 0x34000000 && row.pair.shapeB == 0x12000000);
    assert(row.pair.layerA == 5 && row.pair.layerB == 51 && !row.layerChanged);
    assert(row.events == 3 && row.counts[static_cast<std::size_t>(ContactStage::Manifold)] == 2);
    assert(row.counts[static_cast<std::size_t>(ContactStage::PlayerMeleeDropped)] == 1);
    assert(row.firstFrame == 6 && row.lastFrame == 8);
    assert(table.take(1).size == 0);

    // Unknown batch metadata is enriched by the later contact record.
    pair.shapeA = pair.shapeB = kUnknownContactDetail;
    pair.layerA = pair.layerB = kUnknownContactDetail;
    assert(table.record(pair, ContactStage::SimulationInput, 9, 1));
    pair.layerA = 5; pair.layerB = 51;
    assert(table.record(pair, ContactStage::PlayerMeleeDropped, 10, 1));
    pair.layerB = 44;
    assert(table.record(pair, ContactStage::Manifold, 11, 1));
    snapshot = table.take(1);
    assert(snapshot.size == 1 && snapshot.entries[0].layerChanged);
    assert(snapshot.entries[0].pair.layerA == 5 && snapshot.entries[0].pair.layerB == 44);

    // Generation changes reject in-flight records from disabled/reset settings.
    assert(table.record(pair, ContactStage::Impulse, 12, 1));
    assert(table.take(2).size == 0);
    assert(!table.record(pair, ContactStage::Impulse, 12, 1));
    assert(table.record(pair, ContactStage::Impulse, 1, 2));
    snapshot = table.take(2);
    assert(snapshot.size == 1 && snapshot.entries[0].events == 1 && snapshot.entries[0].firstFrame == 1);

    ContactPairTable<4> bounded;
    for (std::uint32_t i = 0; i < 12; ++i) {
        pair.bodyA = i;
        assert(bounded.record(pair, ContactStage::Manifold, 1, 3) == (i < 4));
    }
    auto full = bounded.take(3);
    assert(full.size == 4 && full.dropped == 8);
    assert(bounded.record(pair, ContactStage::Manifold, 2, 3));
    pair.world = 0;
    assert(!bounded.record(pair, ContactStage::Manifold, 2, 3));
    pair.world = 123; pair.bodyA = pair.bodyB;
    assert(!bounded.record(pair, ContactStage::Manifold, 2, 3));
    pair.bodyA = kUnknownContactDetail;
    assert(!bounded.record(pair, ContactStage::Manifold, 2, 3));
    pair.bodyA = 42;
    assert(!bounded.record(pair, ContactStage::Count, 2, 3));

    // Writers never spin: every contended write is either counted or explicitly
    // dropped. Concurrent frame drains may carry busy buckets but lose no counts.
    ContactPairTable<16> concurrent;
    constexpr int threads = 4, writes = 10000;
    std::barrier start(threads + 1);
    std::atomic<int> finished{0};
    std::vector<std::thread> workers;
    for (int i = 0; i < threads; ++i) workers.emplace_back([&] {
        start.arrive_and_wait();
        for (int n = 0; n < writes; ++n) concurrent.record(pair, ContactStage::Manifold, n, 5);
        finished.fetch_add(1);
    });
    start.arrive_and_wait();
    std::uint64_t accounted = 0;
    const auto drain = [&] {
        const auto batch = concurrent.take(5);
        accounted += batch.dropped;
        for (std::size_t i = 0; i < batch.size; ++i) accounted += batch.entries[i].events;
    };
    while (finished.load() != threads) { drain(); std::this_thread::yield(); }
    for (auto& worker : workers) worker.join();
    drain();
    assert(accounted == threads * writes);
}
