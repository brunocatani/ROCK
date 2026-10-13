#include "physics-interaction/debug/DebugOverlayShapePipeline.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

#include <Windows.h>

namespace rock::debug_overlay_shape
{
    namespace
    {
        struct CacheEntry
        {
            CacheState state{ CacheState::Pending };
            std::shared_ptr<const RegisteredShape> shape;
            std::uint64_t generation{ 0 };
            std::uint64_t lastUseSerial{ 0 };
        };

        struct MeshJob
        {
            ShapeKey key{};
            ShapeRecipe recipe{};
            std::uint64_t generation{ 0 };
            std::uint32_t maxCompletedJobs{ 0 };
        };

        struct CompletedMesh
        {
            ShapeKey key{};
            BuiltShape built{};
            std::uint64_t generation{ 0 };
        };

        bool registerShape(const MeshSink& sink, std::uint64_t meshId, const BuiltShape& built,
            RegisteredShape& output) noexcept
        {
            const auto& mesh = built.mesh;
            if (!sink.registerMesh || !sink.releaseMesh || !mesh.valid || mesh.vertices.empty() ||
                mesh.indices.empty() || mesh.indices.size() > (std::numeric_limits<std::uint32_t>::max)() ||
                !sink.registerMesh(meshId, mesh)) {
                return false;
            }
            output.meshId = meshId;
            output.release = sink.releaseMesh;
            output.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
            output.decodeMode = built.decodeMode;
            output.shapeType = built.shapeType;
            output.approximateBytes = approximateMeshBytes(mesh);
            return true;
        }
    }

    struct ShapePipeline::Impl
    {
        using CacheMap = std::unordered_map<ShapeKey, CacheEntry, ShapeKeyHash>;

        CacheMap cache;
        std::deque<MeshJob> pendingJobs;
        std::deque<CompletedMesh> completedJobs;
        mutable std::mutex mutex;
        std::condition_variable queueCv;
        std::thread worker;
        std::uint64_t generation{ 1 };
        std::uint64_t useSerial{ 0 };
        std::uint64_t evictions{ 0 };
        std::uint64_t staleResultDrops{ 0 };
        std::uint64_t completedQueueDrops{ 0 };
        std::size_t reservedJobs{ 0 };
        std::size_t activeJobs{ 0 };
        std::size_t approximateBytes{ 0 };
        std::uint64_t nextMeshId{ kFirstPipelineMeshId };
        bool running{ false };
        bool stopRequested{ false };

        bool evictOneUnlocked(const ShapeKey* excluded)
        {
            auto victim = cache.end();
            for (auto iterator = cache.begin(); iterator != cache.end(); ++iterator) {
                if (iterator->second.state == CacheState::Pending ||
                    (excluded && iterator->first == *excluded)) {
                    continue;
                }
                if (victim == cache.end() || iterator->second.lastUseSerial < victim->second.lastUseSerial) {
                    victim = iterator;
                }
            }
            if (victim == cache.end()) {
                return false;
            }

            if (victim->second.shape) {
                approximateBytes -= (std::min)(approximateBytes, victim->second.shape->approximateBytes);
            }
            cache.erase(victim);
            ++evictions;
            return true;
        }

        bool ensureEntryCapacityUnlocked(std::uint32_t maxEntries)
        {
            if (maxEntries == 0) {
                return false;
            }
            while (cache.size() >= maxEntries) {
                if (!evictOneUnlocked(nullptr)) {
                    return false;
                }
            }
            return true;
        }

        bool ensureByteCapacityUnlocked(
            std::size_t incomingBytes,
            std::size_t maxBytes,
            const ShapeKey& excluded)
        {
            if (incomingBytes > maxBytes) {
                return false;
            }
            while (approximateBytes > maxBytes - incomingBytes) {
                if (!evictOneUnlocked(&excluded)) {
                    return false;
                }
            }
            return true;
        }

        void workerMain()
        {
            (void)::SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);

            for (;;) {
                MeshJob job{};
                {
                    std::unique_lock lock(mutex);
                    queueCv.wait(lock, [&] { return stopRequested || !pendingJobs.empty(); });
                    if (stopRequested) {
                        return;
                    }
                    job = std::move(pendingJobs.front());
                    pendingJobs.pop_front();
                    ++activeJobs;
                }

                CompletedMesh completed{};
                completed.key = job.key;
                completed.generation = job.generation;
                try {
                    completed.built = buildMeshFromRecipe(job.recipe);
                } catch (...) {
                    completed.built = {};
                }

                {
                    std::scoped_lock lock(mutex);
                    --activeJobs;
                    const auto entry = cache.find(job.key);
                    if (!running || stopRequested || job.generation != generation ||
                        entry == cache.end() || entry->second.state != CacheState::Pending ||
                        entry->second.generation != job.generation) {
                        ++staleResultDrops;
                        continue;
                    }
                    if (completedJobs.size() >= job.maxCompletedJobs) {
                        entry->second.state = CacheState::Unsupported;
                        entry->second.lastUseSerial = ++useSerial;
                        ++completedQueueDrops;
                        continue;
                    }
                    try {
                        completedJobs.push_back(std::move(completed));
                    } catch (...) {
                        entry->second.state = CacheState::Unsupported;
                        entry->second.lastUseSerial = ++useSerial;
                        ++completedQueueDrops;
                    }
                }
            }
        }

        void clearStateUnlocked(CacheMap& retiredCache)
        {
            retiredCache.swap(cache);
            pendingJobs.clear();
            completedJobs.clear();
            reservedJobs = 0;
            approximateBytes = 0;
        }
    };

    ShapePipeline::ShapePipeline() : _impl(std::make_unique<Impl>()) {}

    ShapePipeline::~ShapePipeline()
    {
        shutdown();
    }

    bool ShapePipeline::initialize()
    {
        std::scoped_lock lock(_impl->mutex);
        if (_impl->running) {
            return true;
        }

        _impl->stopRequested = false;
        try {
            _impl->worker = std::thread([implementation = _impl.get()] { implementation->workerMain(); });
        } catch (...) {
            return false;
        }
        _impl->running = true;
        return true;
    }

    void ShapePipeline::shutdown() noexcept
    {
        std::thread worker;
        Impl::CacheMap retiredCache;
        {
            std::scoped_lock lock(_impl->mutex);
            if (!_impl->running && !_impl->worker.joinable()) {
                _impl->clearStateUnlocked(retiredCache);
                return;
            }
            _impl->running = false;
            _impl->stopRequested = true;
            ++_impl->generation;
            _impl->clearStateUnlocked(retiredCache);
            worker = std::move(_impl->worker);
        }
        _impl->queueCv.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
        {
            std::scoped_lock lock(_impl->mutex);
            _impl->activeJobs = 0;
            _impl->stopRequested = false;
        }
    }

    void ShapePipeline::invalidate() noexcept
    {
        Impl::CacheMap retiredCache;
        {
            std::scoped_lock lock(_impl->mutex);
            ++_impl->generation;
            _impl->clearStateUnlocked(retiredCache);
        }
    }

    void ShapePipeline::trim(const PipelineLimits& limits) noexcept
    {
        std::scoped_lock lock(_impl->mutex);
        while (_impl->cache.size() > limits.maxCacheEntries && _impl->evictOneUnlocked(nullptr)) {
        }
        while (_impl->approximateBytes > limits.maxBytes && _impl->evictOneUnlocked(nullptr)) {
        }
    }

    ReserveResult ShapePipeline::reserve(const ShapeKey& key, const PipelineLimits& limits)
    {
        ReserveResult result{};
        try {
            std::scoped_lock lock(_impl->mutex);
            if (!_impl->running || _impl->stopRequested) {
                result.status = ReserveStatus::NotRunning;
                return result;
            }

            if (const auto existing = _impl->cache.find(key); existing != _impl->cache.end()) {
                existing->second.lastUseSerial = ++_impl->useSerial;
                switch (existing->second.state) {
                case CacheState::Ready:
                    result.status = ReserveStatus::Ready;
                    break;
                case CacheState::Pending:
                    result.status = ReserveStatus::Pending;
                    break;
                case CacheState::Unsupported:
                    result.status = ReserveStatus::Unsupported;
                    break;
                case CacheState::Missing:
                default:
                    result.status = ReserveStatus::CacheFull;
                    break;
                }
                return result;
            }

            const std::size_t backlog = _impl->reservedJobs + _impl->pendingJobs.size() +
                                        _impl->activeJobs + _impl->completedJobs.size();
            if (limits.maxQueuedJobs == 0 || backlog >= limits.maxQueuedJobs || limits.maxCompletedJobs == 0) {
                result.status = ReserveStatus::QueueFull;
                return result;
            }
            if (!_impl->ensureEntryCapacityUnlocked(limits.maxCacheEntries)) {
                result.status = ReserveStatus::CacheFull;
                return result;
            }

            const std::uint64_t generation = _impl->generation;
            _impl->cache.emplace(key, CacheEntry{ CacheState::Pending, {}, generation, ++_impl->useSerial });
            ++_impl->reservedJobs;
            result.status = ReserveStatus::Reserved;
            result.reservation = ShapeReservation{ key, generation, limits.maxCompletedJobs, true };
        } catch (...) {
            result = {};
            result.status = ReserveStatus::CacheFull;
        }
        return result;
    }

    bool ShapePipeline::submit(ShapeReservation reservation, ShapeRecipe recipe)
    {
        if (!reservation.valid) {
            return false;
        }

        bool queued = false;
        {
            std::scoped_lock lock(_impl->mutex);
            const auto entry = _impl->cache.find(reservation.key);
            if (!_impl->running || _impl->stopRequested || reservation.generation != _impl->generation ||
                entry == _impl->cache.end() || entry->second.state != CacheState::Pending ||
                entry->second.generation != reservation.generation) {
                return false;
            }
            if (!recipe.valid) {
                if (_impl->reservedJobs > 0) {
                    --_impl->reservedJobs;
                }
                entry->second.state = CacheState::Unsupported;
                entry->second.lastUseSerial = ++_impl->useSerial;
                return false;
            }

            try {
                _impl->pendingJobs.push_back(MeshJob{
                    reservation.key,
                    std::move(recipe),
                    reservation.generation,
                    reservation.maxCompletedJobs
                });
            } catch (...) {
                if (_impl->reservedJobs > 0) {
                    --_impl->reservedJobs;
                }
                entry->second.state = CacheState::Unsupported;
                entry->second.lastUseSerial = ++_impl->useSerial;
                return false;
            }
            if (_impl->reservedJobs > 0) {
                --_impl->reservedJobs;
            }
            queued = true;
        }
        if (queued) {
            _impl->queueCv.notify_one();
        }
        return queued;
    }

    void ShapePipeline::markUnsupported(ShapeReservation reservation) noexcept
    {
        if (!reservation.valid) {
            return;
        }
        std::scoped_lock lock(_impl->mutex);
        const auto entry = _impl->cache.find(reservation.key);
        if (reservation.generation == _impl->generation && entry != _impl->cache.end() &&
            entry->second.state == CacheState::Pending && entry->second.generation == reservation.generation) {
            if (_impl->reservedJobs > 0) {
                --_impl->reservedJobs;
            }
            entry->second.state = CacheState::Unsupported;
            entry->second.lastUseSerial = ++_impl->useSerial;
        }
    }

    LookupResult ShapePipeline::lookup(const ShapeKey& key)
    {
        std::scoped_lock lock(_impl->mutex);
        const auto entry = _impl->cache.find(key);
        if (entry == _impl->cache.end()) {
            return {};
        }
        entry->second.lastUseSerial = ++_impl->useSerial;
        return LookupResult{ entry->second.state, entry->second.shape };
    }

    UploadResult ShapePipeline::processCompletedUploads(
        const MeshSink& sink, std::uint32_t maxUploads, const PipelineLimits& limits)
    {
        UploadResult result{};
        if (!sink.registerMesh || !sink.releaseMesh || maxUploads == 0) {
            return result;
        }

        while (result.processed < maxUploads) {
            CompletedMesh completed{};
            {
                std::scoped_lock lock(_impl->mutex);
                if (_impl->completedJobs.empty()) {
                    break;
                }
                completed = std::move(_impl->completedJobs.front());
                _impl->completedJobs.pop_front();
            }
            ++result.processed;

            try {
                const std::size_t incomingBytes = approximateMeshBytes(completed.built.mesh);
                std::uint64_t meshId = 0;
                {
                    std::scoped_lock lock(_impl->mutex);
                    const auto entry = _impl->cache.find(completed.key);
                    if (!_impl->running || completed.generation != _impl->generation ||
                        entry == _impl->cache.end() || entry->second.state != CacheState::Pending ||
                        entry->second.generation != completed.generation) {
                        ++_impl->staleResultDrops;
                        ++result.stale;
                        continue;
                    }
                    if (!completed.built.mesh.valid || completed.built.decodeMode == debug_overlay_policy::ShapeDecodeMode::Unsupported) {
                        entry->second.state = CacheState::Unsupported;
                        entry->second.lastUseSerial = ++_impl->useSerial;
                        ++result.unsupported;
                        continue;
                    }
                    if (!_impl->ensureByteCapacityUnlocked(incomingBytes, limits.maxBytes, completed.key)) {
                        entry->second.state = CacheState::Unsupported;
                        entry->second.lastUseSerial = ++_impl->useSerial;
                        ++result.failed;
                        continue;
                    }
                    meshId = _impl->nextMeshId++;
                }

                auto uploaded = std::make_shared<RegisteredShape>();
                if (!registerShape(sink, meshId, completed.built, *uploaded)) {
                    std::scoped_lock lock(_impl->mutex);
                    const auto entry = _impl->cache.find(completed.key);
                    if (completed.generation == _impl->generation && entry != _impl->cache.end() &&
                        entry->second.state == CacheState::Pending && entry->second.generation == completed.generation) {
                        entry->second.state = CacheState::Unsupported;
                        entry->second.lastUseSerial = ++_impl->useSerial;
                    }
                    ++result.failed;
                    continue;
                }

                {
                    std::scoped_lock lock(_impl->mutex);
                    const auto entry = _impl->cache.find(completed.key);
                    if (!_impl->running || completed.generation != _impl->generation ||
                        entry == _impl->cache.end() || entry->second.state != CacheState::Pending ||
                        entry->second.generation != completed.generation) {
                        ++_impl->staleResultDrops;
                        ++result.stale;
                        continue;
                    }
                    entry->second.state = CacheState::Ready;
                    entry->second.shape = std::move(uploaded);
                    entry->second.lastUseSerial = ++_impl->useSerial;
                    _impl->approximateBytes += incomingBytes;
                    ++result.uploaded;
                }
            } catch (...) {
                std::scoped_lock lock(_impl->mutex);
                const auto entry = _impl->cache.find(completed.key);
                if (completed.generation == _impl->generation && entry != _impl->cache.end() &&
                    entry->second.state == CacheState::Pending && entry->second.generation == completed.generation) {
                    entry->second.state = CacheState::Unsupported;
                    entry->second.lastUseSerial = ++_impl->useSerial;
                }
                ++result.failed;
            }
        }

        trim(limits);
        return result;
    }

    PipelineStats ShapePipeline::stats() const noexcept
    {
        PipelineStats result{};
        std::scoped_lock lock(_impl->mutex);
        result.entries = _impl->cache.size();
        for (const auto& [_, entry] : _impl->cache) {
            switch (entry.state) {
            case CacheState::Ready:
                ++result.ready;
                break;
            case CacheState::Pending:
                ++result.pending;
                break;
            case CacheState::Unsupported:
                ++result.unsupported;
                break;
            case CacheState::Missing:
            default:
                break;
            }
        }
        result.reservedJobs = _impl->reservedJobs;
        result.queuedJobs = _impl->pendingJobs.size();
        result.activeJobs = _impl->activeJobs;
        result.completedJobs = _impl->completedJobs.size();
        result.approximateBytes = _impl->approximateBytes;
        result.generation = _impl->generation;
        result.evictions = _impl->evictions;
        result.staleResultDrops = _impl->staleResultDrops;
        result.completedQueueDrops = _impl->completedQueueDrops;
        result.running = _impl->running;
        return result;
    }
}
