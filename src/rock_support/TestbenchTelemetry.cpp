#include "rock_support/TestbenchTelemetry.h"

#include "physics-interaction/PhysicsLog.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <mutex>

namespace rock::testbench_telemetry
{
    namespace
    {
        constexpr wchar_t kSectionName[] = L"Local\\RPS_Telemetry_ROCK";
        constexpr std::uint32_t kMagic = 0x54535052; // "RPST", shared with the bench's input ring
        constexpr std::uint32_t kSchemaVersion = 2;
        constexpr std::uint32_t kRecordSize = 512;
        constexpr std::uint32_t kCapacity = 16384; // about 3 minutes of frames at 90 Hz plus events
        constexpr std::size_t kEventQueueCapacity = 512;

        enum class RecordType : std::uint16_t
        {
            Frame = 1,
            Event = 2,
        };

        enum class EventType : std::uint16_t
        {
            Haptic = 13,
            InputGate = 17,
        };

#pragma pack(push, 1)
        struct RingHeader
        {
            std::uint32_t magic;
            std::uint32_t schemaVersion;
            std::uint32_t recordSize;
            std::uint32_t capacity;
            std::uint64_t writeIndex;
            std::uint64_t qpcFrequency;
            std::uint64_t producerStartQpc;
            std::uint32_t producerPid;
            std::uint32_t flags; // bit 0: producer alive
            std::uint8_t reserved[16];
        };
        static_assert(sizeof(RingHeader) == 64);

        struct RecordPrefix
        {
            std::uint16_t type;
            std::uint16_t payloadSize;
            std::uint32_t frameIndex;
            std::uint64_t qpc;
            float gameTimeSeconds; // sum of frame deltas since the section was opened
            float deltaSeconds;
        };
        static_assert(sizeof(RecordPrefix) == 24);

        struct HandRecord
        {
            float position[3];
            float rotation[9];
            float velocity[3];
            std::uint8_t hasVelocity;
            std::uint8_t disabled;
            std::uint16_t pad;
        };
        static_assert(sizeof(HandRecord) == 64);

        struct NodeRecord
        {
            float position[3];
            float rotation[9];
        };
        static_assert(sizeof(NodeRecord) == 48);

        struct InputRecord
        {
            std::uint64_t rawPressed;      // latest raw OpenVR state
            std::uint64_t gamePressed;     // latest state handed to the game
            std::uint64_t rawPressedSeen;  // OR of every raw read since the previous frame
            std::uint64_t gamePressedSeen; // OR of every game-facing read since the previous frame
            float rawTrigger;
            float gameTrigger;
            std::uint32_t blankReasons;  // OR since the previous frame
            std::uint32_t reads;         // game-facing reads since the previous frame
            std::uint32_t bypassedReads; // raw reads by RPS UI / configurator since the previous frame
            std::uint32_t pad;
        };
        static_assert(sizeof(InputRecord) == 56);

        struct FrameRecord
        {
            std::uint32_t flags;
            std::uint32_t droppedEvents;
            std::uint32_t failedReads;     // controller-state reads OpenVR answered false, since the previous frame
            std::uint32_t noHandRoleReads; // reads for a device in neither hand role, since the previous frame
            float hmdPosition[3];
            float hmdForward[3];
            HandRecord hand[2];
            InputRecord input[2];
            NodeRecord room;
            float roomScale;
            NodeRecord hmdNode;
            NodeRecord wand[2]; // left (secondary), right (primary)
        };
        static_assert(sizeof(FrameRecord) == 476);

        struct EventRecord
        {
            std::uint16_t eventType;
            std::uint8_t hand; // 0 left, 1 right, 255 n/a
            std::uint8_t detail;
            std::uint32_t formId;
            std::uint32_t targetFormId;
            float value[4];
            std::uint32_t aux[2];
            char text[48];
        };
        static_assert(sizeof(EventRecord) == 84);
#pragma pack(pop)
        static_assert(sizeof(RecordPrefix) + sizeof(FrameRecord) <= kRecordSize);
        static_assert(sizeof(RecordPrefix) + sizeof(EventRecord) <= kRecordSize);

        enum FrameFlag : std::uint32_t
        {
            kFlagPlayerAvailable = 1u << 0,
            kFlagWeaponDrawn = 1u << 1,
            kFlagMenuBlocking = 1u << 2,
            kFlagLoadingMenu = 1u << 3,
            kFlagGameStopped = 1u << 4,
            kFlagInputMenuBlocking = 1u << 5,
            kFlagWorldReady = 1u << 6,
            kFlagHasHmd = 1u << 7,
            kFlagRoomValid = 1u << 8,
            kFlagHmdNodeValid = 1u << 9,
            kFlagLeftWandValid = 1u << 10,
            kFlagRightWandValid = 1u << 11,
        };

        struct HandInputAccumulator
        {
            std::atomic<std::uint64_t> rawPressed{ 0 };
            std::atomic<std::uint64_t> gamePressed{ 0 };
            std::atomic<std::uint64_t> rawPressedSeen{ 0 };
            std::atomic<std::uint64_t> gamePressedSeen{ 0 };
            std::atomic<float> rawTrigger{ 0.0f };
            std::atomic<float> gameTrigger{ 0.0f };
            std::atomic<std::uint32_t> blankReasons{ 0 };
            std::atomic<std::uint32_t> reads{ 0 };
            std::atomic<std::uint32_t> bypassedReads{ 0 };
        };

        // Queued from any thread; `gate` points at a string literal.
        struct PendingEvent
        {
            EventType type;
            std::uint8_t hand;
            std::uint8_t detail;
            float value0;
            std::uint32_t aux0;
            const char* gate;
        };

        // Producer side, owned by the frame thread.
        struct Ring
        {
            HANDLE mapping = nullptr;
            RingHeader* header = nullptr;
            std::uint8_t* records = nullptr;
            std::uint64_t next = 0;
            float gameTime = 0.0f;
        };

        std::atomic<bool> s_enabled{ false };
        std::array<HandInputAccumulator, 2> s_input{};
        std::atomic<std::uint32_t> s_failedReads{ 0 };
        std::atomic<std::uint32_t> s_noHandRoleReads{ 0 };
        std::mutex s_eventLock;
        std::array<PendingEvent, kEventQueueCapacity> s_events{};
        std::size_t s_eventCount = 0;           // guarded by s_eventLock
        std::uint32_t s_droppedEvents = 0;      // guarded by s_eventLock
        Ring s_ring;                            // frame thread only

        std::uint64_t queryCounter() noexcept
        {
            LARGE_INTEGER value{};
            QueryPerformanceCounter(&value);
            return static_cast<std::uint64_t>(value.QuadPart);
        }

        void closeRing() noexcept
        {
            s_enabled.store(false, std::memory_order_release);
            if (s_ring.header) {
                s_ring.header->flags = 0;
                UnmapViewOfFile(s_ring.header);
            }
            if (s_ring.mapping) {
                CloseHandle(s_ring.mapping);
            }
            s_ring = {};
        }

        bool openRing() noexcept
        {
            constexpr std::uint64_t bytes = sizeof(RingHeader) + static_cast<std::uint64_t>(kCapacity) * kRecordSize;
            HANDLE mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                static_cast<DWORD>(bytes >> 32), static_cast<DWORD>(bytes & 0xFFFFFFFFu), kSectionName);
            if (!mapping) {
                ROCK_LOG_WARN(Init, "Testbench telemetry: CreateFileMapping failed (error {})", GetLastError());
                return false;
            }
            auto* view = static_cast<std::uint8_t*>(MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, static_cast<SIZE_T>(bytes)));
            if (!view) {
                ROCK_LOG_WARN(Init, "Testbench telemetry: MapViewOfFile failed (error {})", GetLastError());
                CloseHandle(mapping);
                return false;
            }

            LARGE_INTEGER frequency{};
            QueryPerformanceFrequency(&frequency);
            auto* header = reinterpret_cast<RingHeader*>(view);
            std::memset(header, 0, sizeof(RingHeader));
            header->schemaVersion = kSchemaVersion;
            header->recordSize = kRecordSize;
            header->capacity = kCapacity;
            header->qpcFrequency = static_cast<std::uint64_t>(frequency.QuadPart);
            header->producerStartQpc = queryCounter();
            header->producerPid = GetCurrentProcessId();
            header->flags = 1;
            // Readers validate the magic last, so a half-written header is never accepted.
            std::atomic_ref<std::uint32_t>(header->magic).store(kMagic, std::memory_order_release);

            s_ring.mapping = mapping;
            s_ring.header = header;
            s_ring.records = view + sizeof(RingHeader);
            s_ring.next = 0;
            s_ring.gameTime = 0.0f;
            for (auto& hand : s_input) {
                hand.rawPressedSeen.store(0, std::memory_order_relaxed);
                hand.gamePressedSeen.store(0, std::memory_order_relaxed);
                hand.blankReasons.store(0, std::memory_order_relaxed);
                hand.reads.store(0, std::memory_order_relaxed);
                hand.bypassedReads.store(0, std::memory_order_relaxed);
            }
            {
                std::scoped_lock lock(s_eventLock);
                s_eventCount = 0;
                s_droppedEvents = 0;
            }
            s_enabled.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Testbench telemetry publishing to RPS_Telemetry_ROCK ({} records of {} bytes)", kCapacity, kRecordSize);
            return true;
        }

        std::uint8_t* beginRecord(RecordType type, std::uint16_t payloadSize, std::uint32_t frameIndex, float deltaSeconds) noexcept
        {
            auto* record = s_ring.records + (s_ring.next % kCapacity) * kRecordSize;
            RecordPrefix prefix{ static_cast<std::uint16_t>(type), payloadSize, frameIndex, queryCounter(), s_ring.gameTime, deltaSeconds };
            std::memcpy(record, &prefix, sizeof(prefix));
            return record + sizeof(prefix);
        }

        void commitRecord() noexcept
        {
            ++s_ring.next;
            std::atomic_ref<std::uint64_t>(s_ring.header->writeIndex).store(s_ring.next, std::memory_order_release);
        }

        void copyVector(float (&out)[3], const float (&in)[3]) noexcept
        {
            std::memcpy(out, in, sizeof(out));
        }

        void copyNode(NodeRecord& out, const NodeSample& in) noexcept
        {
            std::memcpy(out.position, in.position, sizeof(out.position));
            std::memcpy(out.rotation, in.rotation, sizeof(out.rotation));
        }
    }

    bool enabled() noexcept
    {
        return s_enabled.load(std::memory_order_relaxed);
    }

    void observeControllerState(const bool isLeft, const std::uint64_t rawPressed, const std::uint64_t gamePressed,
        const float rawTrigger, const float gameTrigger, const std::uint32_t blankReasons, const bool bypassed) noexcept
    {
        if (!enabled()) {
            return;
        }
        auto& hand = s_input[isLeft ? 0 : 1];
        hand.rawPressed.store(rawPressed, std::memory_order_relaxed);
        hand.rawPressedSeen.fetch_or(rawPressed, std::memory_order_relaxed);
        hand.rawTrigger.store(rawTrigger, std::memory_order_relaxed);
        if (bypassed) {
            hand.bypassedReads.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        hand.gamePressed.store(gamePressed, std::memory_order_relaxed);
        hand.gamePressedSeen.fetch_or(gamePressed, std::memory_order_relaxed);
        hand.gameTrigger.store(gameTrigger, std::memory_order_relaxed);
        hand.blankReasons.fetch_or(blankReasons, std::memory_order_relaxed);
        hand.reads.fetch_add(1, std::memory_order_relaxed);
    }

    void observeUnobservedRead(const UnobservedRead kind) noexcept
    {
        if (!enabled()) {
            return;
        }
        (kind == UnobservedRead::Failed ? s_failedReads : s_noHandRoleReads).fetch_add(1, std::memory_order_relaxed);
    }

    namespace
    {
        void queueEvent(const PendingEvent& event) noexcept
        {
            std::scoped_lock lock(s_eventLock);
            if (s_eventCount < s_events.size()) {
                s_events[s_eventCount++] = event;
            } else {
                ++s_droppedEvents;
            }
        }
    }

    void recordHaptic(const bool isLeft, const std::uint16_t durationMicroseconds, const HapticCaller caller, const bool retargeted) noexcept
    {
        if (!enabled()) {
            return;
        }
        queueEvent({ EventType::Haptic, static_cast<std::uint8_t>(isLeft ? 0 : 1), static_cast<std::uint8_t>(retargeted ? 1 : 0),
            static_cast<float>(durationMicroseconds), static_cast<std::uint32_t>(caller), nullptr });
    }

    void recordInputGate(const char* gate, const bool sourceIsLeft, const bool justPressed, const bool suppressed,
        const bool providerSuppressed) noexcept
    {
        if (!enabled()) {
            return;
        }
        const auto detail = static_cast<std::uint8_t>((justPressed ? 1u : 0u) | (suppressed ? 2u : 0u) | (providerSuppressed ? 4u : 0u));
        queueEvent({ EventType::InputGate, static_cast<std::uint8_t>(sourceIsLeft ? 0 : 1), detail, 0.0f, 0, gate });
    }

    void publishFrame(const bool settingOn, const FrameSample& frame) noexcept
    {
        if (settingOn != (s_ring.header != nullptr)) {
            if (settingOn) {
                if (!openRing()) {
                    return;
                }
            } else {
                closeRing();
                return;
            }
        }
        if (!s_ring.header) {
            return;
        }

        const auto frameIndex = static_cast<std::uint32_t>(frame.frameIndex);
        s_ring.gameTime += frame.deltaSeconds;

        // Events first, so a reader that sees a frame has already seen everything queued before it.
        std::array<PendingEvent, kEventQueueCapacity> events;
        std::size_t eventCount = 0;
        std::uint32_t dropped = 0;
        {
            std::scoped_lock lock(s_eventLock);
            eventCount = s_eventCount;
            std::copy_n(s_events.begin(), eventCount, events.begin());
            dropped = s_droppedEvents;
            s_eventCount = 0;
            s_droppedEvents = 0;
        }
        for (std::size_t i = 0; i < eventCount; ++i) {
            const auto& pending = events[i];
            EventRecord record{};
            record.eventType = static_cast<std::uint16_t>(pending.type);
            record.hand = pending.hand;
            record.detail = pending.detail;
            record.value[0] = pending.value0;
            record.aux[0] = pending.aux0;
            if (pending.gate) {
                strncpy_s(record.text, pending.gate, _TRUNCATE);
            }
            std::memcpy(beginRecord(RecordType::Event, sizeof(record), frameIndex, frame.deltaSeconds), &record, sizeof(record));
            commitRecord();
        }

        FrameRecord record{};
        record.flags = (frame.playerAvailable ? kFlagPlayerAvailable : 0u) | (frame.weaponDrawn ? kFlagWeaponDrawn : 0u) |
                       (frame.menuBlocking ? kFlagMenuBlocking : 0u) | (frame.loadingMenuOpen ? kFlagLoadingMenu : 0u) |
                       (frame.gameStopped ? kFlagGameStopped : 0u) | (frame.inputMenuBlocking ? kFlagInputMenuBlocking : 0u) |
                       (frame.worldReady ? kFlagWorldReady : 0u) | (frame.hasHmd ? kFlagHasHmd : 0u) |
                       (frame.room.valid ? kFlagRoomValid : 0u) | (frame.hmdNode.valid ? kFlagHmdNodeValid : 0u) |
                       (frame.wand[0].valid ? kFlagLeftWandValid : 0u) | (frame.wand[1].valid ? kFlagRightWandValid : 0u);
        record.droppedEvents = dropped;
        record.failedReads = s_failedReads.exchange(0, std::memory_order_relaxed);
        record.noHandRoleReads = s_noHandRoleReads.exchange(0, std::memory_order_relaxed);
        copyVector(record.hmdPosition, frame.hmdPosition);
        copyVector(record.hmdForward, frame.hmdForward);
        copyNode(record.room, frame.room);
        record.roomScale = frame.room.scale;
        copyNode(record.hmdNode, frame.hmdNode);
        copyNode(record.wand[0], frame.wand[0]);
        copyNode(record.wand[1], frame.wand[1]);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& hand = frame.hand[i];
            auto& out = record.hand[i];
            copyVector(out.position, hand.position);
            std::memcpy(out.rotation, hand.rotation, sizeof(out.rotation));
            copyVector(out.velocity, hand.velocity);
            out.hasVelocity = hand.hasVelocity ? 1 : 0;
            out.disabled = hand.disabled ? 1 : 0;

            auto& input = s_input[i];
            auto& in = record.input[i];
            in.rawPressed = input.rawPressed.load(std::memory_order_relaxed);
            in.gamePressed = input.gamePressed.load(std::memory_order_relaxed);
            in.rawPressedSeen = input.rawPressedSeen.exchange(0, std::memory_order_relaxed);
            in.gamePressedSeen = input.gamePressedSeen.exchange(0, std::memory_order_relaxed);
            in.rawTrigger = input.rawTrigger.load(std::memory_order_relaxed);
            in.gameTrigger = input.gameTrigger.load(std::memory_order_relaxed);
            in.blankReasons = input.blankReasons.exchange(0, std::memory_order_relaxed);
            in.reads = input.reads.exchange(0, std::memory_order_relaxed);
            in.bypassedReads = input.bypassedReads.exchange(0, std::memory_order_relaxed);
        }
        std::memcpy(beginRecord(RecordType::Frame, sizeof(record), frameIndex, frame.deltaSeconds), &record, sizeof(record));
        commitRecord();
    }
}
