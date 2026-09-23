#include "physics-interaction/weapon/WeaponTextureSource.h"

#include "RE/Bethesda/BSResourceNiBinaryStream.h"
#include "RE/Bethesda/BSTextureStreamer.h"
#include "REL/Relocation.h"

#include <Windows.h>
#include <array>
#include <limits>
#include <vector>
#include <zlib.h>

namespace rock::weapon_texture_source
{
    namespace
    {
        using Result = texture_alpha::Result;
        using Stream = RE::BSResource::Stream;
        using StreamPtr = RE::BSTSmartPointer<Stream>;
        constexpr std::size_t kMaximumBytes = 64 * 1024 * 1024;

        bool plausible(const void* pointer)
        {
            const auto value = reinterpret_cast<std::uintptr_t>(pointer);
            return value >= 0x10000 && value < 0x0000800000000000 && (value & 7) == 0;
        }

        // NativeDesc is copied by 141D33600 and consumed by 141D37AE0/141D39870.
        // Info is passed to CreateTexture2D at 141D926A0: height then width,
        // mip count, DXGI format, and cube flag (bit 0). The native copy retains
        // the BSFixedString at +78, so this record must run its destructor.
        struct TextureInfo
        {
            std::uint16_t height = 0, width = 0;
            std::uint8_t mips = 0, format = 0, flags = 0, padding = 0;
        };
        using Descriptor = RE::BSTextureStreamer::NativeDesc<TextureInfo>;
        static_assert(sizeof(Descriptor) == 0x80);
        static_assert(offsetof(Descriptor, chunks) == 0x18);
        static_assert(offsetof(Descriptor, streamName) == 0x78);

        bool lookupImpl(const char* path, Descriptor& descriptor, char*& manager, Trace& trace)
        {
            trace.stage = "texture-manager";
            // Both 141D2ED00 and 141D2EF00 obtain this same manager.
            manager = *REL::Relocation<char**>{ REL::Offset(0x6057270) };
            if (!plausible(manager)) return false;
            RE::BSResource::ID id{};
            using MakeID = void* (*)(RE::BSResource::ID*, const char*);
            static REL::Relocation<MakeID> makeID{ REL::Offset(0x1BEE4F0) };
            makeID(&id, path);
            descriptor.chunkOffset = 0x18;
            trace.stage = "texture-record";
            // 141D18230 and 141D68490 construct this record and call the
            // same lookup; 141D36840 holds the registry read lock internally.
            using Find = bool (*)(char*, const RE::BSResource::ID*, Descriptor*);
            static REL::Relocation<Find> find{ REL::Offset(0x1D36840) };
            return find(manager, &id, &descriptor);
        }

        bool lookup(const char* path, Descriptor& descriptor, char*& manager, Trace& trace)
        {
            __try {
                return lookupImpl(path, descriptor, manager, trace);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        class StreamTableLock
        {
        public:
            explicit StreamTableLock(char* manager) : _lock(manager + 0x989B0)
            {
                static REL::Relocation<void (*)(void*)> acquire{ REL::Offset(0x1B932B0) };
                acquire(_lock);
            }
            ~StreamTableLock()
            {
                static REL::Relocation<void (*)(void*)> release{ REL::Offset(0x1B93570) };
                release(_lock);
            }
        private:
            void* _lock;
        };

        bool retainArchive(char* manager, unsigned index, StreamPtr& retained)
        {
            __try {
                // Identical lock, array +989E0, count +989F0 and intrusive
                // Stream refs in 141D37B1C..B73 and 141D39943..9CF.
                const auto count = *reinterpret_cast<const std::uint32_t*>(manager + 0x989F0);
                auto** streams = *reinterpret_cast<Stream***>(manager + 0x989E0);
                if (count > 256 || index >= count || !plausible(streams) || !plausible(streams[index])) return false;
                retained.reset(streams[index]);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        bool readChunk(Stream* stream, std::uint64_t offset, std::span<std::uint8_t> bytes, Trace& trace)
        {
            __try {
                // Native archive reader 141BF7D80 uses vslot 7 with (buffer,
                // byteCount, absoluteOffset, outRead). This positional operation
                // leaves the shared stream cursor unchanged. Do not use the
                // shared CommonLib ReadAt wrapper: its two uint64 arguments swap.
                std::uint64_t read = 0;
                trace.error = static_cast<unsigned>(stream->DoReadAt(bytes.data(), bytes.size(), offset, read));
                return trace.error == 0 && read == bytes.size();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        Result inspectArchive(const char* path, const std::atomic<bool>& stopping, Trace& trace)
        {
            Descriptor descriptor{};
            char* manager = nullptr;
            if (!lookup(path, descriptor, manager, trace)) return Result::Unavailable;
            trace.archive = descriptor.dataFileIndex;
            trace.chunks = descriptor.chunkCount;
            trace.format = descriptor.info.format;
            trace.width = descriptor.info.width;
            trace.height = descriptor.info.height;
            trace.stage = "texture-record-validation";
            if (!descriptor.chunkCount || descriptor.chunkCount > 4 || descriptor.chunkOffset != 0x18 ||
                !descriptor.info.mips || descriptor.info.mips > 15 || !descriptor.info.width || !descriptor.info.height ||
                descriptor.info.width > 16384 || descriptor.info.height > 16384 || (descriptor.info.flags & 1))
                return Result::Unavailable;
            std::size_t total = 0;
            unsigned nextMip = 0;
            for (unsigned i = 0; i < descriptor.chunkCount; ++i) {
                const auto& chunk = descriptor.chunks[i];
                if (chunk.mipFirst != nextMip || chunk.mipLast < chunk.mipFirst || chunk.mipLast >= descriptor.info.mips ||
                    !chunk.uncompressedSize || chunk.uncompressedSize > kMaximumBytes - total || chunk.size > kMaximumBytes ||
                    chunk.dataFileOffset > static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)()) - kMaximumBytes)
                    return Result::Unavailable;
                total += chunk.uncompressedSize;
                nextMip = chunk.mipLast + 1;
            }
            if (nextMip != descriptor.info.mips) return Result::Unavailable;
            trace.stage = "archive-stream";
            StreamPtr stream;
            {
                StreamTableLock lock(manager);
                if (!retainArchive(manager, descriptor.dataFileIndex, stream)) return Result::Unavailable;
            }
            std::vector<std::uint8_t> mipData(total);
            std::vector<std::uint8_t> packed;
            std::size_t offset = 0;
            for (unsigned i = 0; i < descriptor.chunkCount; ++i) {
                if (stopping.load(std::memory_order_acquire)) return Result::Unavailable;
                const auto& chunk = descriptor.chunks[i];
                auto output = std::span(mipData).subspan(offset, chunk.uncompressedSize);
                trace.stage = "archive-read";
                if (chunk.size) {
                    packed.resize(chunk.size);
                    if (!readChunk(stream.get(), chunk.dataFileOffset, packed, trace)) return Result::Unavailable;
                    trace.stage = "archive-inflate";
                    uLongf size = static_cast<uLongf>(output.size());
                    if (uncompress(output.data(), &size, packed.data(), static_cast<uLong>(packed.size())) != Z_OK || size != output.size())
                        return Result::Unavailable;
                } else if (!readChunk(stream.get(), chunk.dataFileOffset, output, trace)) {
                    return Result::Unavailable;
                }
                offset += output.size();
                trace.bytes = offset;
            }
            trace.stage = "archive-alpha";
            return texture_alpha::inspectMipData(mipData, descriptor.info.format, descriptor.info.width,
                descriptor.info.height, descriptor.info.mips, &stopping);
        }

        std::size_t readResourceImpl(RE::BSResourceNiBinaryStream* stream, void* bytes, std::size_t size)
        {
            // VR stream vtable 142E57498 slot 5; shared REL ID is absent.
            using Read = std::size_t (*)(RE::BSResourceNiBinaryStream*, void*, std::size_t);
            static REL::Relocation<Read> read{ REL::Offset(0x1C13110) };
            return read(stream, bytes, size);
        }

        std::size_t readResource(RE::BSResourceNiBinaryStream* stream, void* bytes, std::size_t size)
        {
            __try {
                return readResourceImpl(stream, bytes, size);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return (std::numeric_limits<std::size_t>::max)();
            }
        }
    }

    Result inspect(const char* path, const std::atomic<bool>& stopping, Trace& trace)
    {
        RE::BSResourceNiBinaryStream stream(path);
        trace.error = static_cast<unsigned>(stream.lastError);
        if (!stream.good()) {
            // DX10 archive entries are in the texture manager, not the general
            // resource registry. Native texture loading also resolves both
            // paths (141D68490). Never bypass a present but unreadable override.
            if (stream.lastError == RE::BSResource::ErrorCode::kNotExist) return inspectArchive(path, stopping, trace);
            return Result::Unavailable;
        }
        trace.stage = "resource-read";
        std::vector<std::uint8_t> bytes;
        constexpr std::size_t chunk = 64 * 1024;
        while (bytes.size() < kMaximumBytes && !stopping.load(std::memory_order_acquire)) {
            const auto oldSize = bytes.size();
            bytes.resize(oldSize + chunk);
            const auto count = readResource(&stream, bytes.data() + oldSize, chunk);
            trace.error = static_cast<unsigned>(stream.lastError);
            if (count > chunk) return Result::Unavailable;
            bytes.resize(oldSize + count);
            trace.bytes = bytes.size();
            if (count < chunk) {
                trace.stage = "dds-alpha";
                return texture_alpha::inspect(bytes, &stopping);
            }
        }
        trace.stage = stopping.load(std::memory_order_acquire) ? "stopped" : "byte-limit";
        return Result::Unavailable;
    }
}
