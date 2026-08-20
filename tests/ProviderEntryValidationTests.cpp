#include "api/detail/ProviderEntryValidation.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace rock::provider::detail
{
    bool g_testThreadValid{ true };

    bool entryThreadValid(const EntryThreadPolicy)
    {
        return g_testThreadValid;
    }
}

namespace
{
    using namespace rock::provider;
    using namespace rock::provider::detail;

    struct TestEntry
    {
        std::uint32_t size{ sizeof(TestEntry) };
        std::uint32_t version{ ROCK_PROVIDER_API_VERSION };
        bool semanticValid{ true };
    };

    struct ValidationCase
    {
        const char* name;
        bool threadValid;
        const TestEntry* entry;
        RockProviderResultV1 expectedResult;
        std::size_t expectedSemanticCalls;
    };
}

int main()
{
    const TestEntry badSize{
        .size = sizeof(TestEntry) - 1,
        .version = ROCK_PROVIDER_API_VERSION + 1,
        .semanticValid = false,
    };
    const TestEntry badVersion{
        .version = ROCK_PROVIDER_API_VERSION + 1,
        .semanticValid = false,
    };
    const TestEntry badSemantic{ .semanticValid = false };
    const TestEntry valid{};

    const std::array cases{
        ValidationCase{
            "thread before pointer",
            false,
            nullptr,
            RockProviderResultV1::WrongThread,
            0,
        },
        ValidationCase{
            "pointer before structure",
            true,
            nullptr,
            RockProviderResultV1::InvalidArgument,
            0,
        },
        ValidationCase{
            "size before version and semantics",
            true,
            &badSize,
            RockProviderResultV1::InvalidSize,
            0,
        },
        ValidationCase{
            "version before semantics",
            true,
            &badVersion,
            RockProviderResultV1::UnsupportedVersion,
            0,
        },
        ValidationCase{
            "semantic validation last",
            true,
            &badSemantic,
            RockProviderResultV1::InvalidArgument,
            1,
        },
        ValidationCase{
            "valid entry",
            true,
            &valid,
            RockProviderResultV1::Ok,
            1,
        },
    };

    for (const auto& testCase : cases) {
        g_testThreadValid = testCase.threadValid;
        std::size_t semanticCalls = 0;
        const auto result = validateEntry(
            testCase.entry,
            EntryThreadPolicy::AnimationOwner,
            [&semanticCalls](const TestEntry& entry) {
                ++semanticCalls;
                return entry.semanticValid;
            });
        (void)testCase.name;
        assert(result == testCase.expectedResult);
        assert(semanticCalls == testCase.expectedSemanticCalls);
    }

    return 0;
}
