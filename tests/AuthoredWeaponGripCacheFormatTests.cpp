#include "physics-interaction/weapon/AuthoredWeaponGripCacheFormat.h"

#include <iostream>
#include <string>

namespace
{
    bool expect(const bool condition, const char* message)
    {
        if (!condition) {
            std::cerr << message << '\n';
        }
        return condition;
    }

    rock::authored_weapon_grip_cache::CacheRecord makeRecord()
    {
        using namespace rock::authored_weapon_grip_cache;
        CacheRecord record{};
        record.key.weapon = StableFormIdentity{ .plugin = "ExampleWeapon.esl", .localFormId = 0x812 };
        record.key.pGripVariantKey = 0x1111222233334444ull;
        record.key.instanceContentKey = 0x5555666677778888ull;
        record.key.graphProfileKey = 0x9999AAAABBBBCCCCull;
        record.key.inPowerArmor = true;
        record.rightHandWeaponLocal.translate = { 1.0f, 2.0f, 3.0f };
        for (std::size_t index = 0; index < record.rightFiringFingerLocals.size(); ++index) {
            record.rightFiringFingerLocals[index].translate = { static_cast<float>(index) * 0.01f, 0.0f, 0.0f };
        }
        record.rightFiringFingerMask = kCompleteFiringFingerMask;
        record.idleClipPath = "Actors\\Character\\Weapon\\WPNIdleReady.hkx";
        record.requestedSubgraphIdentifier = 0x12345678ull;
        record.bindingSubgraphIdentifier = 0x87654321ull;
        record.quality = SampleQuality{
            .sampleCount = 5,
            .selectedTimeSeconds = 0.0f,
            .durationSeconds = 1.5f,
            .maxHandTranslationDelta = 0.01f,
            .maxHandRotationDeltaDegrees = 0.1f,
            .maxFingerTranslationDelta = 0.01f,
            .maxFingerRotationDeltaDegrees = 0.2f,
            .maxScaleDelta = 0.0001f,
            .stable = true,
        };
        record.checksum = calculateChecksum(record);
        return record;
    }
}

int main()
{
    using namespace rock::authored_weapon_grip_cache;
    bool ok = true;

    const auto source = makeRecord();
    ok &= expect(validRecord(source), "valid cache record rejected");
    const std::string text = serialize(source);
    CacheRecord parsed{};
    std::string error;
    ok &= expect(parse(text, parsed, &error), "serialized cache record did not parse");
    ok &= expect(parsed.key == source.key, "cache key changed during round trip");
    ok &= expect(parsed.idleClipPath == source.idleClipPath, "clip path changed during round trip");
    ok &= expect(parsed.rightFiringFingerMask == kCompleteFiringFingerMask, "finger mask changed during round trip");
    ok &= expect(parsed.checksum == calculateChecksum(parsed), "round-trip checksum mismatch");

    auto incomplete = source;
    incomplete.rightFiringFingerMask = 0x3FFF;
    incomplete.checksum = calculateChecksum(incomplete);
    ok &= expect(!validRecord(incomplete), "incomplete firing-finger pose accepted");

    auto dynamic = source;
    dynamic.quality.stable = false;
    dynamic.checksum = calculateChecksum(dynamic);
    ok &= expect(!validRecord(dynamic), "dynamic idle sample accepted for persistence");

    auto partialSampling = source;
    partialSampling.quality.sampleCount = 4;
    partialSampling.checksum = calculateChecksum(partialSampling);
    ok &= expect(!validRecord(partialSampling), "partially sampled idle pose accepted for persistence");

    std::string corrupted = text;
    const auto clip = corrupted.find("WPNIdleReady");
    if (clip != std::string::npos) {
        corrupted.replace(clip, 12, "WPNIdleOther");
    }
    CacheRecord rejected{};
    ok &= expect(!parse(corrupted, rejected, &error), "checksum did not reject modified cache data");

    auto invalidRotation = source;
    invalidRotation.rightHandWeaponLocal.rotate[0] = 4.0f;
    invalidRotation.checksum = calculateChecksum(invalidRotation);
    ok &= expect(!validRecord(invalidRotation), "non-orthonormal transform accepted");

    return ok ? 0 : 1;
}
