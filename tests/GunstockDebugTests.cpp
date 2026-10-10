#include "physics-interaction/weapon/GunstockDebug.h"
#include "physics-interaction/TransformMath.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>

int main(int argc, char** argv)
{
    using namespace rock::gunstock_debug;
    if (argc != 2) return 1;
    const std::filesystem::path root(argv[1]);
    std::filesystem::create_directories(root);
    Sample sample{};
    std::strcpy(sample.source, "foregrip \"sample\", position");
    auto pose = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    pose.translate = {17, -23, 61};
    if (!setPose(sample, LeftController, pose) || !sample.has(LeftController)) return 2;
    pose.translate.x = (std::numeric_limits<float>::quiet_NaN)();
    if (setPose(sample, RightController, pose) || sample.has(RightController)) return 3;
    const auto directory = root / ("queue-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uint64_t accepted = 0;
    {
        Recorder recorder(directory);
        for (std::uint64_t i = 1; i <= 4096; ++i) {
            sample.frame = i;
            sample.mark = i / 25;
            if (recorder.submit(sample)) ++accepted;
        }
        if (accepted + recorder.dropped() != 4096) return 4;
        // Destruction must flush every accepted sample, even under saturation.
    }
    std::uint64_t count = 0, previous = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        std::ifstream file(entry.path());
        std::string line;
        std::getline(file, line);
        if (line.find("left_controller_r0") == std::string::npos) return 5;
        while (std::getline(file, line)) {
            const auto frame = std::stoull(line.substr(0, line.find(',')));
            if (frame <= previous) return 6;
            if (line.find(",17,-23,61,1") == std::string::npos ||
                line.find("\"foregrip \"\"sample\"\", position\"") == std::string::npos) return 8;
            previous = frame; ++count;
        }
    }
    if (!count || count != accepted) return 7;
    const auto holding=directory/"holding";
    {
        Recorder recorder(directory/"unused-csv",holding);Sample capture{};capture.phase=2;capture.frame=10;capture.holding.requested=true;capture.holding.ready=true;capture.holding.nativeFingersValid=true;capture.holding.captureOnly=true;
        std::strcpy(capture.holding.weapon,"test.esp:000123");auto identity=rock::transform_math::makeIdentityTransform<RE::NiTransform>();capture.holding.rightBoneInWand=identity;capture.holding.leftBoneInWand=identity;capture.holding.nativeRight=identity;
        for(auto& f:capture.holding.nativeFingers)f=identity;setPose(capture,WeaponRendered,identity);setPose(capture,LeftRendered,identity);setPose(capture,RightRendered,identity);
        if(!recorder.submit(capture))return 9;
    }
    std::ifstream captureFile(holding/"0-0-10.json");nlohmann::json capture;captureFile>>capture;
    if(capture.at("leftGripBaselineY")!=0.0f||capture.at("weapon")!="test.esp:000123"||capture.at("nativeFingers").size()!=15||!capture.contains("renderedLeft")||std::filesystem::exists(directory/"unused-csv"))return 10;
    std::cout << "Accepted " << accepted << " samples; verified ordered, complete drain and invalid-pose rejection.\n";
    return 0;
}
