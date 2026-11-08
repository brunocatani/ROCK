#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

using namespace rock::saved_grab_offset;

namespace
{
    void check(bool value)
    {
        if (!value)
            throw std::runtime_error("Saved grab offset regression");
    }

    SavedGrabOffsetFile sample()
    {
        SavedGrabOffsetFile file;
        file.object = { "Fallout4.esm", 0x001beaee };
        file.objectName = "Amontillado Bottle";
        file.right.present = true;
        file.right.hasFingerPose = true;
        file.right.hasFingerJointValues = true;
        file.right.hasExactFingerRotations = true;
        file.right.fingerValues[0] = 1.8f;
        for (int i = 0; i < 15; ++i) {
            file.right.fingerJointValues[i] = i % 3 == 0 ? 1.6f : i % 3 == 1 ? 1.8f : 2.0f;
            auto& rotation = file.right.exactFingerRotations[i];
            const float angle = 0.17f * i;
            rotation[0] = std::cos(angle);
            rotation[1] = std::sin(angle);
            rotation[3] = -std::sin(angle);
            rotation[4] = std::cos(angle);
            rotation[8] = 1;
        }
        file.left.present = true;
        file.left.translateGame[1] = 3.25f;
        file.left.bodyNode = "c_Torso";
        return file;
    }
}

int main()
{
    try {
        auto file = sample();
        SavedGrabOffsetFile loaded;
        std::string error;
        check(parse(serialize(file), loaded, &error) && error.empty());
        check(loaded.right.hasExactFingerRotations && !loaded.left.hasExactFingerRotations);
        check(loaded.left.translateGame[1] == 3.25f);
        // The body node names the frame of a saved offset; a rigid object's offset has none.
        check(loaded.left.bodyNode == "c_Torso" && loaded.right.bodyNode.empty());
        for (int i = 0; i < 15; ++i) {
            check(loaded.right.fingerJointValues[i] == file.right.fingerJointValues[i]);
            for (int c = 0; c < 9; ++c)
                check(loaded.right.exactFingerRotations[i][c] == file.right.exactFingerRotations[i][c]);
        }
        auto valid = nlohmann::json::parse(serialize(file));
        // Count-based ammo variants are separate saved objects; files of
        // every other object keep the identity they always had.
        check(!valid["object"].contains("variant") && loaded.object.variant.empty());
        auto single = file;
        single.object.variant = "single";
        check(parse(serialize(single), loaded, &error) && error.empty() && loaded.object.variant == "single" &&
            loaded.object.localFormId == file.object.localFormId);
        auto badVariant = nlohmann::json::parse(serialize(single));
        badVariant["object"]["variant"] = 1;
        check(!parse(badVariant.dump(), loaded, &error));
        for (int version : { 1, 2 }) {
            auto old = valid;
            old["format"] = version;
            old["right"].erase("exactFinger");
            if (version == 1)
                old["right"].erase("finger");
            check(parse(old.dump(), loaded, &error) && error.empty());
            check(loaded.right.present && !loaded.right.hasExactFingerRotations);
            check(loaded.right.hasFingerPose == (version == 2));
        }
        for (int badCase = 0; badCase < 6; ++badCase) {
            auto bad = valid;
            if (badCase == 0)
                bad["right"]["exactFinger"]["rig"] = "unmatched";
            if (badCase == 5)
                bad["right"]["bodyNode"] = 7;
            if (badCase == 1)
                bad["right"]["exactFinger"]["rotations"].erase(14);
            if (badCase == 2)
                bad["right"]["exactFinger"]["rotations"][0][0] = -1; // reflection
            if (badCase == 3)
                bad["right"]["exactFinger"]["rotations"][0][1] = 2; // shear
            if (badCase == 4)
                bad["right"]["finger"]["jointValues"][0] = 2.1;
            check(parse(bad.dump(), loaded, &error) && !error.empty());
            check(!loaded.right.present && loaded.left.present);
        }
        auto oldWithUnknown = valid;
        oldWithUnknown["format"] = 2;
        check(parse(oldWithUnknown.dump(), loaded, &error) && !loaded.right.hasExactFingerRotations);
        valid["format"] = 5;
        check(!parse(valid.dump(), loaded, &error));
        valid["format"] = "not a number";
        check(!parse(valid.dump(), loaded, &error));
        valid["format"] = 4;
        valid["objectName"] = nlohmann::json::array();
        check(!parse(valid.dump(), loaded, &error));
        std::cout << "PASS: exact rotation round trip, over-open scalars, body node frame, legacy formats, opposite "
                     "hand, variant identity, rejection\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
