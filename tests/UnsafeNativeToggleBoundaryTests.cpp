#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{
    bool hasSourceExtension(const std::filesystem::path& path)
    {
        const auto ext = path.extension().string();
        return ext == ".cpp" || ext == ".h" || ext == ".hpp";
    }

    std::string readFile(const std::filesystem::path& path)
    {
        std::ifstream in(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
}

int main()
{
    const std::filesystem::path root{ ROCK_PHYSICS_INTERACTION_SOURCE_DIR };
    const std::vector<std::string> forbidden{
        "setBroadPhaseEnabled",
        "SetBodyBroadPhaseEnabled",
        "kFunc_SetBodyBroadPhaseEnabled",
    };

    bool ok = true;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file() || !hasSourceExtension(entry.path())) {
            continue;
        }

        const auto contents = readFile(entry.path());
        for (const auto& token : forbidden) {
            if (contents.find(token) != std::string::npos) {
                std::printf("Unsafe native body broadphase/deactivation token '%s' remains in %s\n", token.c_str(), entry.path().string().c_str());
                ok = false;
            }
        }
    }

    return ok ? 0 : 1;
}
