#include "rock_support/ResourceUtils.h"

#include "rock_support/Logger.h"

#include <Windows.h>
#include <ShlObj_core.h>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace rock::resources
{
    std::string getPathInDocuments(const std::string& relativePath)
    {
        char documentsPath[MAX_PATH]{};
        if (FAILED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documentsPath))) {
            throw std::runtime_error("Failed to resolve the Windows Documents directory");
        }
        return std::string(documentsPath) + relativePath;
    }

    void createDirectoryTreeForFile(const std::string& filePath)
    {
        auto path = std::filesystem::path(filePath);
        if (path.has_extension()) {
            path = path.parent_path();
        }
        if (path.empty() || std::filesystem::exists(path)) {
            return;
        }

        logger::info("Creating directory: {}", path.string());
        std::filesystem::create_directories(path);
    }
}
