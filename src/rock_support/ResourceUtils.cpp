#include "rock_support/ResourceUtils.h"

#include "rock_support/Logger.h"

#include <Windows.h>
#include <ShlObj_core.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace rock::resources
{
    namespace
    {
        [[nodiscard]] std::string loadResource(const std::string& moduleName, const std::uint16_t resourceId)
        {
            const HMODULE module = GetModuleHandleA((moduleName + ".dll").c_str());
            if (!module) {
                throw std::runtime_error("Module not loaded while reading embedded resource: " + moduleName);
            }

            const HRSRC resource = FindResourceA(module, MAKEINTRESOURCEA(resourceId), MAKEINTRESOURCEA(10));
            if (!resource) {
                throw std::runtime_error(
                    "Embedded resource not found for module '" + moduleName + "', id " + std::to_string(resourceId));
            }

            const HGLOBAL loadedResource = LoadResource(module, resource);
            if (!loadedResource) {
                throw std::runtime_error(
                    "Failed to load embedded resource for module '" + moduleName + "', id " + std::to_string(resourceId));
            }

            const DWORD resourceSize = SizeofResource(module, resource);
            const void* resourceData = LockResource(loadedResource);
            if (!resourceData) {
                throw std::runtime_error(
                    "Failed to lock embedded resource for module '" + moduleName + "', id " + std::to_string(resourceId));
            }

            return std::string(static_cast<const char*>(resourceData), resourceSize);
        }
    }

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

    void createFileFromResourceIfMissing(
        const std::string& filePath,
        const std::string& moduleName,
        const std::uint16_t resourceId,
        const bool normalizeNewlines)
    {
        if (std::filesystem::exists(filePath)) {
            return;
        }

        logger::info("Creating '{}' from embedded resource {}", filePath, resourceId);
        auto contents = loadResource(moduleName, resourceId);
        if (normalizeNewlines) {
            std::erase(contents, '\r');
        }

        std::ofstream output(filePath, std::ios::binary | std::ios::trunc);
        if (!output || !output.write(contents.data(), static_cast<std::streamsize>(contents.size()))) {
            output.close();
            std::error_code ignored;
            std::filesystem::remove(filePath, ignored);
            throw std::runtime_error("Failed to create file from embedded resource: " + filePath);
        }

        logger::debug("Created '{}' from embedded resource ({} bytes)", filePath, contents.size());
    }
}
