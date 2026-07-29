#pragma once

#include <cstdint>
#include <string>

namespace rock::resources
{
    [[nodiscard]] std::string getPathInDocuments(const std::string& relativePath);
    void createDirectoryTreeForFile(const std::string& filePath);
    void createFileFromResourceIfMissing(
        const std::string& filePath,
        const std::string& moduleName,
        std::uint16_t resourceId,
        bool normalizeNewlines);
}
