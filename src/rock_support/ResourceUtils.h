#pragma once

#include <string>

namespace rock::resources
{
    [[nodiscard]] std::string getPathInDocuments(const std::string& relativePath);
    void createDirectoryTreeForFile(const std::string& filePath);
}
