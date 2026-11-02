// File-system helpers for ROCK's files under the user's Documents folder.
#pragma once

#include <string>

namespace rock::resources
{
    // Absolute path of relativePath under the Windows Documents known folder.
    [[nodiscard]] std::string getPathInDocuments(const std::string& relativePath);
    // Creates every missing directory above filePath.
    void createDirectoryTreeForFile(const std::string& filePath);
}
