/*
 * Recent.cpp
 */

#include "Recent.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <vector>

namespace BulletEngine {
namespace project {

namespace fs = std::filesystem;

constexpr size_t MAX_PATHS = 10;        // older ones fall off end

// where list lives, beside whatever else user owns
static fs::path listPath()
{
    const char* home = std::getenv("HOME");
    const fs::path root = home ? fs::path(home) / ".config" : fs::temp_directory_path();

    return root / "BulletEngine" / "recent";
}

Recent& Recent::instance()
{
    static Recent inst;
    return inst;
}

Recent::Recent()
{
    std::ifstream file(listPath());
    std::string line;

    while (m_paths.size() < MAX_PATHS && std::getline(file, line))
    {
        // only absolute paths were ever written, anything else is not ours
        if (!line.empty() && line.front() == '/')
        {
            m_paths.push_back(line);
        }
    }
}

void Recent::add(const std::string& path)
{
    std::erase(m_paths, path);
    m_paths.insert(m_paths.begin(), path);

    if (m_paths.size() > MAX_PATHS)
    {
        m_paths.resize(MAX_PATHS);
    }

    write();
}

void Recent::remove(const std::string& path)
{
    std::erase(m_paths, path);
    write();
}

void Recent::write() const
{
    const fs::path path = listPath();

    std::error_code error;
    fs::create_directories(path.parent_path(), error);

    std::ofstream file(path, std::ios::trunc);

    for (const std::string& line : m_paths)
    {
        file << line << '\n';
    }

    // list outlives run only if it reaches disk before window closes
    file.flush();
}

} // namespace project
} // namespace BulletEngine
