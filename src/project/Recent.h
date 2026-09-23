/*
 * Recent.h
 */

#pragma once

#include <string>
#include <vector>

namespace BulletEngine {
namespace project {

// projects opened before, newest first, kept beside user settings
class Recent {
public:
    static Recent& instance();

    const std::vector<std::string>& getPaths() const { return m_paths; }

    void add(const std::string& path);        // moves it to front, writes list out
    void remove(const std::string& path);     // folder is gone, so is its line

private:
    Recent();

    Recent(const Recent&) = delete;
    Recent& operator=(const Recent&) = delete;

    void write() const;

    std::vector<std::string> m_paths;
};

} // namespace project
} // namespace BulletEngine
