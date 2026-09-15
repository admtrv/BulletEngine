/*
 * Project.h
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace project {

// one file or folder under project root
struct Entry {
    std::string name;               // as panel reads it
    std::string key;                // path from root, what asset field stores
    bool directory = false;
    int64_t written = 0;            // changed one means file was touched
    std::vector<Entry> children;
};

// folder editor is opened with, everything user owns lives under it
class Project {
public:
    static Project& instance();

    bool open(const std::string& path);
    bool isOpen() const { return !m_root.empty(); }

    const std::string& getRoot() const { return m_root; }
    const std::string& getName() const { return m_name; }

    // key is relative to root, loaders need whole path
    std::string getPath(const std::string& key) const;

    const Entry& getTree() const { return m_tree; }
    void rescan();

    // every file of a kind, in tree order
    std::vector<std::string> getKeys(std::string_view extension) const;

    // contents, tree is rescanned on success
    bool move(const std::string& key, const std::string& folder);
    bool remove(const std::string& key);

    // rescans on change, returns keys that went stale
    std::vector<std::string> poll(float dt);

private:
    Project() = default;

    Project(const Project&) = delete;
    Project& operator=(const Project&) = delete;

    std::string m_root;
    std::string m_name;
    Entry m_tree;

    // write times of last scan, keyed as asset field is
    std::unordered_map<std::string, int64_t> m_written;
    float m_untilPoll = 0.0f;
};

} // namespace project
} // namespace BulletEngine
