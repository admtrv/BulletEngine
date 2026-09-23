/*
 * Project.h
 */

#pragma once

#include "project/Settings.h"

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

    bool open(const std::string& path);             // folder holding project file, what editor is pointed at
    bool isOpen() const { return !m_root.empty(); }
    static bool create(const std::string& folder, const Settings& settings);    // fills fresh folder with project file and first scene

    const Settings& getSettings() const { return m_settings; }
    bool setSettings(const Settings& settings);     // writes them back to project file

    const std::string& getRoot() const { return m_root; }
    const std::string& getName() const { return m_settings.name; }
    std::string getPath(const std::string& key) const;      // key is relative to root, loaders need whole path

    // tree
    const Entry& getTree() const { return m_tree; }
    void rescan();
    std::vector<std::string> getKeys(std::string_view extension) const;     // every file of a kind, in tree order
    std::vector<std::string> poll(float dt);                                // rescans on change, returns keys that went stale

    // contents, tree is rescanned on success
    bool move(const std::string& key, const std::string& folder);
    bool remove(const std::string& key);

private:
    Project() = default;

    Project(const Project&) = delete;
    Project& operator=(const Project&) = delete;

    std::string m_root;
    std::string m_projectKey;    // file this project was opened by
    Settings m_settings;
    Entry m_tree;

    // write times of last scan, keyed as asset field is
    std::unordered_map<std::string, int64_t> m_written;
    float m_untilPoll = 0.0f;
};

} // namespace project
} // namespace BulletEngine
