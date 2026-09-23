/*
 * Project.cpp
 */

#include "Project.h"

#include "scene/Serializer.h"

#include <algorithm>
#include <filesystem>
#include <iostream>

namespace BulletEngine {
namespace project {

namespace fs = std::filesystem;

constexpr float POLL_INTERVAL = 1.0f;       // seconds between looks

constexpr const char* PROJECT_EXTENSION = ".project";
constexpr const char* PROJECT_FILE = "Project.project";     // same name everywhere, folder says which project it is
constexpr const char* SCENE_EXTENSION = ".scene";

// folders first, then names, as file manager reads
static bool byKind(const Entry& a, const Entry& b)
{
    return a.directory != b.directory ? a.directory : a.name < b.name;
}

// every file under entry, depth first keeps tree order
template<class Visit>
static void forEachFile(const Entry& entry, Visit visit)
{
    for (const Entry& child : entry.children)
    {
        if (child.directory)
        {
            forEachFile(child, visit);
        }
        else
        {
            visit(child);
        }
    }
}

static void scan(const fs::path& path, const fs::path& root, Entry& entry)
{
    std::error_code error;

    for (const fs::directory_entry& item : fs::directory_iterator(path, error))
    {
        const std::string name = item.path().filename().string();

        // dot folders carry version control and editor state, not assets
        if (name.starts_with('.'))
        {
            continue;
        }

        Entry child;
        child.name = name;
        child.key = fs::relative(item.path(), root, error).generic_string();
        child.directory = item.is_directory(error);
        child.written = item.last_write_time(error).time_since_epoch().count();

        if (child.directory)
        {
            scan(item.path(), root, child);
        }

        entry.children.push_back(std::move(child));
    }

    std::sort(entry.children.begin(), entry.children.end(), byKind);
}

Project& Project::instance()
{
    static Project project;
    return project;
}

bool Project::open(const std::string& path)
{
    std::error_code error;
    const fs::path root = fs::absolute(path, error);

    if (error || !fs::is_directory(root, error))
    {
        std::cerr << "project open failed: " << path << '\n';
        return false;
    }

    m_root = root.generic_string();
    rescan();

    // project file says what it is, first one found stands for it
    const std::vector<std::string> files = getKeys(PROJECT_EXTENSION);

    if (files.empty() || !readSettings(m_settings, getPath(files.front())))
    {
        std::cerr << "project open failed: " << path << " (no project file)\n";

        m_root.clear();
        return false;
    }

    m_projectKey = files.front();
    return true;
}

bool Project::create(const std::string& folder, const Settings& settings)
{
    std::error_code error;
    fs::create_directories(folder, error);

    if (error)
    {
        std::cerr << "project create failed: " << folder << '\n';
        return false;
    }

    const fs::path root(folder);

    if (!writeSettings(settings, (root / PROJECT_FILE).generic_string()))
    {
        return false;
    }

    // scene it starts with, so fresh project opens into something
    const fs::path scene = root / settings.startScene;
    fs::create_directories(scene.parent_path(), error);

    ecs::World world;
    return scene::save(world, scene.generic_string());
}

bool Project::setSettings(const Settings& settings)
{
    if (m_projectKey.empty())
    {
        return false;
    }

    m_settings = settings;
    return writeSettings(m_settings, getPath(m_projectKey));
}

std::string Project::getPath(const std::string& key) const
{
    if (m_root.empty())
    {
        return key;
    }

    // empty key is root itself, everything else hangs off it
    return key.empty() ? m_root : (fs::path(m_root) / key).generic_string();
}

void Project::rescan()
{
    m_tree = Entry{};

    if (m_root.empty())
    {
        return;
    }

    m_tree.name = fs::path(m_root).filename().string();
    m_tree.directory = true;

    scan(m_root, m_root, m_tree);

    m_written.clear();
    forEachFile(m_tree, [this](const Entry& file) { m_written.emplace(file.key, file.written); });
}

std::vector<std::string> Project::getKeys(std::string_view extension) const
{
    std::vector<std::string> keys;

    forEachFile(m_tree, [&](const Entry& file) {
        if (file.key.ends_with(extension))
        {
            keys.push_back(file.key);
        }
    });

    return keys;
}

bool Project::move(const std::string& key, const std::string& folder)
{
    const fs::path from = getPath(key);
    const fs::path to = fs::path(getPath(folder)) / from.filename();

    if (from == to)
    {
        return false;
    }

    // folder cannot move inside itself, would take target with it
    std::error_code error;

    if (fs::is_directory(from, error) && to.string().starts_with(from.string() + "/"))
    {
        std::cerr << "move failed: " << key << " into itself\n";
        return false;
    }

    fs::rename(from, to, error);

    if (error)
    {
        std::cerr << "move failed: " << key << " (" << error.message() << ")\n";
        return false;
    }

    rescan();
    return true;
}

bool Project::remove(const std::string& key)
{
    std::error_code error;
    fs::remove_all(getPath(key), error);

    if (error)
    {
        std::cerr << "remove failed: " << key << " (" << error.message() << ")\n";
        return false;
    }

    rescan();
    return true;
}

std::vector<std::string> Project::poll(float dt)
{
    m_untilPoll -= dt;

    if (m_root.empty() || m_untilPoll > 0.0f)
    {
        return {};
    }

    m_untilPoll = POLL_INTERVAL;

    std::unordered_map<std::string, int64_t> before = std::move(m_written);
    rescan();

    // files written since last look
    std::vector<std::string> stale;

    for (const auto& [key, written] : m_written)
    {
        const auto found = before.find(key);

        if (found != before.end() && found->second != written)
        {
            stale.push_back(key);
        }
    }

    return stale;
}

} // namespace project
} // namespace BulletEngine
