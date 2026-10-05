/*
 * Registry.h
 */

#pragma once

#include "app/TaskExecutor.h"
#include "assets/Handle.h"

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace BulletEngine {
namespace assets {

class Registry {
public:
    static Registry& instance();

    // prepare runs on worker, finish needs graphics context so it runs on main thread
    template<class T>
    using Prepare = std::function<std::any(const std::string&)>;

    template<class T>
    using Finish = std::function<std::shared_ptr<T>(std::any&)>;

    template<class T>
    void setLoader(std::function<std::shared_ptr<T>(const std::string&)> loader)
    {
        setLoader<T>([loader](const std::string& key) { return std::any(loader(key)); },
                     [](std::any& ready) { return std::any_cast<std::shared_ptr<T>>(ready); });
    }

    template<class T>
    void setLoader(Prepare<T> prepare, Finish<T> finish)
    {
        Steps steps;

        steps.prepare = std::move(prepare);
        steps.finish = [finish = std::move(finish)](std::any& ready) -> std::shared_ptr<void> { return finish(ready); };

        m_loaders[std::type_index(typeid(T))] = std::move(steps);
    }

    // handle comes back at once, empty until read is through
    template<class T>
    Handle<T> load(const std::string& key)
    {
        if (key.empty())
        {
            return {};
        }

        const std::type_index type(typeid(T));

        if (auto found = find(type, key))
        {
            return Handle<T>(key, std::static_pointer_cast<Slot<T>>(found));
        }

        auto slot = std::make_shared<Slot<T>>();
        remember(type, key, slot);

        const Steps* steps = loaderOf(type);

        if (!steps)
        {
            slot->state = State::Failed;
            return Handle<T>(key, slot);
        }

        // slot is watched rather than held, so dropping every handle drops work too
        app::TaskExecutor::instance().run(
            [prepare = steps->prepare, source = m_resolve ? m_resolve(key) : key]() { return prepare(source); },
            [finish = steps->finish, watched = std::weak_ptr<Slot<T>>(slot)](std::any ready) {
                const std::shared_ptr<Slot<T>> slot = watched.lock();

                if (!slot)
                {
                    return;
                }

                slot->asset = std::static_pointer_cast<T>(finish(ready));
                slot->state = slot->asset ? State::Ready : State::Failed;
            });

        return Handle<T>(key, slot);
    }

    // key to path, resolved here so no worker reaches into project
    void setResolver(std::function<std::string(const std::string&)> resolve) { m_resolve = std::move(resolve); }

    void forget(const std::string& key);    // next load reads file again
    void clear();

    // cache drops what nothing holds, so caller keeps this while rebuilding what held it
    using Retained = std::vector<std::shared_ptr<void>>;
    Retained retainAll() const;

    size_t getCount() const { return m_loaders.size(); }

private:
    Registry() = default;
    ~Registry() = default;

    Registry(const Registry&) = delete;
    Registry& operator=(const Registry&) = delete;

    struct Steps {
        std::function<std::any(const std::string&)> prepare;
        std::function<std::shared_ptr<void>(std::any&)> finish;
    };

    const Steps* loaderOf(std::type_index type) const;

    std::shared_ptr<void> find(std::type_index type, const std::string& key) const;
    void remember(std::type_index type, const std::string& key, std::shared_ptr<void> slot);

    std::function<std::string(const std::string&)> m_resolve;
    std::unordered_map<std::type_index, Steps> m_loaders;
    std::unordered_map<std::type_index, std::unordered_map<std::string, std::weak_ptr<void>>> m_slots;
};

} // namespace assets
} // namespace BulletEngine
