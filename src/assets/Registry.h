/*
 * Registry.h
 */

#pragma once

#include "assets/Handle.h"

#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace BulletEngine {
namespace assets {

// routes asset keys to their loaders
class Registry {
public:
    static Registry& instance();

    // how a type is built from a key
    template<class T>
    using Loader = std::function<std::shared_ptr<T>(const std::string&)>;

    template<class T>
    void setLoader(Loader<T> loader)
    {
        m_loaders[std::type_index(typeid(T))] = [loader = std::move(loader)](const std::string& key) -> std::shared_ptr<void> {
            return loader(key);
        };
    }

    template<class T>
    Handle<T> load(const std::string& key)
    {
        auto asset = std::static_pointer_cast<T>(build(std::type_index(typeid(T)), key));
        return Handle<T>(key, std::move(asset));
    }

    void clear() { m_loaders.clear(); }

    size_t getCount() const { return m_loaders.size(); }

private:
    Registry() = default;

    std::shared_ptr<void> build(std::type_index type, const std::string& key) const;

    std::unordered_map<std::type_index, std::function<std::shared_ptr<void>(const std::string&)>> m_loaders;
};

} // namespace assets
} // namespace BulletEngine
