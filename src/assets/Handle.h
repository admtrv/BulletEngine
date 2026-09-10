/*
 * Handle.h
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

namespace BulletEngine {
namespace assets {

// reference to a loaded asset, carries the key it was loaded by
template<class T>
class Handle {
public:
    Handle() = default;
    Handle(std::string key, std::shared_ptr<T> asset) : m_key(std::move(key)), m_asset(std::move(asset)) {}

    // identity, what gets written to a scene file
    const std::string& getKey() const { return m_key; }

    // access
    T* get() const { return m_asset.get(); }
    const std::shared_ptr<T>& getShared() const { return m_asset; }
    T* operator->() const { return m_asset.get(); }
    T& operator*() const { return *m_asset; }

    bool isValid() const { return m_asset != nullptr; }
    explicit operator bool() const { return isValid(); }

    void reset() { m_key.clear(); m_asset.reset(); }

    bool operator==(const Handle& other) const { return m_asset == other.m_asset; }
    bool operator!=(const Handle& other) const { return !(*this == other); }

private:
    std::string m_key;
    std::shared_ptr<T> m_asset;
};

} // namespace assets
} // namespace BulletEngine
