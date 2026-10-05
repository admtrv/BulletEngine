/*
 * Handle.h
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

namespace BulletEngine {
namespace assets {

enum class State {
    Empty,      // handle names nothing
    Loading,
    Ready,
    Failed      // key names nothing, or what it names could not be read
};

// shared, so every handle of one key sees asset arrive
template<class T>
struct Slot {
    std::shared_ptr<T> asset;
    State state = State::Loading;
};

template<class T>
class Handle {
public:
    Handle() = default;
    explicit Handle(std::string key, std::shared_ptr<Slot<T>> slot) : m_key(std::move(key)), m_slot(std::move(slot)) {}

    const std::string& getKey() const { return m_key; }      // what scene file holds

    // access
    T* get() const { return m_slot ? m_slot->asset.get() : nullptr; }
    std::shared_ptr<T> getShared() const { return m_slot ? m_slot->asset : nullptr; }
    T* operator->() const { return get(); }
    T& operator*() const { return *get(); }

    State getState() const { return m_slot ? m_slot->state : State::Empty; }
    bool isValid() const { return get() != nullptr; }
    explicit operator bool() const { return isValid(); }

    void reset() { m_key.clear(); m_slot.reset(); }

    bool operator==(const Handle& other) const { return m_slot == other.m_slot; }
    bool operator!=(const Handle& other) const { return !(*this == other); }

private:
    std::string m_key;
    std::shared_ptr<Slot<T>> m_slot;
};

} // namespace assets
} // namespace BulletEngine
