/*
 * Scheduler.h
 */

#pragma once

#include "app/Phases.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace BulletEngine {

// fwd
namespace ecs { class World; }

namespace app {

// frame timing passed to hooks
struct FrameContext {
    ecs::World* world = nullptr;

    float deltaTime = 0.0f;         // real time since last frame
    float fixedDeltaTime = 0.0f;    // length of one fixed step
    float alpha = 0.0f;             // 0..1 towards next fixed step
    uint64_t frame = 0;
};

using HookFn = std::function<void(const FrameContext&)>;

// handle to registered hook
using HookId = uint32_t;
inline constexpr HookId INVALID_HOOK = 0;

inline constexpr int DEFAULT_ORDER = 0;

// per-phase hook lists
class Scheduler {
public:
    Scheduler() = default;

    // registration
    HookId add(Phase phase, HookFn fn, int order = DEFAULT_ORDER, std::string name = {});   // lower order runs first
    void remove(HookId id);

    void clear(Phase phase);
    void clear();

    // state
    void setEnabled(HookId id, bool enabled);
    bool isEnabled(HookId id) const;

    void run(Phase phase, const FrameContext& context);

    size_t getCount(Phase phase) const;

private:
    struct Hook {
        HookId id = INVALID_HOOK;
        int order = DEFAULT_ORDER;
        uint32_t sequence = 0;      // breaks order ties
        bool enabled = true;
        std::string name;
        HookFn fn;
    };

    static constexpr size_t PHASE_COUNT = static_cast<size_t>(Phase::Count);

    std::vector<Hook>& bucket(Phase phase) { return m_hooks[static_cast<size_t>(phase)]; }
    const std::vector<Hook>& bucket(Phase phase) const { return m_hooks[static_cast<size_t>(phase)]; }

    Hook* find(HookId id);
    const Hook* find(HookId id) const;

    std::vector<Hook> m_hooks[PHASE_COUNT];

    HookId m_nextId = 1;
    uint32_t m_nextSequence = 0;

    // deferred removal while phase runs
    bool m_running = false;
    std::vector<HookId> m_pendingRemoval;
};

} // namespace app
} // namespace BulletEngine
