/*
 * Scheduler.cpp
 */

#include "Scheduler.h"

#include <algorithm>

namespace BulletEngine {
namespace app {

HookId Scheduler::add(Phase phase, HookFn fn, int order, std::string name)
{
    if (!fn)
    {
        return INVALID_HOOK;
    }

    auto& hooks = bucket(phase);

    Hook hook;
    hook.id = m_nextId++;
    hook.order = order;
    hook.sequence = m_nextSequence++;
    hook.name = std::move(name);
    hook.fn = std::move(fn);

    const HookId id = hook.id;

    // insert sorted, cheaper than sorting whole bucket on every run
    const auto position = std::upper_bound(hooks.begin(), hooks.end(), hook,
        [](const Hook& a, const Hook& b) {
            return a.order != b.order ? a.order < b.order : a.sequence < b.sequence;
        });

    hooks.insert(position, std::move(hook));

    return id;
}

void Scheduler::remove(HookId id)
{
    if (id == INVALID_HOOK)
    {
        return;
    }

    if (m_running)
    {
        m_pendingRemoval.push_back(id);
        return;
    }

    for (auto& hooks : m_hooks)
    {
        const auto it = std::find_if(hooks.begin(), hooks.end(),
            [id](const Hook& hook) { return hook.id == id; });

        if (it != hooks.end())
        {
            hooks.erase(it);
            return;
        }
    }
}

void Scheduler::clear(Phase phase)
{
    bucket(phase).clear();
}

void Scheduler::clear()
{
    for (auto& hooks : m_hooks)
    {
        hooks.clear();
    }

    m_pendingRemoval.clear();
}

void Scheduler::setEnabled(HookId id, bool enabled)
{
    if (Hook* hook = find(id))
    {
        hook->enabled = enabled;
    }
}

bool Scheduler::isEnabled(HookId id) const
{
    const Hook* hook = find(id);
    return hook && hook->enabled;
}

void Scheduler::run(Phase phase, const FrameContext& context)
{
    auto& hooks = bucket(phase);

    m_running = true;

    // hook registered during the run waits for next frame
    const size_t bound = hooks.size();

    for (size_t i = 0; i < bound && i < hooks.size(); i++)
    {
        if (!hooks[i].enabled || !hooks[i].fn)
        {
            continue;
        }

        // copied, bucket may reallocate while the hook runs
        HookFn fn = hooks[i].fn;
        fn(context);
    }

    m_running = false;

    for (HookId id : m_pendingRemoval)
    {
        remove(id);
    }

    m_pendingRemoval.clear();
}

size_t Scheduler::getCount(Phase phase) const
{
    return bucket(phase).size();
}

Scheduler::Hook* Scheduler::find(HookId id)
{
    return const_cast<Hook*>(static_cast<const Scheduler*>(this)->find(id));
}

const Scheduler::Hook* Scheduler::find(HookId id) const
{
    if (id == INVALID_HOOK)
    {
        return nullptr;
    }

    for (const auto& hooks : m_hooks)
    {
        const auto it = std::find_if(hooks.begin(), hooks.end(),
            [id](const Hook& hook) { return hook.id == id; });

        if (it != hooks.end())
        {
            return &(*it);
        }
    }

    return nullptr;
}

} // namespace app
} // namespace BulletEngine
