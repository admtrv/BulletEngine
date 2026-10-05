/*
 * TaskExecutor.cpp
 */

#include "TaskExecutor.h"

#include <exception>
#include <iostream>

namespace BulletEngine {
namespace app {

constexpr size_t WORKERS_SPARE = 1;     // core left to main thread
constexpr size_t WORKERS_LEAST = 1;

TaskExecutor& TaskExecutor::instance()
{
    static TaskExecutor executor;
    return executor;
}

TaskExecutor::~TaskExecutor()
{
    stop();
}

void TaskExecutor::start(size_t workers)
{
    if (!m_workers.empty())
    {
        return;
    }

    if (workers == 0)
    {
        const size_t cores = std::thread::hardware_concurrency();
        workers = cores > WORKERS_SPARE ? cores - WORKERS_SPARE : WORKERS_LEAST;
    }

    {
        const std::lock_guard lock(m_mutex);
        m_stopping = false;
    }

    m_workers.reserve(workers);

    for (size_t i = 0; i < workers; i++)
    {
        m_workers.emplace_back([this]() { worker(); });
    }
}

void TaskExecutor::stop()
{
    {
        const std::lock_guard lock(m_mutex);

        m_stopping = true;
        m_queued = {};
    }

    m_wakeup.notify_all();
    m_workers.clear();      // joins, so nothing reaches finished after it

    const std::lock_guard lock(m_mutex);
    m_finished.clear();
}

void TaskExecutor::submit(Work work, Done done)
{
    if (!work)
    {
        return;
    }

    // nobody to wait for without workers
    if (m_workers.empty())
    {
        work();

        if (done)
        {
            done();
        }

        return;
    }

    {
        const std::lock_guard lock(m_mutex);
        m_queued.emplace(std::move(work), std::move(done));
    }

    m_wakeup.notify_one();
}

void TaskExecutor::drain()
{
    std::vector<Done> ready;

    {
        const std::lock_guard lock(m_mutex);
        ready.swap(m_finished);
    }

    // done may submit again, so mutex is let go of first
    for (const Done& done : ready)
    {
        try
        {
            done();
        }
        catch (const std::exception& error)
        {
            std::cerr << "task result failed: " << error.what() << '\n';
        }
    }
}

void TaskExecutor::worker()
{
    while (true)
    {
        std::pair<Work, Done> task;

        {
            std::unique_lock lock(m_mutex);
            m_wakeup.wait(lock, [this]() { return m_stopping || !m_queued.empty(); });

            if (m_stopping)
            {
                return;
            }

            task = std::move(m_queued.front());
            m_queued.pop();
        }

        try
        {
            task.first();
        }
        catch (const std::exception& error)
        {
            std::cerr << "task failed: " << error.what() << '\n';
            task.second = {};
        }

        const std::lock_guard lock(m_mutex);

        if (task.second)
        {
            m_finished.push_back(std::move(task.second));
        }
    }
}

} // namespace app
} // namespace BulletEngine
