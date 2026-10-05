/*
 * TaskExecutor.h
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace BulletEngine {
namespace app {

class TaskExecutor {
public:
    using Work = std::function<void()>;     // runs on worker
    using Done = std::function<void()>;     // runs on main thread

    static TaskExecutor& instance();

    // workers
    void start(size_t workers = 0);     // zero asks hardware how many it has
    void stop();                        // waits for what runs, drops what waits

    // produce runs on worker, its result reaches apply on main thread
    template<class Produce, class Apply>
    void run(Produce produce, Apply apply)
    {
        auto result = std::make_shared<std::invoke_result_t<Produce>>();

        submit([produce = std::move(produce), result]() { *result = produce(); },
               [apply = std::move(apply), result]() { apply(std::move(*result)); });
    }

    void drain();   // runs every Done collected so far, once per frame

private:
    TaskExecutor() = default;
    ~TaskExecutor();

    TaskExecutor(const TaskExecutor&) = delete;
    TaskExecutor& operator=(const TaskExecutor&) = delete;

    void submit(Work work, Done done);
    void worker();

    std::vector<std::jthread> m_workers;

    mutable std::mutex m_mutex;
    std::condition_variable m_wakeup;

    std::queue<std::pair<Work, Done>> m_queued;
    std::vector<Done> m_finished;

    bool m_stopping = false;
};

} // namespace app
} // namespace BulletEngine
