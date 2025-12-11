#ifndef APEXVELOCITY_THREAD_POOL_H
#define APEXVELOCITY_THREAD_POOL_H

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace apex {

/**
 * @brief Simple fixed-size thread pool for background task execution.
 *
 * This is a generic utility class used by batch solvers and other
 * high-throughput components. It is intentionally minimal and does not
 * depend on any external libraries.
 */
class ThreadPool {
public:
    explicit ThreadPool(std::size_t num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief Enqueue a callable for execution on the pool.
     *
     * The callable must be invocable with no arguments and return a value
     * (which may be void). A std::future for the return type is returned.
     */
    template <class F>
    auto enqueue(F&& f)
        -> std::future<typename std::invoke_result<F>::type> {
        using ReturnT = typename std::invoke_result<F>::type;

        auto task = std::make_shared<std::packaged_task<ReturnT()>>(
            std::forward<F>(f));

        std::future<ReturnT> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (stop_) {
                throw std::runtime_error(
                    "ThreadPool::enqueue on stopped ThreadPool");
            }
            tasks_.emplace([task]() { (*task)(); });
        }
        condition_.notify_one();
        return res;
    }

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;

    std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_{false};
};

}  // namespace apex

#endif  // APEXVELOCITY_THREAD_POOL_H


