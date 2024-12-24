#ifndef THREADPOOL_HPP
#define THREADPOOL_HPP

#include <cstddef>
#include <functional>
#include <future>
#include <unordered_map>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stop_token>
#include <tuple>
#include <type_traits>
#include <utility>
#include <memory>
#include <vector>
#include <stdexcept>

class ThreadPool {
private:
    class BaseFuture {
    public:
        virtual void wait() = 0;
        uint32_t id;
        BaseFuture(uint32_t nid) : id(nid) {}
        virtual ~BaseFuture() = default;
    };

    template <class T>
    class Future : public BaseFuture {
    private:
        std::future<T> future;
    public:
        Future(uint32_t nid, std::future<T> &&f) : BaseFuture(nid), future(std::move(f)) {}
        void wait() override { future.wait(); }
        T get() { return future.get(); }
    };

    std::vector<std::thread> workers;
    std::queue<std::future<void>> tasks;
    std::unordered_map<int, std::unique_ptr<BaseFuture>> futures;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::condition_variable futureCondition;
    std::atomic_uint32_t nextId{0};
    std::atomic<bool> stop_flag{false};
    std::atomic<int64_t> active_tasks{0};
    std::mutex wait_mutex;
    std::condition_variable wait_condition;

public:
    ThreadPool() = default;

    ThreadPool(size_t numThreads) {
        workers.reserve(numThreads);
        for (int i = 0; i < numThreads; ++i) {
            workers.emplace_back(&ThreadPool::workerThread, this);
        }
        stop_flag = false;
        active_tasks = 0;
    }

    ~ThreadPool() {
        stopAll();
    }

    template <typename F, class... Args>
    int enqueue(F& f, Args &&...args) {
        uint32_t id = nextId.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(queueMutex);
        ++active_tasks;
        tasks.emplace(std::async(std::launch::deferred, f, args...));
        condition.notify_one();
        return id;
    }

    void wait(int id) {
        std::unique_lock<std::mutex> lock(queueMutex);
        futureCondition.wait(lock, [this, id] {
            return futures.find(id) != futures.end();
        });
        if (futures.find(id) != futures.end()) {
            futures[id]->wait();
        }
    }

    template <typename T>
    T get(int id) {
        std::unique_lock<std::mutex> lock(queueMutex);
        futureCondition.wait(lock, [this, id] {
            return futures.find(id) != futures.end();
        });
        auto it = futures.find(id);
        if (it == futures.end()) {
            throw std::runtime_error("Task id not found");
        }
        auto futurePtr = dynamic_cast<Future<T>*>(it->second.get());
        if (!futurePtr) {
            throw std::runtime_error("Invalid type for future");
        }
        T result = futurePtr->get();
        futures.erase(it);
        return result;
    }

    void erase(int id) {
        std::lock_guard<std::mutex> lock(queueMutex);
        futures.erase(id);
    }

    void wait_all() {
        std::unique_lock<std::mutex> lock(queueMutex);
        wait_condition.wait(lock, [this]{
            return tasks.empty() && active_tasks.load() < 30;
        });
    }

    void stopAll() {
        {
            wait_all();
            stop_flag = true;
        }

        for (auto& worker : workers) {
            condition.notify_all();
            if (worker.joinable())
                worker.join();
        }
    }

private:
    void workerThread() {
        while (!stop_flag) {
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                condition.wait(lock, [this]() -> bool {
                    return stop_flag || !tasks.empty();
                });
                if (stop_flag && tasks.empty())
                    break;
                if (!tasks.empty()) {
                    auto task = std::move(tasks.front());
                    tasks.pop();
                    lock.unlock();
                    task.get();
                    --active_tasks;
                    wait_condition.notify_all();
                }
            }
        }
    }
};

#endif // THREADPOOL_HPP