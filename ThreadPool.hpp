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

    class BasePackagedTask {
    public:
        virtual std::unique_ptr<BaseFuture> execute() = 0;
        virtual ~BasePackagedTask() = default;
        uint32_t id;
        BasePackagedTask(uint32_t nid) : id(nid) {}
    };

    template <class F, class... Args>
    class PackagedTask : public BasePackagedTask {
    private:
        using ReturnType = typename std::invoke_result<F, Args...>::type;
        std::function<ReturnType()> func;
        std::packaged_task<ReturnType()> task;
    public:
        PackagedTask(uint32_t nid, F &&f, Args &&...nargs)
            : BasePackagedTask(nid),
              func(std::bind(std::forward<F>(f), std::forward<Args>(nargs)...)),
              task(func) {}

        std::unique_ptr<BaseFuture> execute() override {
            auto fut = task.get_future();
            task();
            return std::make_unique<Future<ReturnType>>(id, std::move(fut));
        }
    };

    std::vector<std::jthread> workers;
    std::queue<std::unique_ptr<BasePackagedTask>> tasks;
    std::unordered_map<int, std::unique_ptr<BaseFuture>> futures;
    std::mutex queueMutex;
    std::condition_variable condition;
    std::condition_variable futureCondition;
    std::atomic_uint32_t nextId{0};
    std::atomic<bool> stop_flag{false};

public:
    ThreadPool() = default;

    ThreadPool(size_t numThreads) {
        for (size_t i = 0; i < numThreads; i++) {
            workers.emplace_back([this](std::stop_token stoken) {
                this->workerThread(stoken);
            });
        }
    }

    ~ThreadPool() {
        stopAll();
    }

    template <class F, class... Args>
    int enqueue(F &&f, Args &&...args) {
        uint32_t id = nextId.fetch_add(1, std::memory_order_relaxed);
        auto task = std::make_unique<PackagedTask<F, Args...>>(
            id, std::forward<F>(f), std::forward<Args>(args)...);
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            tasks.push(std::move(task));
        }
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

    void stopAll() {
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            stop_flag.store(true);
        }
        condition.notify_all();
        for (auto& worker : workers) {
            if (worker.joinable()) worker.request_stop();
        }
    }

private:
    void workerThread(std::stop_token stoken) {
        while (!stoken.stop_requested()) {
            std::unique_ptr<BasePackagedTask> task;
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                condition.wait(lock, [this, &stoken] {
                    return stoken.stop_requested() || !tasks.empty();
                });
                if (stoken.stop_requested() && tasks.empty())
                    break;
                if (!tasks.empty()) {
                    task = std::move(tasks.front());
                    tasks.pop();
                }
            }
            if (task) {
                try {
                    auto future = task->execute();
                    {
                        std::lock_guard<std::mutex> lock(queueMutex);
                        futures[future->id] = std::move(future);
                    }
                    futureCondition.notify_all();
                } catch (...) {
                    // parsing exceptions, can be ignored
                }
            }
        }
    }
};

#endif // THREADPOOL_HPP