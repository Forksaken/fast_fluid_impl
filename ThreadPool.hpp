#ifndef THREADPOOL_HPP
#define THREADPOOL_HPP

#include <cstddef>
#include <functional>
#include <future>
#include <map>
#include <queue>
#include <thread>
#include <unordered_map>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stop_token>
#include <tuple>
#include <type_traits>
#include <utility>

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
        using ReturnType = typename std::result_of<F(Args...)>::type;
        std::function<ReturnType(Args...)> func;
        std::tuple<std::decay_t<Args>...> args;
        std::packaged_task<ReturnType()> task;
    public:
        PackagedTask(uint32_t nid, F &&f, Args &&...nargs)
            : BasePackagedTask(nid),
              func(std::forward<F>(f)),
              args(std::make_tuple(std::forward<Args>(nargs)...)),
              task([this] { return std::apply(func, args); }) {}
        std::unique_ptr<BaseFuture> execute() override {
            task();
            return std::make_unique<Future<ReturnType>>(id, task.get_future());
        }
    };

    std::vector<std::jthread> workers;
    std::unordered_map<int, std::unique_ptr<BaseFuture>> futures;
    std::atomic_uint32_t nextId;
    std::queue<std::unique_ptr<BasePackagedTask>> tasks;

    std::mutex queueMutex;
    std::condition_variable condition;
    std::condition_variable futureCondition;

public:
    ThreadPool() = default;

    void init(int numThreads) {
        nextId = 0;
        for (int i = 0; i < numThreads; i++) {
            workers.emplace_back([this](std::stop_token stoken) {
                this->workerThread(stoken);
            });
        }
    }

    ThreadPool(size_t numThreads) {
        nextId = 0;
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
        uint32_t id = nextId++;
        auto task = std::make_unique<PackagedTask<F, Args...>>(
            id, std::forward<F>(f), std::forward<Args>(args)...); {
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
        futures[id]->wait();
    }

    template <typename T>
    T get(int id) {
        std::unique_lock<std::mutex> lock(queueMutex);
        futureCondition.wait(lock, [this, id] {
            return futures.find(id) != futures.end();
        });
        auto future = dynamic_cast<Future<T> *>(futures[id].get());
        if (!future) {
            throw std::runtime_error("Invalid type for future");
        }
        return future->get();
    }

    void erase(int id) {
        std::lock_guard<std::mutex> lock(queueMutex);
        futures.erase(id);
    }

    void stopAll() {
        condition.notify_all();
        for (std::jthread &worker : workers) {
            if (worker.joinable()) worker.request_stop();
        }
    }

private:
    void workerThread(std::stop_token stoken) {
        while (true) {
            std::unique_ptr<BasePackagedTask> task; {
                std::unique_lock<std::mutex> lock(queueMutex);
                condition.wait(lock, [this, &stoken] {
                    return stoken.stop_requested() || !tasks.empty();
                });
                if (stoken.stop_requested() && tasks.empty())
                    return;
                if (!tasks.empty()) {
                    task = std::move(tasks.front());
                    tasks.pop();
                }
            }
            if (task) {
                auto future = task->execute(); {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    futures[future->id] = std::move(future);
                }
                futureCondition.notify_all();
            }
        }
    }
};

#endif // THREADPOOL_HPP