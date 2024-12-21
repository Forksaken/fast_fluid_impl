#ifndef THREADS_POOL_HPP
#define THREADS_POOL_HPP

#include <vector>
#include <thread>
#include <queue>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <atomic>

class ThreadPool {
public:
    explicit ThreadPool(size_t n) : stop_(false) {
        workers_.reserve(n);
        for (size_t i=0;i<n;++i) {
            workers_.emplace_back([this](){
                for(;;){
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lk(qmtx_);
                        cv_.wait(lk,[this]{return stop_||!tasks_.empty();});
                        if(stop_&&tasks_.empty()) return;
                        task=std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                }
            });
        }
    }
    template<class F> void enqueue(F&& f){
        {
            std::unique_lock<std::mutex> lk(qmtx_);
            tasks_.push(std::function<void()>(f));
        }
        cv_.notify_one();
    }
    ~ThreadPool(){
        {
            std::unique_lock<std::mutex> lk(qmtx_);
            stop_=true;
        }
        cv_.notify_all();
        for(auto &w:workers_) if(w.joinable()) w.join();
    }
private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex qmtx_;
    std::condition_variable cv_;
    bool stop_;
};

#endif