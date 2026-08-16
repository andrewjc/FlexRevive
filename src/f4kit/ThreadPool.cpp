// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "f4kit/ThreadPool.h"
#include "f4kit/Log.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace f4kit::threads {

namespace {

struct Pool {
    std::vector<std::thread> workers;

    std::mutex m;
    std::condition_variable cvWork; // published job, woken workers
    std::condition_variable cvDone; // last worker out, woken caller

    // Written under m, then read without it. Acquiring m publishes them, and nothing writes
    // them again until every worker has finished.
    const std::function<void(int)>* job = nullptr;
    int count = 0;
    int grain = 1;

    std::atomic<int> cursor{0};       // next unclaimed chunk
    unsigned long long generation = 0; // bumped per job; distinguishes a real wake from a spurious one
    int running = 0;                   // workers still inside the current job
};

// Never deleted; see the header.
std::atomic<Pool*> s_pool{nullptr};
std::once_flag s_once;

// Take chunks until there are none left. Shared by the workers and by the calling thread.
void Drain(Pool& p)
{
    for (;;) {
        const int begin = p.cursor.fetch_add(p.grain, std::memory_order_relaxed);
        if (begin >= p.count)
            return;
        const int end = std::min(begin + p.grain, p.count);
        for (int k = begin; k < end; ++k)
            (*p.job)(k);
    }
}

void WorkerMain(Pool* pool)
{
    Pool& p = *pool;
    unsigned long long seen = 0;

    for (;;) {
        {
            std::unique_lock<std::mutex> lock(p.m);
            p.cvWork.wait(lock, [&] { return p.generation != seen; });
            seen = p.generation;
        }

        Drain(p);

        {
            std::lock_guard<std::mutex> lock(p.m);
            if (--p.running == 0)
                p.cvDone.notify_one();
        }
    }
}

} // namespace

void Start(int totalThreads)
{
    std::call_once(s_once, [totalThreads] {
        const unsigned hw = std::thread::hardware_concurrency();

        int total = totalThreads;
        if (total <= 0) {
            // Two cores of headroom for the game's own render and worker threads.
            total = (hw > 3) ? int(hw) - 2 : 1;
        }
        total = std::max(1, std::min(total, 16));

        const int extra = total - 1;
        if (extra <= 0) {
            log::Write("solver thread pool: disabled, stepping on the calling thread only");
            return;
        }

        auto* p = new (std::nothrow) Pool();
        if (!p) {
            log::Write("solver thread pool: out of memory, falling back to single-threaded");
            return;
        }

        p->workers.reserve(size_t(extra));
        for (int i = 0; i < extra; ++i) {
            try {
                p->workers.emplace_back(WorkerMain, p);
            } catch (...) {
                // Carry on with whatever started.
                break;
            }
        }

        if (p->workers.empty()) {
            log::Write("solver thread pool: no threads could be started, single-threaded");
            return; // p is leaked, matching its normal lifetime
        }

        s_pool.store(p, std::memory_order_release);
        log::Write("solver thread pool: %d worker threads plus the game thread "
                   "(%u hardware threads)", int(p->workers.size()), hw);
    });
}

int Workers()
{
    Pool* p = s_pool.load(std::memory_order_acquire);
    return p ? int(p->workers.size()) : 0;
}

void ParallelFor(int count, int grain, const std::function<void(int)>& body)
{
    if (count <= 0)
        return;

    Pool* pool = s_pool.load(std::memory_order_acquire);
    if (!pool) {
        for (int k = 0; k < count; ++k)
            body(k);
        return;
    }

    Pool& p = *pool;
    {
        std::lock_guard<std::mutex> lock(p.m);
        p.job = &body;
        p.count = count;
        p.grain = std::max(grain, 1);
        p.cursor.store(0, std::memory_order_relaxed);
        p.running = int(p.workers.size());
        ++p.generation;
    }
    p.cvWork.notify_all();

    // The caller drains too, and on a light frame often finishes the job before the workers
    // wake at all.
    Drain(p);

    std::unique_lock<std::mutex> lock(p.m);
    p.cvDone.wait(lock, [&] { return p.running == 0; });
    p.job = nullptr;
}

}
