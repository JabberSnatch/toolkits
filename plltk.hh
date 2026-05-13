#pragma once

#include <dstk.hh>

#include <atomic>

namespace plltk
{

struct Mutex
{
    std::atomic_flag state{};

    void Lock() {
        while (state.test_and_set(std::memory_order_acquire))
            state.wait(true, std::memory_order_relaxed);
    }

    void Unlock() {
        state.clear(std::memory_order_release);
        state.notify_one();
    }
};

struct ScopeLock
{
    ScopeLock(Mutex& mutex_) : mutex{ mutex_ } { mutex.Lock(); }
    ~ScopeLock() { mutex.Unlock(); }

    Mutex& mutex;
};

struct ConcurrentQueue
{
    struct Node {
        using ValueHandle = uint64_t;

        ValueHandle value;
        std::atomic<struct Node*> next;
    };

    Node root = { {}, nullptr };

    std::atomic<Node*> head = &root;
    std::atomic<Node*> tail = &root;

    void Enqueue()
    {
        Node* candidate = ReserveNode_();
        candidate->value;
        candidate->next = nullptr;

        for(;;) {
            Node* local_tail = tail;
            Node* local_next = local_tail->next;

            if (local_tail == tail
                && local_next == nullptr)
            {
                if (local_tail->next.compare_exchange_weak(local_next, candidate))
                    if (tail.compare_exchange_weak(
                            local_tail, candidate,
                            std::memory_order_release, std::memory_order_relaxed))
                        break;
                else
                    tail.compare_exchange_weak(local_tail, local_next);
            }
        }
    }

    Node* ReserveNode_()
    {
        ScopeLock lock{ pool_mutex };
        Node* node = node_pool[node_pool.Reserve()];
        return node;
    }

    void ReleaseNode_(Node* node_)
    {
        ScopeLock lock{ pool_mutex };
        node_pool.Release(node_pool.Find(node_));
    }

    Mutex pool_mutex{};
    dstk::ObjectPool<Node> node_pool{ 1024 * 1024 };
};

} // namespace plltk
