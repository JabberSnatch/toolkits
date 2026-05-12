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
        std::atomic<struct Node*> next_node;
        std::atomic<uint32_t> next_tag;
    };

    Node root = { {}, nullptr, 0u };

    std::atomic<Node*> head_node = &root;
    std::atomic<uint32_t> head_tag;
    std::atomic<Node*> tail_node = &root;
    std::atomic<uint32_t> tail_tag;

    void Enqueue()
    {
        Node* candidate_node = ReserveNode_();
        candidate_node->value;
        candidate_node->next_node = nullptr;

        for(;;) {
            Node* local_tail = tail_node;
            Node* local_next = local_tail->next_node;

            if (local_tail == tail_node
                && local_next->next_node == nullptr)
            {
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
    dstk::ObjectPool<Node> node_pool{};
};

} // namespace plltk
