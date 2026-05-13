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

    //Node* root = { {}, nullptr };

    std::atomic<Node*> head = new Node({}, nullptr);
    std::atomic<Node*> tail = head.load();

    void Enqueue()
    {
        Node* candidate = new Node();
        candidate->value;
        candidate->next = nullptr;

        for(;;) {
            Node* local_tail = tail;
            Node* local_next = local_tail->next;

            if (local_tail == tail
                && local_next == nullptr)
            {
                if (local_tail->next.compare_exchange_weak(local_next, candidate))
                {
                    tail.compare_exchange_weak(
                        local_tail, candidate,
                        std::memory_order_release, std::memory_order_relaxed);
                    break;
                }
                else
                    tail.compare_exchange_weak(local_tail, local_next);
            }
        }
    }

    Node::ValueHandle Dequeue()
    {
        Node::ValueHandle output = 0ull;

        for (;;) {
            Node* local_head = head;
            Node* local_tail = tail;
            Node* local_next = local_head->next;

            if (local_head != head)
                continue;

            if (local_head == local_tail)
            {
                if (local_next == nullptr)
                    break;
                tail.compare_exchange_weak(local_tail, local_next);
            }
            else
            {
                output = local_next->value;
                if (head.compare_exchange_weak(local_head, local_next))
                {
                    delete local_head;
                    break;
                }
            }
        }

        return output;
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
