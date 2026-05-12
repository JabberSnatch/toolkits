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


    Node* ReserveNode() {
        uint64_t handle = next_free_node.fetch_add(1);

        pool_mutex.Lock();
        if (handle > node_pool.blocks.size() * node_pool.block_size)
        {
        }
        pool_mutex.Unlock();
    }

    Mutex pool_mutex{};
    dstk::BlockVector node_pool{ sizeof(Node), 256ull };
    std::atomic<uint64_t> pool_size = 0ull;
    std::atomic<uint64_t> next_free_node = 0ull;
};

} // namespace plltk
