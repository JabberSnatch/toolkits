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
    struct Cell;

    struct CellID {
        Cell* cell = nullptr;
        uint16_t tag = 0;

        bool operator==(CellID const& o_) { return 0 == std::memcmp(this, &o_, sizeof(CellID)); }
        bool operator!=(CellID const& o_) { return !(*this == o_); }
    };

    struct Cell {
        using ValueHandle = uint64_t;

        ValueHandle value = {};
        //std::atomic<struct Cell*> next;
        std::atomic<CellID> next = {};
    };

    //Node* root = { {}, nullptr };

    //std::atomic<Cell*> head = new Cell({}, nullptr);
    //std::atomic<Cell*> tail = head.load();

    std::atomic<CellID> head = CellID{ new Cell(), 0 };
    std::atomic<CellID> tail = head.load();

    void Enqueue()
    {
        Cell* candidate = new Cell();
        candidate->value;
        candidate->next = CellID{};

        for(;;) {
            CellID local_tail = tail;
            CellID local_next = local_tail.cell->next;

            if (local_tail == tail
                && local_next.cell == nullptr)
            {
                if (local_tail.cell->next.compare_exchange_weak(
                        local_next,
                        CellID{ candidate, local_next.tag+1u }))
                {
                    tail.compare_exchange_weak(
                        local_tail,
                        CellID{ candidate, local_tail.tag+1u },
                        std::memory_order_release, std::memory_order_relaxed);
                    break;
                }
                else
                    tail.compare_exchange_weak(
                        local_tail,
                        CellID{ local_next.cell, local_tail.tag+1u });
            }
        }
    }

    Cell::ValueHandle Dequeue()
    {
        Cell::ValueHandle output = 0ull;

        for (;;) {
            CellID local_head = head;
            CellID local_tail = tail;
            CellID local_next = local_head.cell->next;

            if (local_head != head)
                continue;

            if (local_head == local_tail)
            {
                if (local_next.cell == nullptr)
                    break;
                tail.compare_exchange_weak(
                    local_tail,
                    CellID{ local_next.cell, local_tail.tag+1u });
            }
            else
            {
                output = local_next.cell->value;
                if (head.compare_exchange_weak(
                        local_head,
                        CellID{ local_next.cell, local_head.tag+1u }))
                {
                    delete local_head.cell;
                    break;
                }
            }
        }

        return output;
    }

    Cell* ReserveCell_()
    {
        ScopeLock lock{ pool_mutex };
        Cell* cell = cell_pool[cell_pool.Reserve()];
        return cell;
    }

    void ReleaseCell_(Cell* cell_)
    {
        ScopeLock lock{ pool_mutex };
        cell_pool.Release(cell_pool.Find(cell_));
    }

    Mutex pool_mutex{};
    dstk::ObjectPool<Cell> cell_pool{ 1024 * 1024 };
};

} // namespace plltk
