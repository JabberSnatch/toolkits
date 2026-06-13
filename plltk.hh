#pragma once

#include <dstk.hh>

#include <atomic>

#define BIG_CELLID
#define ATOMIC_REF

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

#ifdef BIG_CELLID
    struct CellID {
        alignas(2 * sizeof(void*)) Cell* cell = nullptr;

#ifndef ATOMIC_REF
        uint16_t tag = 0;
#else
        uint64_t tag = 0;
#endif

        bool operator==(CellID const& o_) { return 0 == std::memcmp(this, &o_, sizeof(CellID)); }
        bool operator!=(CellID const& o_) { return !(*this == o_); }
    };

#else

    using CellID = uint64_t;
    static CellID CellID_Pack(Cell* cell, uint16_t tag) {
        return (((CellID)cell) & 0xffffffffffffull) << 16 | (CellID)tag;
    }
    static Cell* CellID_Cell(CellID cellID) {
        return (Cell*)(cellID >> 16);
    }
    static uint16_t CellID_Tag(CellID cellID) {
        return (uint16_t)(cellID & 0xffffull);
    }
#endif

    struct Cell {
        using ValueHandle = uint64_t;

        ValueHandle value = {};
        std::atomic<CellID> next = {};
    };

#ifdef BIG_CELLID
#ifndef ATOMIC_REF
    std::atomic<CellID> head = CellID{ new Cell(), 0 };
#else
    alignas(2 * sizeof(void*)) CellID head_storage = CellID{ new Cell(), 0 };
    std::atomic_ref<CellID> head{ head_storage };
#endif
#else
    std::atomic<CellID> head = CellID_Pack(new Cell(), 0);
#endif

#ifndef ATOMIC_REF
    std::atomic<CellID> tail = head.load();
#else
    alignas(2 * sizeof(void*)) CellID tail_storage = head_storage;
    std::atomic_ref<CellID> tail{ tail_storage };
#endif

    void Enqueue()
    {
        Cell* candidate = new Cell();
        candidate->value;
        candidate->next = CellID{};

        for(;;) {
            CellID local_tail = tail;

#ifdef BIG_CELLID
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
#else
            CellID local_next = CellID_Cell(local_tail)->next;

            if (local_tail == tail
                && CellID_Cell(local_next) == nullptr)
            {
                if (CellID_Cell(local_tail)->next.compare_exchange_weak(
                        local_next,
                        CellID_Pack(candidate, CellID_Tag(local_next)+1u)))
                {
                    tail.compare_exchange_weak(
                        local_tail,
                        CellID_Pack(candidate, CellID_Tag(local_tail)+1u),
                        std::memory_order_release, std::memory_order_relaxed);
                    break;
                }
                else
                    tail.compare_exchange_weak(
                        local_tail,
                        CellID_Pack(CellID_Cell(local_next), CellID_Tag(local_tail)+1u));
            }
#endif
        }
    }

    Cell::ValueHandle Dequeue()
    {
        Cell::ValueHandle output = 0ull;

        for (;;) {
            CellID local_head = head;
            CellID local_tail = tail;

#ifdef BIG_CELLID
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
#else
            CellID local_next = CellID_Cell(local_head)->next;

            if (local_head != head)
                continue;

            if (local_head == local_tail)
            {
                if (CellID_Cell(local_next) == nullptr)
                    break;
                tail.compare_exchange_weak(
                    local_tail,
                    CellID_Pack(CellID_Cell(local_next), CellID_Tag(local_tail)+1u));
            }
            else
            {
                output = CellID_Cell(local_next)->value;
                if (head.compare_exchange_weak(
                        local_head,
                        CellID_Pack(CellID_Cell(local_next), CellID_Tag(local_head)+1u)))
                {
                    delete CellID_Cell(local_head);
                    break;
                }
            }
#endif

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
