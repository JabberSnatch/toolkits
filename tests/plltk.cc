#include "plltk.hh"

#include <iostream>
#include <thread>
#include <vector>

#define PRODUCER_COUNT 24
#define CONSUMER_COUNT PRODUCER_COUNT

void AtomicStressTest(plltk::AtomicU128& atomic, uint64_t thread_id)
{
    uint64_t value[2];
    value[0] = thread_id;
    value[1] = thread_id;
    atomic.Store(value);
}

void ProducerThread(plltk::ConcurrentQueue& queue)
{
    for (uint32_t index = 0; index < ((1 << 20) / PRODUCER_COUNT); ++index)
        queue.Enqueue();
}

void ConsumerThread(plltk::ConcurrentQueue& queue)
{
    for (uint32_t index = 0; index < ((1 << 20) / CONSUMER_COUNT); ++index)
        queue.Dequeue();
}

int main(int argc, char const** argv)
{
    plltk::ConcurrentQueue queue{};
    plltk::AtomicU128 u128atomic{ 0ull, 0ull };

    for (uint32_t test_index = 0; test_index < 1024; ++test_index)
    {
        std::vector<std::thread> atomics{};
        for (uint64_t index = 1; index < 128; ++index)
            atomics.emplace_back(&AtomicStressTest, std::ref(u128atomic), index);

        for (auto&& thread : atomics)
            if (thread.joinable())
                thread.join();

        if (u128atomic.high != u128atomic.low)
            std::cout << "atomic corruption" << std::endl;
    }
    return 0;

    std::vector<std::thread> producers{};
    for (uint32_t index = 0; index < PRODUCER_COUNT; ++index)
        producers.emplace_back(&ProducerThread, std::ref(queue));

    for (auto&& thread : producers)
        if (thread.joinable())
            thread.join();

    std::vector<std::thread> consumers{};
    for (uint32_t index = 0; index < CONSUMER_COUNT; ++index)
        consumers.emplace_back(&ConsumerThread, std::ref(queue));

    for (auto&& thread : consumers)
        if (thread.joinable())
            thread.join();

    return 0;
}
