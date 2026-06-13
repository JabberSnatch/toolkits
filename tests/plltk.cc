#include "plltk.hh"

#include <iostream>
#include <thread>
#include <vector>

#define PRODUCER_COUNT 24
#define CONSUMER_COUNT PRODUCER_COUNT

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
