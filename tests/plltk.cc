#include "plltk.hh"

#include <thread>
#include <vector>

void ProducerThread(plltk::ConcurrentQueue& queue)
{
    for (uint32_t index = 0; index < (1 << 17); ++index)
        queue.Enqueue();
}

int main(int argc, char const** argv)
{
    plltk::ConcurrentQueue queue{};

    std::vector<std::thread> producers{};
    for (uint32_t index = 0; index < 32; ++index)
    {
        producers.emplace_back(&ProducerThread, std::ref(queue));
    }

    for (auto&& thread : producers)
    {
        if (thread.joinable())
            thread.join();
    }

    return 0;
}
