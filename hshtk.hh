#pragma once

namespace hshtk
{

inline uint32_t Hash32_Murmur3(uint8_t const* data_, size_t size_, uint32_t seed = 0ull)
{
    uint32_t hash = seed;
    uint32_t key = 0u;

    static auto const Kernel = [](uint32_t key) -> uint32_t {
        key *= 0xcc9e2d51u;
        key = (key << 15) | (key >> 17);
        return key * 0x1b873593u;
    };

    for (uint32_t index = 0; index < size_/4; ++index)
    {
        std::memcpy(&key, data_, 4);
        data_ += 4;
        hash ^= Kernel(key);
        hash = (hash << 13) | (hash >> 19);
        hash = hash * 5 + 0xe6546b64;
    }

    key = 0u;
    for (uint32_t index = 0; index < size_%4; ++index)
        key = (key << 8) | data_[index];

    hash ^= Kernel(key);
    hash ^= size_;
    hash ^= hash >> 16;
    hash *= 0x85ebca6bu;
    hash ^= hash >> 13;
    hash *= 0xc2b2ae35u;
    hash ^= hash >> 16;
    return hash;
}

} // namespace hshtk
