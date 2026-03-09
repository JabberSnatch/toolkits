#pragma once

namespace bintk
{

inline std::uint16_t ReverseBits(std::uint16_t bits)
{
    bits = ((bits & 0x00ff) << 8) | ((bits & 0xff00) >> 8);
    bits = ((bits & 0x0f0f) << 4) | ((bits & 0xf0f0) >> 4);
    bits = ((bits & 0x3333) << 2) | ((bits & 0xcccc) >> 2);
    bits = ((bits & 0x5555) << 1) | ((bits & 0xaaaa) >> 1);
    return bits;
}

inline std::uint32_t UnpackBytes(std::uint32_t count, std::uint8_t const*& stream)
{
    std::uint32_t output = 0;
    if (count > 0)
        output |= *stream++;
    if (count > 1)
        output |= ((uint32_t)*stream++ << 8);
    if (count > 2)
        output |= ((uint32_t)*stream++ << 16);
    if (count > 3)
        output |= ((uint32_t)*stream++ << 24);
    return output;
}

inline void SkipBits(std::uint32_t count, std::uint8_t const*& stream, std::uint32_t& offset)
{
    std::uint32_t head = std::min(8 - offset, count)%8;
    std::uint32_t tail = (count > head) ? (count-head)%8 : 0;
    std::uint32_t body = count - head - tail;

    offset += head;
    if (offset > 7)
    {
        offset = offset % 8;
        ++stream;
    }

    stream += body/8;
    offset += tail;
}

inline std::uint32_t PeekBits(std::uint32_t count, std::uint8_t const* stream, std::uint32_t offset)
{
    std::uint32_t head = std::min(8 - offset, count)%8;
    std::uint32_t tail = (count > head) ? (count-head)%8 : 0;
    std::uint32_t body = count - head - tail;

    if (count == 0)
        return 0;

    std::uint32_t v = 0;
    if (head != 0)
    {
        v |= ((*stream >> offset) & ((1 << head)-1));
        offset += head;
        if (offset > 7)
        {
            offset = offset % 8;
            ++stream;
        }
    }

    if (body != 0)
        v |= UnpackBytes(body/8, stream) << head;

    if (tail != 0)
    {
        v |= (*stream & ((1 << tail)-1)) << (head + body);
        offset += tail;
    }

    return v;
}

inline std::uint32_t UnpackBits(std::uint32_t count, std::uint8_t const*& stream, std::uint32_t& offset)
{
    std::uint32_t head = std::min(8 - offset, count)%8;
    std::uint32_t tail = (count > head) ? (count-head)%8 : 0;
    std::uint32_t body = count - head - tail;

    if (count == 0)
        return 0;

    std::uint32_t v = 0;
    if (head != 0)
    {
        v |= ((*stream >> offset) & ((1 << head)-1));
        offset += head;
        if (offset > 7)
        {
            offset = offset % 8;
            ++stream;
        }
    }

    if (body != 0)
        v |= UnpackBytes(body/8, stream) << head;

    if (tail != 0)
    {
        v |= (*stream & ((1 << tail)-1)) << (head + body);
        offset += tail;
    }

    return v;
}

} // namespace bintk
