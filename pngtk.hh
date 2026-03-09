#pragma once

#include <iostream>
#include "bintk.hh"

namespace pngtk
{

#define CompareTag(_stream, tag0, tag1, tag2, tag3) \
    (char)_stream[0] == tag0                        \
        && (char)_stream[1] == tag1                 \
        && (char)_stream[2] == tag2                 \
        && (char)_stream[3] == tag3

inline void LoadPNG(uint8_t const* _stream, size_t _size)
{
    uint8_t const* stream_start = _stream;

    uint64_t signature =
        (uint64_t)bintk::UnpackBytesBE(4, _stream) << 32
        | (uint64_t)bintk::UnpackBytesBE(4, _stream);

    if (signature != 0x89504E470D0A1A0Aull)
    {
        std::cout << "Invalid signature" << std::endl;
        return;
    }

    for (;;)
    {
        uint32_t chunk_length = bintk::UnpackBytesBE(4, _stream);
        if (CompareTag(_stream, 'I', 'H', 'D', 'R'))
        {
            std::cout << "IHDR" << std::endl;
        }
        else if (CompareTag(_stream, 'P', 'L', 'T', 'E'))
        {
            std::cout << "PLTE" << std::endl;
        }
        else if (CompareTag(_stream, 'I', 'D', 'A', 'T'))
        {
            std::cout << "IDAT" << std::endl;
        }
        else if (CompareTag(_stream, 'I', 'E', 'N', 'D'))
        {
            std::cout << "IEND" << std::endl;
        }
        else if (CompareTag(_stream, 't', 'R', 'N', 'S'))
        {
            std::cout << "tRNS" << std::endl;
        }
        else
        {
            std::cerr << "Unknown tag " << std::string((char const*)_stream, 4) << std::endl;
        }

        _stream += chunk_length + 8;
        if (_stream - stream_start >= (ptrdiff_t)_size)
            break;
    }
}

} // namespace pngtk
