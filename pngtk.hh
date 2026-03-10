#pragma once

#include <iostream>
#include "bintk.hh"
#include "numtk.hh"
#include "ziptk.hh"

namespace pngtk
{

#define CompareTag(_stream, tag0, tag1, tag2, tag3) \
    (char)_stream[0] == tag0                        \
        && (char)_stream[1] == tag1                 \
        && (char)_stream[2] == tag2                 \
        && (char)_stream[3] == tag3

enum ColorType : uint8_t {
    ColorType_Greyscale = 0,
    ColorType_Truecolor = 2,
    ColorType_Indexed = 3,
    ColorType_GreyscaleAlpha = 4,
    ColorType_TruecolorAlpha = 6,

    ColorType_TruecolorBit = 2,
    ColorType_AlphaBit = 4,
};

enum InterlaceMode : uint8_t {
    InterlaceMode_None = 0,
    InterlaceMode_Adam7 = 1
};

struct PNGHeader {
    uint32_t width;
    uint32_t height;
    uint8_t bit_depth;
    ColorType color_type;
    uint8_t compression;
    uint8_t filter;
    InterlaceMode interlace;
};

struct PNGFile
{
    PNGHeader header;
    std::vector<numtk::vec4<uint16_t>> pixel_data;
};

inline void LoadPNG(uint8_t const* _stream, size_t _size, PNGFile* _png_file)
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

    PNGHeader header = {};
    std::vector<numtk::vec3<uint8_t>> palette = {};
    std::vector<uint8_t> palette_alpha = {};
    numtk::vec3<uint16_t> alpha_color = {};
    std::vector<numtk::vec4<uint16_t>> buffer = {};
    uint32_t pixel_index = 0;
    uint32_t pixel_count = 0;

    for (;;)
    {
        uint32_t chunk_length = bintk::UnpackBytesBE(4, _stream);
        uint8_t const* tag_start = _stream;
        _stream += 4;
        uint8_t const* block = _stream;

        if (CompareTag(tag_start, 'I', 'H', 'D', 'R'))
        {
            std::cout << "IHDR" << std::endl;

            header.width = bintk::UnpackBytesBE(4, block);
            header.height = bintk::UnpackBytesBE(4, block);
            header.bit_depth = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.color_type = (ColorType)bintk::UnpackBytesBE(1, block);
            header.compression = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.filter = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.interlace = (InterlaceMode)bintk::UnpackBytesBE(1, block);

            pixel_count = header.width * header.height;
            buffer.resize(pixel_count);
        }

        else if (CompareTag(tag_start, 'P', 'L', 'T', 'E'))
        {
            std::cout << "PLTE" << std::endl;

            uint32_t entry_count = chunk_length / 3;
            palette.reserve(entry_count);
            for (uint32_t index = 0u; index < entry_count; ++index)
            {
                numtk::vec3<uint8_t> color = {
                    (uint8_t)bintk::UnpackBytesBE(1, block),
                    (uint8_t)bintk::UnpackBytesBE(1, block),
                    (uint8_t)bintk::UnpackBytesBE(1, block)
                };
                palette.push_back(color);
            }
        }

        else if (CompareTag(tag_start, 't', 'R', 'N', 'S'))
        {
            std::cout << "tRNS" << std::endl;
            if (header.color_type == ColorType_Greyscale)
            {
                alpha_color.x = (uint16_t)bintk::UnpackBytes(2, block);
            }
            else if (header.color_type == ColorType_Truecolor)
            {
                alpha_color = numtk::vec3<uint16_t>{
                    (uint16_t)bintk::UnpackBytes(2, block),
                    (uint16_t)bintk::UnpackBytes(2, block),
                    (uint16_t)bintk::UnpackBytes(2, block)
                };
            }
            else if (header.color_type == ColorType_Indexed)
            {
                uint32_t entry_count = chunk_length;
                palette_alpha.reserve(entry_count);
                for (uint32_t index = 0u; index < entry_count; ++index)
                    palette_alpha.push_back((uint8_t)bintk::UnpackBytes(1, block));
            }
        }

        else if (CompareTag(tag_start, 'I', 'D', 'A', 'T'))
        {
            std::cout << "IDAT" << std::endl;
            ziptk::ExtractZLib(block);
            std::vector<uint8_t> block_data = ziptk::Inflate(block);

            uint8_t const* block_stream = block_data.data();
            uint32_t bit_offset = 0;
            while (block_stream - block_data.data() < (ptrdiff_t)block_data.size()
                   && pixel_index < pixel_count)
            {
                numtk::vec4<uint16_t> pixel_value = {};
                if (header.color_type != ColorType_Indexed)
                {
                    pixel_value[0] = (uint16_t)bintk::UnpackBits(
                        header.bit_depth, block_stream, bit_offset);

                    if (header.color_type & ColorType_TruecolorBit)
                    {
                        pixel_value[1] = (uint16_t)bintk::UnpackBits(
                            header.bit_depth, block_stream, bit_offset);
                        pixel_value[2] = (uint16_t)bintk::UnpackBits(
                            header.bit_depth, block_stream, bit_offset);
                    }

                    if (header.color_type & ColorType_AlphaBit)
                    {
                        pixel_value[3] = (uint16_t)bintk::UnpackBits(
                            header.bit_depth, block_stream, bit_offset);
                    }
                }
                else
                {
                    uint8_t color_index = (uint8_t)bintk::UnpackBits(
                        header.bit_depth, block_stream, bit_offset);
                    numtk::vec3<uint8_t> const& source = palette[color_index];
                    pixel_value[0] = (uint16_t)source[0];
                    pixel_value[1] = (uint16_t)source[1];
                    pixel_value[2] = (uint16_t)source[2];
                    pixel_value[3] = (uint32_t)color_index >= palette_alpha.size()
                        ? 255
                        : (uint16_t)palette_alpha[color_index];
                }

                buffer[pixel_index++] = pixel_value;
            }
        }

        else if (CompareTag(tag_start, 'I', 'E', 'N', 'D'))
        {
            std::cout << "IEND" << std::endl;
        }

        else
        {
            std::cerr << "Unknown tag " << std::string((char const*)_stream, 4) << std::endl;
        }

        _stream += chunk_length + 4;
        if (_stream - stream_start >= (ptrdiff_t)_size)
            break;
    }

    if (_png_file)
    {
        _png_file->header = header;
        _png_file->pixel_data = std::move(buffer);
    }
}

} // namespace pngtk
