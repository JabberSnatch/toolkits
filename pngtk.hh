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

enum FilterMode {
    FilterMode_None = 0,
    FilterMode_Sub = 1,
    FilterMode_Up = 2,
    FilterMode_Average = 3,
    FilterMode_Paeth = 4,
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
        std::cerr << "Error : Invalid signature" << std::endl;
        return;
    }

    PNGHeader header = {};
    std::vector<numtk::vec3<uint8_t>> palette = {};
    std::vector<uint8_t> palette_alpha = {};
    bool alpha_color_set = false;
    numtk::vec3<uint16_t> alpha_color = {};
    std::vector<uint8_t> data_stream = {};

    std::vector<numtk::vec4<uint16_t>> buffer = {};
    uint32_t pixel_count = 0;
    uint32_t pixel_channel_count = 0;
    uint32_t pixel_byte_stride = 0;

    for (;;)
    {
        uint32_t chunk_length = bintk::UnpackBytesBE(4, _stream);
        uint8_t const* tag_start = _stream;
        _stream += 4;
        uint8_t const* block = _stream;

        if (CompareTag(tag_start, 'I', 'H', 'D', 'R'))
        {
            header.width = bintk::UnpackBytesBE(4, block);
            header.height = bintk::UnpackBytesBE(4, block);
            header.bit_depth = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.color_type = (ColorType)bintk::UnpackBytesBE(1, block);
            header.compression = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.filter = (uint8_t)bintk::UnpackBytesBE(1, block);
            header.interlace = (InterlaceMode)bintk::UnpackBytesBE(1, block);

            if (header.interlace)
                std::cerr << "Error : Unsupported interlace mode" << std::endl;

            pixel_count = header.width * header.height;
            pixel_channel_count =
                ((header.color_type & ColorType_TruecolorBit) ? 3 : 1)
                + ((header.color_type & ColorType_AlphaBit) ? 1 : 0);

            if (header.color_type != ColorType_Indexed)
                pixel_byte_stride = header.bit_depth < 8
                    ? 1
                    : header.bit_depth * pixel_channel_count;
            else
                pixel_byte_stride = 1;

            buffer.resize(pixel_count);
        }

        else if (CompareTag(tag_start, 'P', 'L', 'T', 'E'))
        {
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
            if (header.color_type == ColorType_Greyscale)
            {
                alpha_color.x = (uint16_t)bintk::UnpackBytesBE(2, block);
                alpha_color_set = true;
            }
            else if (header.color_type == ColorType_Truecolor)
            {
                alpha_color = numtk::vec3<uint16_t>{
                    (uint16_t)bintk::UnpackBytesBE(2, block),
                    (uint16_t)bintk::UnpackBytesBE(2, block),
                    (uint16_t)bintk::UnpackBytesBE(2, block)
                };
                alpha_color_set = true;
            }
            else if (header.color_type == ColorType_Indexed)
            {
                uint32_t entry_count = chunk_length;
                palette_alpha.reserve(entry_count);
                for (uint32_t index = 0u; index < entry_count; ++index)
                    palette_alpha.push_back((uint8_t)bintk::UnpackBytesBE(1, block));
            }
        }

        else if (CompareTag(tag_start, 'I', 'D', 'A', 'T'))
        {
            std::copy(block, block+chunk_length,
                      std::back_inserter(data_stream));
        }

        else if (CompareTag(tag_start, 'I', 'E', 'N', 'D'))
        {
        }

        else
        {
            std::cerr << "Error : Unknown tag " << std::string((char const*)_stream, 4) << std::endl;
        }

        _stream += chunk_length + 4;
        if (_stream - stream_start >= (ptrdiff_t)_size)
            break;
    }

    {
        uint8_t const* block = data_stream.data();

        ziptk::ZLibHeader zlib_header = ziptk::ExtractZLib(block);
        (void)zlib_header;
        std::vector<uint8_t> block_data = ziptk::Inflate(block);

        uint8_t const* block_stream = block_data.data();
        uint32_t bit_offset = 0;

        std::vector<uint8_t> filtered_scanline[2];
        uint32_t scanline_byte_count = (header.bit_depth * header.width + 7) / 8;
        filtered_scanline[0].resize(scanline_byte_count);
        filtered_scanline[1].resize(scanline_byte_count);
        uint8_t const* previous_scanline = nullptr;

        for (uint32_t scanline_index = 0u; scanline_index < header.height; ++scanline_index)
        {
            if (block_stream - block_data.data() >= (ptrdiff_t)block_data.size())
                break;

            FilterMode filter = (FilterMode)bintk::UnpackBytesBE(1, block_stream);
            uint8_t const* scanline = block_stream;
            if ((filter == FilterMode_Up || filter == FilterMode_Average)
                && !previous_scanline)
                filter = (FilterMode)((uint32_t)filter & 1); // Up -> None, Average -> Sub

            if (filter == FilterMode_Paeth)
            {
                uint8_t* recon = filtered_scanline[scanline_index & 1].data();
                scanline = recon;

                static auto PaethPredictor = [](
                    uint8_t const* scanline,
                    uint8_t const* prev_scanline,
                    uint32_t byte_index,
                    uint32_t pixel_byte_stride)
                {
                    int16_t a = (int16_t)(
                        byte_index >= pixel_byte_stride
                        ? scanline[byte_index - pixel_byte_stride]
                        : (uint8_t)0);

                    int16_t b = (int16_t)(
                        prev_scanline
                        ? prev_scanline[byte_index]
                        : (uint8_t)0);

                    int16_t c = (int16_t)(
                        prev_scanline && byte_index >= pixel_byte_stride
                        ? prev_scanline[byte_index - pixel_byte_stride]
                        : (uint8_t)0);

                    int16_t p = a + b - c;
                    int16_t pa = std::abs(p - a);
                    int16_t pb = std::abs(p - b);
                    int16_t pc = std::abs(p - c);
                    if (pa <= pb && pa <= pc) return a;
                    else if (pb <= pc) return b;
                    else return c;
                };

                for (uint32_t byte_index = 0;
                     byte_index < scanline_byte_count;
                     ++byte_index)
                {
                    recon[byte_index] = block_stream[byte_index]
                        + PaethPredictor(scanline, previous_scanline,
                                         byte_index, pixel_byte_stride);
                }
            }

            else if (filter != FilterMode_None)
            {
                uint8_t* recon = filtered_scanline[scanline_index & 1].data();
                scanline = recon;

                std::memcpy(recon, block_stream, pixel_byte_stride);

                for (uint32_t byte_index = pixel_byte_stride;
                     byte_index < scanline_byte_count;
                     ++byte_index)
                {
                    if (filter == FilterMode_Sub)
                    {
                        recon[byte_index] = block_stream[byte_index]
                            + scanline[byte_index - pixel_byte_stride];
                    }

                    else if (filter == FilterMode_Up)
                    {
                        recon[byte_index] = block_stream[byte_index]
                            + previous_scanline[byte_index];
                    }

                    else if (filter == FilterMode_Average)
                    {
                        recon[byte_index] = block_stream[byte_index]
                            + (uint8_t)(
                                ((uint16_t)scanline[byte_index - pixel_byte_stride]
                                 + (uint16_t)previous_scanline[byte_index]) / 2);
                    }
                }
            }

            uint8_t const* current_byte = scanline;
            for (uint32_t pixel_index = 0u; pixel_index < header.width; ++pixel_index)
            {
                numtk::vec4<uint16_t> pixel_value = {};
                if (header.color_type != ColorType_Indexed)
                {
                    pixel_value[0] = (uint16_t)bintk::UnpackBitsBE(
                        header.bit_depth, current_byte, bit_offset);

                    if (header.color_type & ColorType_TruecolorBit)
                    {
                        pixel_value[1] = (uint16_t)bintk::UnpackBitsBE(
                            header.bit_depth, current_byte, bit_offset);
                        pixel_value[2] = (uint16_t)bintk::UnpackBitsBE(
                            header.bit_depth, current_byte, bit_offset);
                    }

                    if (header.color_type & ColorType_AlphaBit)
                    {
                        pixel_value[3] = (uint16_t)bintk::UnpackBitsBE(
                            header.bit_depth, current_byte, bit_offset);
                    }
                    else
                        pixel_value[3] = (uint16_t)0xffff;
                }
                else
                {
                    uint8_t color_index = (uint8_t)bintk::UnpackBitsBE(
                        header.bit_depth, current_byte, bit_offset);
                    numtk::vec3<uint8_t> const& source = palette[color_index];
                    pixel_value[0] = (uint16_t)source[0];
                    pixel_value[1] = (uint16_t)source[1];
                    pixel_value[2] = (uint16_t)source[2];
                    pixel_value[3] = (uint32_t)color_index >= palette_alpha.size()
                        ? 255
                        : (uint16_t)palette_alpha[color_index];
                }

                if (alpha_color_set)
                {
                    if (alpha_color.x != pixel_value[0]
                        || alpha_color.y != pixel_value[1]
                        || alpha_color.z != pixel_value[2])
                        pixel_value[3] = (uint16_t)0xffff;
                    else
                        pixel_value[3] = (uint16_t)0;
                }

                buffer[scanline_index * header.width + pixel_index] = pixel_value;
            }

            block_stream += current_byte - scanline;
            if (bit_offset)
                block_stream++;
            bit_offset = 0;

            previous_scanline = scanline;
        }
    }


    if (_png_file)
    {
        _png_file->header = header;
        _png_file->pixel_data = std::move(buffer);
    }
}

} // namespace pngtk
