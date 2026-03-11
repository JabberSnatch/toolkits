#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

namespace ziptk
{

struct GZipHeader
{
    uint16_t magic;
    uint8_t compression;
    uint8_t header_flags;
    uint32_t timestamp;
    uint8_t compression_flags;
    uint8_t os_id;
};

GZipHeader ExtractGZip(uint8_t const*& stream);

struct ZLibHeader
{
    uint8_t CM;
    uint8_t CINFO;
    uint8_t FCHECK;
    uint8_t FDICT;
    uint8_t FLEVEL;
    uint32_t DICTID;
};
ZLibHeader ExtractZLib(uint8_t const*& stream);

std::vector<uint8_t> Inflate(uint8_t const* stream);

} // namespace ziptk

#ifdef ZIPTK_IMPLEMENTATION

#include <cstring>
#include "bintk.hh"

namespace ziptk
{

struct HuffmannTable
{
    static constexpr uint32_t kInvalidValue = ~0u;
    static constexpr uint32_t kSecondaryFlag = 0x10000000u;
    struct Entry
    {
        uint32_t value = kInvalidValue;
        uint8_t size = 0;
    };

    using ValueContainer = std::array<Entry, 256>;
    ValueContainer primary_storage{};
    std::vector<ValueContainer> secondary_storage{};

    Entry const& Lookup(uint16_t key)
    {
        Entry const& primary = primary_storage[(key&0xff00) >> 8];
        if ((primary.value & kSecondaryFlag) == 0)
            return primary;
        else
            return secondary_storage[primary.value & ~kSecondaryFlag][key&0xff];
    }

    void InsertValue(uint16_t key, uint8_t size, uint32_t value)
    {
        if (size > 8)
        {
            uint32_t primary_key = key >> (size-8);
            Entry& primary = primary_storage[primary_key];
            if (primary.value == kInvalidValue)
            {
                uint32_t secondary_index = (uint32_t)secondary_storage.size();
                secondary_storage.emplace_back();
                ValueContainer& values = secondary_storage.back();
                primary.value = kSecondaryFlag | secondary_index;
            }

            uint32_t secondary_key = (key & ((1<<(size-8))-1)) << (16-size);
            for (uint32_t index = 0; index < (1 << (16 - size)); ++index)
                secondary_storage[primary.value & ~kSecondaryFlag][secondary_key++] =
                    Entry{ value, size };
        }
        else
        {
            uint32_t shifted_key = key << (8 - size);
            for (uint32_t index = 0; index < (1 << (8 - size)); ++index)
                primary_storage[shifted_key++] = Entry{ value, size };
        }
    }
};

HuffmannTable GenerateHuffmannTable(std::vector<uint32_t> const& sizes)
{
    HuffmannTable table{};

    // From DEFLATE specifications
    std::vector<uint32_t> counts{};
    for (uint32_t size : sizes)
    {
        if (size == 0)
            continue;
        if (counts.size() <= size)
            counts.resize(size+1);
        counts[size]++;
    }

    uint64_t code = 0;
    std::vector<uint64_t> next_code(counts.size());
    for (uint32_t bits = 1; bits < counts.size(); ++bits)
    {
        code = (code + counts[bits-1]) << 1;
        next_code[bits] = code;
    }

    std::vector<uint64_t> codes{};
    codes.reserve(sizes.size());
    for (uint32_t index = 0; index < sizes.size(); ++index)
    {
        uint32_t size = sizes[index];
        if (size == 0)
            codes.push_back(~0ull);
        else
        {
            codes.push_back(next_code[size]);
            table.InsertValue((uint16_t)next_code[size], size, index);
            ++next_code[size];
        }
    }

    return table;
}


enum BlockType
{
    NoCompression = 0,
    FixedCodes = 1,
    DynamicCodes = 2,
    Error = 3
};

static const std::array<uint32_t, 29> kLengthBase = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13,
    15, 17, 19, 23, 27, 31, 35, 43, 51, 59,
    67, 83, 99, 115, 131, 163, 195, 227, 258
};
static const std::array<uint32_t, 29> kLengthExtraBits {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
    4, 4, 4, 4, 5, 5, 5, 5, 0
};

static const std::array<uint32_t, 30> kDistanceBase = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25,
    33, 49, 65, 97, 129, 193, 257, 385, 513, 769,
    1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
};
static const std::array<uint32_t, 30> kDistanceExtraBits = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3,
    4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
    9, 9, 10, 10, 11, 11, 12, 12, 13, 13
};

GZipHeader ExtractGZip(std::uint8_t const*& stream)
{
    GZipHeader header = {};
    header.magic = (uint16_t)bintk::UnpackBytes(2, stream);
    header.compression = (uint8_t)bintk::UnpackBytes(1, stream);
    header.header_flags = (uint8_t)bintk::UnpackBytes(1, stream);
    header.timestamp = bintk::UnpackBytes(4, stream);
    header.compression_flags = (uint8_t)bintk::UnpackBytes(1, stream);
    header.os_id = (uint8_t)bintk::UnpackBytes(1, stream);
    return header;
}

ZLibHeader ExtractZLib(std::uint8_t const*& stream)
{
    std::uint32_t offset = 0;
    ZLibHeader header = {};
    header.CM = (uint8_t)bintk::UnpackBits(4, stream, offset);
    header.CINFO = (uint8_t)bintk::UnpackBits(4, stream, offset);
    header.FCHECK = (uint8_t)bintk::UnpackBits(5, stream, offset);
    header.FDICT = (uint8_t)bintk::UnpackBits(1, stream, offset);
    header.FLEVEL = (uint8_t)bintk::UnpackBits(2, stream, offset);
    header.DICTID = 0u;
    if (header.FDICT)
        header.DICTID = bintk::UnpackBytes(4, stream);
    return header;
}

std::vector<uint8_t> Inflate(std::uint8_t const* stream)
{
    std::vector<uint8_t> output_stream = {};
    std::uint8_t const* base = stream;

    std::uint32_t offset = 0;

    for (;;)
    {
        uint32_t block_header = bintk::UnpackBits(3, stream, offset);
        bool BFINAL = !!(block_header&1);
        BlockType BTYPE = (BlockType)(block_header >> 1);

        if (BTYPE == BlockType::Error)
        {
            std::cerr << "Invalid block type" << std::endl;
            break;
        }

        if (BTYPE == BlockType::NoCompression)
        {
            ++stream;
            std::uint32_t LEN = bintk::UnpackBytes(2, stream);
            std::uint32_t NLEN = bintk::UnpackBytes(2, stream);

            std::uint32_t output_offset = output_stream.size();
            output_stream.resize(output_stream.size() + LEN);
            std::memcpy(output_stream.data() + output_offset,
                        stream,
                        (std::size_t)LEN);

            stream += LEN;
        }
        else
        {
            HuffmannTable litlen_code = {};
            HuffmannTable dist_code = {};

            if (BTYPE == BlockType::DynamicCodes)
            {
                uint32_t HLIT = bintk::UnpackBits(5, stream, offset);
                uint32_t HDIST = bintk::UnpackBits(5, stream, offset);
                uint32_t HCLEN = bintk::UnpackBits(4, stream, offset);

                static const std::vector<uint32_t> kCodeLengthTable = {
                    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
                };

                std::vector<uint32_t> code_lengths{};
                code_lengths.resize(19);
                for (uint32_t code_index = 0; code_index < HCLEN+4; ++code_index)
                    code_lengths[kCodeLengthTable[code_index]] =
                        bintk::UnpackBits(3, stream, offset);

                HuffmannTable secondary_code = GenerateHuffmannTable(code_lengths);

                std::vector<uint32_t> litlen_code_lengths{};
                litlen_code_lengths.reserve(HLIT+HDIST+258);
                uint32_t entry_index = 0;
                while (entry_index < HLIT+257)
                {
                    uint16_t stream_bits = bintk::ReverseBits(
                        (uint16_t)bintk::PeekBits(16, stream, offset)
                    );
                    HuffmannTable::Entry const& entry = secondary_code.Lookup(stream_bits);
                    bintk::SkipBits(entry.size, stream, offset);

                    if (entry.value == 16)
                    {
                        uint32_t value = litlen_code_lengths.back();
                        uint32_t repeat_count = bintk::UnpackBits(2, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            litlen_code_lengths.push_back(value);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 17)
                    {
                        uint32_t repeat_count = bintk::UnpackBits(3, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            litlen_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 18)
                    {
                        uint32_t repeat_count = bintk::UnpackBits(7, stream, offset) + 11;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            litlen_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else
                    {
                        litlen_code_lengths.push_back(entry.value);
                        ++entry_index;
                    }
                }

                std::vector<uint32_t> dist_code_lengths{};
                dist_code_lengths.reserve(HDIST+1);
                if (litlen_code_lengths.size() > HLIT+257)
                {
                    std::copy(litlen_code_lengths.begin()+HLIT+257, litlen_code_lengths.end(),
                              std::back_inserter(dist_code_lengths));
                    litlen_code_lengths.resize(HLIT+257);
                }

                litlen_code = GenerateHuffmannTable(litlen_code_lengths);

                while (entry_index < HDIST+HLIT+258)
                {
                    uint16_t stream_bits = bintk::ReverseBits(
                        (uint16_t)bintk::PeekBits(16, stream, offset)
                    );
                    HuffmannTable::Entry const& entry = secondary_code.Lookup(stream_bits);
                    bintk::SkipBits(entry.size, stream, offset);

                    if (entry.value == 16)
                    {
                        uint32_t value = dist_code_lengths.back();
                        uint32_t repeat_count = bintk::UnpackBits(2, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            dist_code_lengths.push_back(value);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 17)
                    {
                        uint32_t repeat_count = bintk::UnpackBits(3, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            dist_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 18)
                    {
                        uint32_t repeat_count = bintk::UnpackBits(7, stream, offset) + 11;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            dist_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else
                    {
                        dist_code_lengths.push_back(entry.value);
                        ++entry_index;
                    }
                }

                dist_code = GenerateHuffmannTable(dist_code_lengths);
            }

            else if (BTYPE == BlockType::FixedCodes)
            {
                std::vector<uint32_t> litlen_lengths = {};
                litlen_lengths.reserve(288);
                for (uint32_t index = 0; index < 144; ++index)
                    litlen_lengths.push_back(8);
                for (uint32_t index = 144; index < 256; ++index)
                    litlen_lengths.push_back(9);
                for (uint32_t index = 256; index < 280; ++index)
                    litlen_lengths.push_back(7);
                for (uint32_t index = 280; index < 288; ++index)
                    litlen_lengths.push_back(8);
                litlen_code = GenerateHuffmannTable(litlen_lengths);

                std::vector<uint32_t> dist_lengths = {};
                for (uint32_t index = 0; index < 32; ++index)
                    dist_lengths.push_back(5);
                dist_code = GenerateHuffmannTable(dist_lengths);
            }

            for (;;)
            {
                uint16_t stream_bits = bintk::ReverseBits(
                    (uint16_t)bintk::PeekBits(16, stream, offset)
                );
                HuffmannTable::Entry const& litlen_entry = litlen_code.Lookup(stream_bits);
                bintk::SkipBits(litlen_entry.size, stream, offset);

                if (litlen_entry.value == 256)
                    break;

                if (litlen_entry.value < 256)
                    output_stream.push_back(litlen_entry.value);
                else
                {
                    uint32_t length = 0;
                    {
                        uint32_t const base = kLengthBase[litlen_entry.value - 257];
                        uint32_t const extra_bits = kLengthExtraBits[litlen_entry.value - 257];
                        length = base + bintk::UnpackBits(extra_bits, stream, offset);
                    }

                    stream_bits = bintk::ReverseBits(
                        (uint16_t)bintk::PeekBits(16, stream, offset)
                    );
                    HuffmannTable::Entry const& dist_entry = dist_code.Lookup(stream_bits);
                    bintk::SkipBits(dist_entry.size, stream, offset);

                    uint32_t distance = 0;
                    {
                        uint32_t const base = kDistanceBase[dist_entry.value];
                        uint32_t const extra_bits = kDistanceExtraBits[dist_entry.value];
                        distance = base + bintk::UnpackBits(extra_bits, stream, offset);
                    }

                    uint32_t begin = output_stream.size()-distance;
                    std::size_t old_size = output_stream.size();
                    output_stream.resize(output_stream.size() + length);
                    if ((begin+length) < old_size)
                        std::memcpy(&output_stream[old_size], &output_stream[begin], length);
                    else
                        for (uint32_t byte_index = 0; byte_index < length; ++byte_index)
                            output_stream[old_size+byte_index] =
                                output_stream[begin+byte_index];

                }
            }
        }

        if (BFINAL)
            break;

        // TEMPORARY STOPPER
        break;
    }

    if (offset != 0)
        ++stream;
    uint32_t stream_checksum = bintk::UnpackBytesBE(4, stream);
    uint32_t checksum_s1 = 1;
    uint32_t checksum_s2 = 0;
    for (uint8_t byte : output_stream)
    {
        checksum_s1 = (checksum_s1 + byte) % 65521u;
        checksum_s2 = (checksum_s2 + checksum_s1) % 65521u;
    }
    uint32_t computed_checksum = checksum_s2*65536 + checksum_s1;

    return output_stream;
}

#endif // #defined ZIPTK_IMPLEMENTATION

} // namespace ziptk
