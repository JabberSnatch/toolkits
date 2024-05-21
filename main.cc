#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>
#include <bitset>

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

std::uint16_t ReverseBits(std::uint16_t bits)
{
    bits = ((bits & 0x00ff) << 8) | ((bits & 0xff00) >> 8);
    bits = ((bits & 0x0f0f) << 4) | ((bits & 0xf0f0) >> 4);
    bits = ((bits & 0x3333) << 2) | ((bits & 0xcccc) >> 2);
    bits = ((bits & 0x5555) << 1) | ((bits & 0xaaaa) >> 1);
    return bits;
}

std::uint32_t UnpackBytes(std::uint32_t count, std::uint8_t const*& stream)
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

void AdvanceBits(std::uint32_t count, std::uint8_t const*& stream, std::uint32_t& offset)
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

std::uint32_t PeekBits(std::uint32_t count, std::uint8_t const* stream, std::uint32_t offset)
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

std::uint32_t UnpackBits(std::uint32_t count, std::uint8_t const*& stream, std::uint32_t& offset)
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

struct GZipHeader
{
    uint16_t magic;
    uint8_t compression;
    uint8_t header_flags;
    uint32_t timestamp;
    uint8_t compression_flags;
    uint8_t os_id;
};

void inflate(std::uint8_t const* stream)
{
    std::vector<uint8_t> output_stream = {};
    std::uint8_t const* base = stream;

    GZipHeader header = {};
    header.magic = (uint16_t)UnpackBytes(2, stream);
    header.compression = (uint8_t)UnpackBytes(1, stream);
    header.header_flags = (uint8_t)UnpackBytes(1, stream);
    header.timestamp = UnpackBytes(4, stream);
    header.compression_flags = (uint8_t)UnpackBytes(1, stream);
    header.os_id = (uint8_t)UnpackBytes(1, stream);
    std::cout << std::hex << header.magic << std::endl
              << std::hex << (uint32_t)header.compression << std::endl
              << std::hex << (uint32_t)header.header_flags << std::endl
              << std::hex << header.timestamp << std::endl
              << std::hex << (uint32_t)header.compression_flags << std::endl
              << std::hex << (uint32_t)header.os_id << std::endl;

    std::uint32_t offset = 0;

    for (;;)
    {
        std::cout << std::hex << (uint32_t)*stream << std::endl;
        {
            uint8_t const* temp = stream + 255;
            for (uint32_t i = 0; i < 256; ++i)
                std::cout << std::bitset<8>(*(temp - i)) << " ";
            std::cout << std::endl;
        }

        uint32_t block_header = UnpackBits(3, stream, offset);
        bool BFINAL = !!(block_header&1);
        BlockType BTYPE = (BlockType)(block_header >> 1);

        std::cout << std::hex << block_header << std::endl;
        std::cout << BTYPE << std::endl;

        std::cout << std::bitset<3>(block_header) << " ";

        if (BTYPE == BlockType::Error)
        {
            std::cerr << "Invalid block type" << std::endl;
            break;
        }

        if (BTYPE == BlockType::NoCompression)
        {
            ++stream;
            std::uint32_t LEN = UnpackBytes(2, stream);
            std::uint32_t NLEN = UnpackBytes(2, stream);

            stream += LEN;
        }
        else
        {
            if (BTYPE == BlockType::DynamicCodes)
            {
                uint32_t HLIT = UnpackBits(5, stream, offset);
                uint32_t HDIST = UnpackBits(5, stream, offset);
                uint32_t HCLEN = UnpackBits(4, stream, offset);

                //std::cout << std::bitset<5>(HLIT) << " ";
                //std::cout << std::bitset<5>(HDIST) << " ";
                //std::cout << std::bitset<4>(HCLEN) << " " ;

                static const std::vector<uint32_t> kCodeLengthTable = {
                    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
                };

                std::vector<uint32_t> code_lengths{};
                code_lengths.resize(19);
                for (uint32_t code_index = 0; code_index < HCLEN+4; ++code_index)
                {
                    code_lengths[kCodeLengthTable[code_index]] = UnpackBits(3, stream, offset);
                    //std::cout << std::bitset<3>(code_lengths[kCodeLengthTable[code_index]]) << " ";
                }

                //std::cout << std::endl;
                HuffmannTable secondary_code = GenerateHuffmannTable(code_lengths);

                std::vector<uint32_t> litlen_code_lengths{};
                litlen_code_lengths.reserve(HLIT+HDIST+258);
                uint32_t entry_index = 0;
                while (entry_index < HLIT+257)
                {
                    uint16_t stream_bits = ReverseBits((uint16_t)PeekBits(16, stream, offset));
                    HuffmannTable::Entry const& entry = secondary_code.Lookup(stream_bits);
                    AdvanceBits(entry.size, stream, offset);

                    /*
                    #define PRINT_BITS(BITCOUNT)                           \
                        case BITCOUNT:                                     \
                            std::cout << std::bitset<BITCOUNT>(stream_bits >> (16 - entry.size)) << " "; \
                            break;

                    switch (entry.size)
                    {
                        PRINT_BITS(1);
                        PRINT_BITS(2);
                        PRINT_BITS(3);
                        PRINT_BITS(4);
                        PRINT_BITS(5);
                        PRINT_BITS(6);
                        PRINT_BITS(7);
                    default: break;
                    }
                    #undef PRINT_BITS
                    */

                    if (entry.value == 16)
                    {
                        uint32_t value = litlen_code_lengths.back();
                        uint32_t repeat_count = UnpackBits(2, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            litlen_code_lengths.push_back(value);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 17)
                    {
                        uint32_t repeat_count = UnpackBits(3, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            litlen_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 18)
                    {
                        uint32_t repeat_count = UnpackBits(7, stream, offset) + 11;
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

                //std::cout << std::endl;

                /*
                for (uint32_t length : code_lengths)
                    std::cout << length << " ";
                std::cout << std::endl;
                */

                std::vector<uint32_t> dist_code_lengths{};
                dist_code_lengths.reserve(HDIST+1);
                if (litlen_code_lengths.size() > HLIT+257)
                {
                    std::copy(litlen_code_lengths.begin()+HLIT+257, litlen_code_lengths.end(),
                              std::back_inserter(dist_code_lengths));
                    litlen_code_lengths.resize(HLIT+257);
                }

                HuffmannTable litlen_code = GenerateHuffmannTable(litlen_code_lengths);

                while (entry_index < HDIST+HLIT+258)
                {
                    uint16_t stream_bits = ReverseBits((uint16_t)PeekBits(16, stream, offset));
                    HuffmannTable::Entry const& entry = secondary_code.Lookup(stream_bits);
                    AdvanceBits(entry.size, stream, offset);

                    if (entry.value == 16)
                    {
                        uint32_t value = dist_code_lengths.back();
                        uint32_t repeat_count = UnpackBits(2, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            dist_code_lengths.push_back(value);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 17)
                    {
                        uint32_t repeat_count = UnpackBits(3, stream, offset) + 3;
                        for (uint32_t repeat = 0; repeat < repeat_count; ++repeat)
                            dist_code_lengths.push_back(0);
                        entry_index += repeat_count;
                    }

                    else if (entry.value == 18)
                    {
                        uint32_t repeat_count = UnpackBits(7, stream, offset) + 11;
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

                HuffmannTable dist_code = GenerateHuffmannTable(dist_code_lengths);
                std::cout << std::endl;

                for (;;)
                {
                    uint16_t stream_bits = ReverseBits((uint16_t)PeekBits(16, stream, offset));
                    HuffmannTable::Entry const& litlen_entry = litlen_code.Lookup(stream_bits);
                    AdvanceBits(litlen_entry.size, stream, offset);

                    if (litlen_entry.value == 256)
                        break;

                    if (litlen_entry.value < 256)
                        output_stream.push_back(litlen_entry.value);
                    else
                    {
                        uint32_t length = 0;
                        {
                            uint32_t const base = kLengthBase[litlen_entry.value - 256];
                            uint32_t const extra_bits = kLengthExtraBits[litlen_entry.value - 256];
                            length = base + UnpackBits(extra_bits, stream, offset);
                        }

                        stream_bits = ReverseBits((uint16_t)PeekBits(16, stream, offset));
                        HuffmannTable::Entry const& dist_entry = dist_code.Lookup(stream_bits);
                        AdvanceBits(dist_entry.size, stream, offset);

                        uint32_t distance = 0;
                        {
                            uint32_t const base = kDistanceBase[dist_entry.value];
                            uint32_t const extra_bits = kDistanceExtraBits[dist_entry.value];
                            distance = base + UnpackBits(extra_bits, stream, offset);
                        }

                        for (uint32_t byte_index = 0; byte_index < length; ++byte_index)
                            output_stream.push_back(
                                output_stream[output_stream.size()-1-distance+byte_index]);

                        int i = 0;
                    }
                }
            }
        }

        if (BFINAL)
            break;

        // TEMPORARY STOPPER
        break;
    }

    for (uint8_t c : output_stream)
    {
        std::cout << c;
    }
    std::cout << std::endl;

    return;
}

int main(int argc, char const** argv)
{
#if 0
    {
        std::vector<uint64_t> fixed_codes = [](){
            std::vector<uint32_t> sizes{};
            sizes.reserve(288);
            for (int index = 0; index < 144; ++index)
                sizes.push_back(8);
            for (int index = 144; index < 256; ++index)
                sizes.push_back(9);
            for (int index = 256; index < 280; ++index)
                sizes.push_back(7);
            for (int index = 280; index < 288; ++index)
                sizes.push_back(8);
            return GenerateHuffmannTable(sizes);
        }();
    }
#endif

    {
        HuffmannTable test_code = GenerateHuffmannTable({ 3, 3, 3, 3, 3, 2, 4, 4 });
        std::cout << "expected 7 : " << test_code.Lookup(0xe000).value << std::endl;
        std::cout << "expected 7 : " << test_code.Lookup(0xef0f).value << std::endl;
        std::cout << "expected 8 : " << test_code.Lookup(0xf123).value << std::endl;
        std::cout << "expected 6 : " << test_code.Lookup(0x3123).value << std::endl;
        std::cout << std::endl;
    }

#if 0
    {
        HuffmannTable fixed_codes = GenerateHuffmannTable({ 13, 14, 12, 2, 2, 11, 13 });
        std::cout << std::endl;
    }
#endif

#if 0
    uint64_t unpacktest = 0xdeadbeefdeadbeef;
    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 0;
        uint32_t unpack = UnpackBits(15, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 0;
        uint32_t unpack = UnpackBits(7, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 1;
        uint32_t unpack = UnpackBits(5, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 4;
        uint32_t unpack = UnpackBits(4, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 4;
        uint32_t unpack = UnpackBits(1, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*)&unpacktest;
        uint32_t offset = 4;
        uint32_t unpack = UnpackBits(32, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }

    {
        uint8_t const* stream = (uint8_t const*) &unpacktest;
        uint32_t offset = 0;
        uint32_t unpack = UnpackBits(32, stream, offset);
        std::cout << std::hex << unpack << ":" << std::dec << offset << std::endl;
    }
#endif

    /*
      1 0000 0000 -> EOF
      1 0000 0001 -> 3
      1 0000 0010 -> 4
      ...
      1 0000 1000 -> 10
      1 0000 1001 0 -> 11
      1 0000 1001 1 -> 12
      1 0000 1010 0 -> 13
      ...
      1 0000 1100 1 -> 18
      ...
      1 0111 0001 -> 258
    */

    if (argc == 2)
    {
        std::FILE* file = std::fopen(argv[1], "rb");
        std::fseek(file, 0, SEEK_END);
        uint64_t size = std::ftell(file);
        std::fseek(file, 0, SEEK_SET);
        std::string contents(size, '\0');
        std::fread(contents.data(), 1, size, file);
        std::fclose(file);

        inflate((std::uint8_t const*)contents.data());
        //std::cout << contents << std::endl;
    }

    return 0;
}
