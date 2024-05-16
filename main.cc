#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

std::vector<uint64_t> GenerateHuffmannTable(std::vector<uint32_t> const &sizes)
{
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
    for (uint32_t size : sizes)
    {
        if (size == 0)
            codes.push_back(~0ull);
        else
        {
            codes.push_back(next_code[size]);
            ++next_code[size];
        }
    }

    return codes;
}

enum BlockType
{
    NoCompression = 0,
    FixedCodes = 1,
    DynamicCodes = 2,
    Error = 3
};

std::uint32_t UnpackBytes(std::uint32_t count, std::uint8_t const*& stream)
{
    std::uint32_t output = 0;
    if (count > 0)
        output |= *stream++;
    if (count > 1)
        output = (output << 8) | *stream++;
    if (count > 2)
        output = (output << 8) | *stream++;
    if (count > 3)
        output = (output << 8) | *stream++;
    return output;
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
        v |= ((*stream >> offset) & ((1 << head)-1)) << (tail + body);
        offset += head;
        if (offset > 7)
        {
            offset = offset % 8;
            ++stream;
        }
    }

    if (body != 0)
        v |= UnpackBytes(body/8, stream) << tail;

    if (tail != 0)
    {
        v |= (*stream & ((1 << tail)-1));
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

        uint32_t block_header = UnpackBits(3, stream, offset);
        bool BFINAL = !!(block_header&1);
        BlockType BTYPE = (BlockType)(block_header >> 1);

        std::cout << std::hex << block_header << std::endl;
        std::cout << BTYPE << std::endl;

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
                std::cout << std::dec
                          << HLIT << std::endl
                          << HDIST << std::endl
                          << HCLEN << std::endl;

                static const uint32_t kCodeLengthTable[] = {
                    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
                };

                std::vector<uint32_t> code_lengths{};
                code_lengths.reserve(HCLEN+4);
                for (uint32_t code_index = 0; code_index < HCLEN+4; ++code_index)
                    code_lengths.push_back(UnpackBits(3, stream, offset));

                std::vector<uint64_t> secondary_code = GenerateHuffmannTable(code_lengths);
                std::cout << std::endl;
            }
        }

        if (BFINAL)
            break;

        // TEMPORARY STOPPER
        break;
    }

    return;
}

int main(int argc, char const** argv)
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
