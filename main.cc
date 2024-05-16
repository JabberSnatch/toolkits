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
    GZipHeader const* header = (GZipHeader const*)stream;
    std::cout << std::hex << header->magic << std::endl
              << std::hex << header->compression << std::endl
              << std::hex << header->header_flags << std::endl
              << std::hex << header->timestamp << std::endl
              << std::hex << header->compression_flags << std::endl
              << std::hex << header->os_id << std::endl;
    std::uint8_t const* payload = (std::uint8_t const*)(header+1);
    return;
}

int main(int argc, char const** argv)
{
    auto table = GenerateHuffmannTable({ 3, 3, 3, 3, 3, 2, 4, 4 });

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
