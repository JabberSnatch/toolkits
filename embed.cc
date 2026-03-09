#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#define BMPTK_IMPLEMENTATION
#include "bmptk.h"

std::vector<uint8_t> LoadFile(char const* _path)
{
    std::ifstream source_file(_path, std::ios_base::binary);

    source_file.seekg(0, std::ios_base::end);
    size_t size = source_file.tellg();
    source_file.seekg(0, std::ios_base::beg);

    std::vector<uint8_t> memory{};
    memory.resize(size);

    source_file.read((char*)memory.data(), size);

    return memory;
}

int main(int argc, char const** argv)
{
    if (argc == 1)
    {
        std::cout << "Missing path to BMP file as first argument." << std::endl;
        return 1;
    }

    std::vector<uint8_t> memory = LoadFile(argv[1]);
    uint32_t cell_count_x = (uint32_t)atoi(argv[2]);
    uint32_t cell_count_y = (uint32_t)atoi(argv[3]);
    uint32_t char_count = (uint32_t)atoi(argv[4]);

    bmptk::BitmapFile bmpfile = {};
    bmptk::Result res = bmptk::LoadBMP(memory.data(), &bmpfile);
    if (res != bmptk::Result::Success)
    {
        std::cout << "Failed to load BMP." << std::endl;
        return 1;
    }

    uint32_t cell_size_x = abs(bmpfile.width) / cell_count_x;
    uint32_t cell_size_y = abs(bmpfile.height) / cell_count_y;
    uint32_t cell_pixel_count = cell_size_x * cell_size_y;
    std::vector<uint64_t> encoded_data(char_count);

    std::cout << "static constexpr uint64_t kFontData[" << char_count << "] = {\n";

    for (uint32_t char_index = 0u; char_index < char_count; ++char_index)
    {
        uint64_t encoded_char = 0ull;
        uint32_t cell_index_x = char_index % cell_count_x;
        uint32_t cell_index_y = char_index / cell_count_x;

        for (uint32_t pixel_index = 0u; pixel_index < cell_pixel_count; ++pixel_index)
        {
            uint32_t pixel_index_x = cell_index_x * cell_size_x + (pixel_index % cell_size_x);
            uint32_t pixel_index_y = (bmpfile.height-1) - (cell_index_y * cell_size_y + (pixel_index / cell_size_x));

            bmptk::PixelValue pixel = bmptk::GetPixel(&bmpfile, pixel_index_x, pixel_index_y);
            encoded_char |= (pixel.red != 0) ? (1ull << (cell_pixel_count-1-pixel_index)) : 0ull;
        }

        if (char_index % 4 == 0)
            std::cout << "    ";

        std::cout << "0x" << std::setfill('0') << std::setw(16) << std::hex << encoded_char;
        std::cout << "ull, ";

        if ((char_index+1) % 4 == 0)
            std::cout << std::endl;
    }

    std::cout << "\n};\n";

    return 0;
}
