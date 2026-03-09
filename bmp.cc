#include <fstream>
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

void WriteFile(char const* _path, uint8_t const* _base, uint32_t _size)
{
    std::ofstream dest_file(_path, std::ios_base::binary);
    dest_file.write((char const*)_base, (std::streamsize)_size);
}

int main(int argc, char const** argv)
{
    if (argc == 1)
    {
        std::cout << "Missing path to BMP file as first argument." << std::endl;
        return 1;
    }

    {
        std::vector<uint8_t> memory = LoadFile(argv[1]);

        bmptk::BitmapFile bmpfile = {};
        bmptk::Result res = bmptk::LoadBMP(memory.data(), &bmpfile);
        if (res != bmptk::Result::Success)
        {
            std::cout << "Failed to load BMP." << std::endl;
            return 1;
        }

        std::cout << std::hex << (uintptr_t)bmpfile.memory << std::endl
                  << (uintptr_t)bmpfile.pixel_data << std::endl
                  << std::dec << (ptrdiff_t)(bmpfile.pixel_data - bmpfile.memory) << std::endl;

        std::cout << std::dec << bmpfile.width << " " << bmpfile.height << std::endl;
        std::cout << "bpp " << bmpfile.bpp << std::endl;
        std::cout << "bi " << bmpfile.bi << std::endl;
        std::cout << "size " << bmpfile.size << std::endl;

        std::cout << abs(bmpfile.width * bmpfile.height * ((int32_t)bmpfile.bpp/8)) << std::endl;

#if 0
        for (uint32_t y = 0u; y < abs(bmpfile.height); ++y)
        {
            for (uint32_t x = 0u; x < abs(bmpfile.width); ++x)
            {
                bmptk::PixelValue pixel = bmptk::GetPixel(&bmpfile, x, y);
                std::cout << (uint32_t)pixel.red << " "
                          << (uint32_t)pixel.green << " "
                          << (uint32_t)pixel.blue << std::endl;
            }
        }
#endif
    }

    {
        bmptk::BitmapV1Header bmpheader = {};
        bmpheader.width = 2;
        bmpheader.height = -2;

        std::vector<uint8_t> memory(bmptk::AllocSize(&bmpheader));
        std::vector<bmptk::PixelValue> pixels(4);
        pixels[0] = bmptk::PixelValue{ 255, 0, 0 };
        pixels[1] = bmptk::PixelValue{ 0, 255, 0 };
        pixels[2] = bmptk::PixelValue{ 0, 0, 255 };
        pixels[3] = bmptk::PixelValue{ 255, 255, 255 };

        std::cout << memory.size() << std::endl;

        WriteBMP(&bmpheader, pixels.data(), memory.data());
        WriteFile("testfile.bmp", memory.data(), memory.size());
    }

    return 0;
}
