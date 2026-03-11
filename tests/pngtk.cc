#include <fstream>
#include <iostream>
#include <vector>

#define ZIPTK_IMPLEMENTATION
#include "pngtk.hh"

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
        return 1;

    std::string file_path{ argv[1] }

    std::vector<uint8_t> memory = LoadFile(file_path.c_str());
    pngtk::PNGFile png_file = {};
    pngtk::LoadPNG(memory.data(), memory.size(), &png_file);

    float weight = 0.f;
    for (numtk::vec4<uint16_t> const& pixel : png_file.pixel_data)
        if (pixel[3] != 0)
            weight += 1.f;

    weight = 1.f / weight;

    numtk::vec4<float> accum = {};
    for (numtk::vec4<uint16_t> const& pixel : png_file.pixel_data)
        accum += pixel.cast<float>() * weight;

    std::string blockid = file_path;
    {
        std::string::size_type begin = blockid.rfind('\\');
        std::string::size_type end = blockid.rfind('.');
        blockid = blockid.substr(begin+1, end-begin-1);
    }

    std::cout << blockid << std::endl;
    std::cout << "[ " << (uint16_t)std::round(accum[0])
              << ", " << (uint16_t)std::round(accum[1])
              << ", " << (uint16_t)std::round(accum[2])
              << " ]" << std::endl;

    return 0;
}
