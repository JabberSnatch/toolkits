#include <fstream>
#include <iostream>
#include <vector>
#include <filesystem>

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

void PrintColor(std::string const& blockid, numtk::vec4<float> const& accum)
{
    std::cout << "    " "{ " "\"" << blockid << "\"" ", "
              << "numtk::vec3<int8_t>{ "
              << "'\\x" << std::hex << (uint16_t)std::round(accum[0]) << "', "
              << "'\\x" << std::hex << (uint16_t)std::round(accum[1]) << "', "
              << "'\\x" << std::hex << (uint16_t)std::round(accum[2]) << "' }.cast<uint8_t>() },\n";
    std::cout << std::dec;
}

void PrintFile(std::string const& file_path)
{
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
        if (pixel[3] != 0)
            accum += pixel.cast<float>() * weight;

    if (!(png_file.header.color_type & pngtk::ColorType_TruecolorBit))
    {
        accum[1] = accum[0];
        accum[2] = accum[0];
    }

    std::string blockid = file_path;
    {
        std::string::size_type begin = blockid.rfind('\\');
        std::string::size_type end = blockid.rfind('.');
        blockid = blockid.substr(begin+1, end-begin-1);
    }

    if (blockid == "grass_block_top"
        || blockid == "oak_leaves"
        || blockid == "jungle_leaves"
        || blockid == "acacia_leaves"
        || blockid == "dark_oak_leaves"
        || blockid == "vine")
        accum = accum * numtk::vec4f{
            (float)0x91 / (float)0xff,
            (float)0xbd / (float)0xff,
            (float)0x59 / (float)0xff,
            1.f
        };

    if (blockid == "birch_leaves")
        accum = accum * numtk::vec4f{
            (float)0x80 / (float)0xff,
            (float)0xa7 / (float)0xff,
            (float)0x55 / (float)0xff
        };

    if (blockid == "spruce_leaves")
        accum = accum * numtk::vec4f{
            (float)0x6f / (float)0xff,
            (float)0x99 / (float)0xff,
            (float)0x61 / (float)0xff
        };

    PrintColor(blockid, accum);
    if (blockid == "grass_block_top")
        PrintColor("grass_block", accum);
}

int main(int argc, char const** argv)
{
    if (argc <= 2)
        return 1;

    std::string mode{ argv[1] };

    if (mode == "file")
        PrintFile(std::string{ argv[2] });

    if (mode == "directory")
    {
        std::filesystem::path root_folder{ argv[2] };

        std::cout <<
            "static std::unordered_map<std::string, numtk::vec3<uint8_t>> const kBlockColors = {\n";

        for (auto&& direntry : std::filesystem::directory_iterator{ root_folder })
        {
            if (direntry.path().extension() != ".png")
                continue;
            std::string file_path = direntry.path().string();
            PrintFile(file_path);
        }

        std::cout << "};\n";
    }

    return 0;
}
