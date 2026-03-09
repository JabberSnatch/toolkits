#include <fstream>
#include <iostream>
#include <vector>

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

    std::vector<uint8_t> memory = LoadFile(argv[1]);
    pngtk::LoadPNG(memory.data(), memory.size());

    return 0;
}
