#define ZIPTK_IMPLEMENTATION
#include "ziptk.hh"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>
#include <bitset>
#include <functional>

std::uint64_t UnpackBytesBE(std::uint32_t count, std::uint8_t const*& stream)
{
    std::uint64_t output = 0;
    if (count > 0)
        output |= *stream++;
    if (count > 1)
        output = (output << 8) | *stream++;
    if (count > 2)
        output = (output << 8) | *stream++;
    if (count > 3)
        output = (output << 8) | *stream++;
    if (count > 4)
        output = (output << 8) | *stream++;
    if (count > 5)
        output = (output << 8) | *stream++;
    if (count > 6)
        output = (output << 8) | *stream++;
    if (count > 7)
        output = (output << 8) | *stream++;
    return output;
}

enum NBTTag
{
    TAG_End = 0,
    TAG_Byte = 1,
    TAG_Short = 2,
    TAG_Int = 3,
    TAG_Long = 4,
    TAG_Float = 5,
    TAG_Double = 6,
    TAG_Byte_Array = 7,
    TAG_String = 8,
    TAG_List = 9,
    TAG_Compound = 10,
    TAG_Int_Array = 11,
    TAG_Long_Array = 12,
};

uint32_t NBTTypeSize(NBTTag type);
void* NBTDataAlloc(NBTTag type, uint32_t count);
void NBTDataFree(void* alloc);

using NBTDataDeleter = std::function<void(void*)>;
NBTDataDeleter NBTGetDataDeleter(NBTTag type);

struct NBTField
{
    NBTTag type;
    std::string name;
    void* payload;

    NBTField() = default;
    NBTField(NBTField const&) = delete;
    NBTField const& operator=(NBTField const&) = delete;
    NBTField(NBTField&& o): type{ o.type }, name{ o.name }, payload{ o.payload }
    { o.payload = nullptr; }
    NBTField const& operator=(NBTField&& o) {
        this->~NBTField();
        type = o.type;
        name = o.name;
        payload = o.payload;
        o.payload = nullptr;
        return *this;
    }
    ~NBTField()
    {
        if (payload)
        {
            NBTDataDeleter deleter = NBTGetDataDeleter(type);
            if (deleter) deleter(payload);
            NBTDataFree(payload);
        }
    }
};

using NBTCompound = std::vector<NBTField>;
using NBTByteArray = std::vector<int8_t>;
using NBTIntArray = std::vector<int32_t>;
using NBTLongArray = std::vector<int64_t>;
using NBTString = std::string;

struct NBTList
{
    NBTTag type;
    uint32_t size;
    void* data;

    NBTList() = default;
    NBTList(NBTList const&) = delete;
    NBTList const& operator=(NBTList const&) = delete;
    NBTList(NBTList&& o): type{ o.type }, size{ o.size }, data{ o.data }
    { o.data = nullptr; }
    NBTList const& operator=(NBTList&& o) {
        this->~NBTList();
        type = o.type;
        size = o.size;
        data = o.data;
        o.data = nullptr;
        return *this;
    }
    ~NBTList()
    {
        if (data)
        {
            NBTDataDeleter deleter = NBTGetDataDeleter(type);
            if (deleter)
            {
                uint32_t type_size = NBTTypeSize(type);
                uint8_t* element = (uint8_t*)data;
                for (uint32_t index = 0; index < size; ++index, element += type_size)
                    deleter(element);
            }
            NBTDataFree(data);
        }
    }
};

uint32_t NBTTypeSize(NBTTag type)
{
    switch (type)
    {
    case TAG_End: return 0;
    case TAG_Byte: return 1;
    case TAG_Short: return 2;
    case TAG_Int: return 4;
    case TAG_Long: return 8;
    case TAG_Float: return 4;
    case TAG_Double: return 8;
    case TAG_Byte_Array: return sizeof(NBTByteArray);
    case TAG_String: return sizeof(NBTString);
    case TAG_List: return sizeof(NBTList);
    case TAG_Compound: return sizeof(NBTCompound);
    case TAG_Int_Array: return sizeof(NBTIntArray);
    case TAG_Long_Array: return sizeof(NBTLongArray);
    default: return 0;
    }
}

void* NBTDataAlloc(NBTTag type, uint32_t count)
{
    uint64_t alloc_size = NBTTypeSize(type) * count;
    return new uint8_t[alloc_size];
}

NBTDataDeleter NBTGetDataDeleter(NBTTag type)
{
    switch (type)
    {
    case TAG_Byte_Array: return [](void* alloc){ ((NBTByteArray*)alloc)->~NBTByteArray(); };
    case TAG_String: return [](void* alloc){ ((NBTString*)alloc)->~NBTString(); };
    case TAG_List: return [](void* alloc){ ((NBTList*)alloc)->~NBTList(); };
    case TAG_Compound: return [](void* alloc){ ((NBTCompound*)alloc)->~NBTCompound(); };
    case TAG_Int_Array: return [](void* alloc){ ((NBTIntArray*)alloc)->~NBTIntArray(); };
    case TAG_Long_Array: return [](void* alloc){ ((NBTLongArray*)alloc)->~NBTLongArray(); };
    default: return nullptr;
    }
}

void NBTDataFree(void* alloc)
{
    delete [] (uint8_t*)alloc;
}

void UnpackFieldData(NBTTag type, void* data,
                     uint8_t const* stream_base, size_t stream_length,
                     uint8_t const*& stream)
{
    if (stream - stream_base > stream_length)
        return;

    switch (type)
    {
    case NBTTag::TAG_Byte:
        *(int8_t*)data = (int8_t)UnpackBytesBE(1, stream);
        break;

    case NBTTag::TAG_Short:
        *(int16_t*)data = (int16_t)UnpackBytesBE(2, stream);
        break;

    case NBTTag::TAG_Int:
        *(int32_t*)data = (int32_t)UnpackBytesBE(4, stream);
        break;

    case NBTTag::TAG_Long:
        *(int64_t*)data = (int64_t)UnpackBytesBE(8, stream);
        break;

    case NBTTag::TAG_Float:
        *(float*)data = (float)UnpackBytesBE(4, stream);
        break;

    case NBTTag::TAG_Double:
        *(double*)data = (double)UnpackBytesBE(8, stream);
        break;

    case NBTTag::TAG_Byte_Array:
    {
        int32_t size = UnpackBytesBE(4, stream);
        new (data) NBTByteArray();
        NBTByteArray& byte_array = *(NBTByteArray*)data;
        byte_array.reserve((size_t)size);
        for (int32_t index = 0; index < size; ++index)
            byte_array.push_back((int8_t)*stream++);
    } break;

    case NBTTag::TAG_String:
    {
        size_t string_size = (size_t)UnpackBytesBE(2, stream);
        new (data) std::string((char const*)stream, string_size);
        stream += string_size;
    } break;

    case NBTTag::TAG_List:
    {
        NBTList& list = *(NBTList*)data;
        list.type = (NBTTag)*stream++;
        list.size = (int32_t)UnpackBytesBE(4, stream);
        list.data = NBTDataAlloc(list.type, list.size);

        uint8_t* data = (uint8_t*)list.data;
        uint32_t element_size = NBTTypeSize(list.type);
        for (uint32_t index = 0; index < list.size; ++index)
            UnpackFieldData(list.type, data + element_size*index,
                            stream_base, stream_length,
                            stream);
    } break;

    case NBTTag::TAG_Compound:
    {
        new (data) NBTCompound;
        NBTCompound& compound = *(NBTCompound*)data;

        NBTTag element_type;
        for (;;)
        {
            element_type = (NBTTag)*stream++;
            if (element_type == NBTTag::TAG_End)
                break;

            compound.emplace_back();
            NBTField& field = compound.back();
            field.type = element_type;

            uint32_t name_length = UnpackBytesBE(2, stream);
            field.name = std::string((char*)stream, name_length);
            stream += name_length;

            field.payload = NBTDataAlloc(field.type, 1);
            UnpackFieldData(field.type, field.payload,
                            stream_base, stream_length,
                            stream);
        }
    } break;

    case NBTTag::TAG_Int_Array:
    {
        int32_t size = UnpackBytesBE(4, stream);
        new (data) NBTIntArray();
        NBTIntArray& int_array = *(NBTIntArray*)data;
        int_array.reserve((size_t)size);
        for (int32_t index = 0; index < size; ++index)
            int_array.push_back((int32_t)UnpackBytesBE(4, stream));
    } break;

    case NBTTag::TAG_Long_Array:
    {
        int32_t size = UnpackBytesBE(4, stream);
        new (data) NBTLongArray();
        NBTLongArray& long_array = *(NBTLongArray*)data;
        long_array.reserve((size_t)size);
        for (int32_t index = 0; index < size; ++index)
            long_array.push_back((int64_t)UnpackBytesBE(8, stream));
    } break;

    default: break;
    }
}

void PrintNBTField(NBTField const& field, uint32_t recursion);

void PrintNBTData(NBTTag type, void* data, uint32_t recursion, bool print_indent)
{
    std::string indent((size_t)recursion, '\t');
    if (print_indent)
        std::cout << indent;

    switch (type)
    {
    case NBTTag::TAG_Byte:
        std::cout << " (byte): " << (int32_t)*(int8_t*)data;
        break;

    case NBTTag::TAG_Short:
        std::cout << " (short): " << *(int16_t*)data;
        break;

    case NBTTag::TAG_Int:
        std::cout << " (int): " << *(int32_t*)data;
        break;

    case NBTTag::TAG_Long:
        std::cout << " (long): " << *(int64_t*)data;
        break;

    case NBTTag::TAG_Float:
        std::cout << " (float): " << *(float*)data;
        break;

    case NBTTag::TAG_Double:
        std::cout << " (double): " << *(double*)data;
        break;

    case NBTTag::TAG_Byte_Array:
    {
        std::cout << " (byte array): [";
        NBTByteArray const& byte_array = *(NBTByteArray*)data;
        for (auto element : byte_array)
            std::cout << (int32_t)element << ", ";
        std::cout << "]";
    } break;

    case NBTTag::TAG_String:
        std::cout << " (string): " << *(NBTString*)data;
        break;

    case NBTTag::TAG_List:
    {
        std::cout << " (list): [" << std::endl;
        NBTList const& list = *(NBTList*)data;

        uint8_t* list_data = (uint8_t*)list.data;
        uint32_t element_size = NBTTypeSize(list.type);
        for (uint32_t index = 0; index < list.size; ++index)
        {
            PrintNBTData(list.type,
                         list_data + element_size*index,
                         recursion+1, true);
            std::cout << std::endl;
        }

        std::cout << indent << "]";
    } break;

    case NBTTag::TAG_Compound:
    {
        std::cout << " (compound): {" << std::endl;
        NBTCompound const& compound = *(NBTCompound*)data;

        for (NBTField const& field : compound)
            PrintNBTField(field, recursion+1);

        std::cout << indent << "}";
    } break;

    case NBTTag::TAG_Int_Array:
    {
        std::cout << " (int array): [";
        NBTIntArray const& int_array = *(NBTIntArray*)data;
        for (auto element : int_array)
            std::cout << element << ", ";
        std::cout << "]";
    } break;

    case NBTTag::TAG_Long_Array:
    {
        std::cout << " (long array): [";
        NBTLongArray const& long_array = *(NBTLongArray*)data;
        for (auto element : long_array)
            std::cout << element << ", ";
        std::cout << "]";
    } break;

    default: break;
    }
}

void PrintNBTField(NBTField const& field, uint32_t recursion)
{
    std::string indent((size_t)recursion, '\t');
    std::cout << indent << field.name;

    PrintNBTData(field.type, field.payload, recursion, false);

    //if (field.type != NBTTag::TAG_Compound)
    std::cout << std::endl;
}

int main(int argc, char const** argv)
{
    if (argc == 2)
    {
        std::FILE* file = std::fopen(argv[1], "rb");
        std::fseek(file, 0, SEEK_END);
        uint64_t size = std::ftell(file);
        std::fseek(file, 0, SEEK_SET);
        std::string contents(size, '\0');
        std::fread(contents.data(), 1, size, file);
        std::fclose(file);

        std::uint8_t const* input_stream = (std::uint8_t const*)contents.data();
        ziptk::GZipHeader gzip_header = ziptk::ExtractGZip(input_stream);
        std::vector<uint8_t> output_stream = ziptk::Inflate(input_stream);

        uint8_t const* stream = output_stream.data();
        NBTTag current_field = NBTTag::TAG_End;
        std::vector<NBTField> field_list = {};
        while (stream - output_stream.data() < output_stream.size())
        {
            if (current_field == NBTTag::TAG_End)
                current_field = (NBTTag)*stream++;

            if (current_field == NBTTag::TAG_End)
            {
                std::cout << "compound end or something" << std::endl;
                continue;
            }

            field_list.emplace_back();
            NBTField& field = field_list.back();
            field.type = current_field;

            uint32_t name_length = UnpackBytesBE(2, stream);
            field.name = std::string((char*)stream, name_length);
            stream += name_length;

            field.payload = NBTDataAlloc(field.type, 1);
            UnpackFieldData(field.type, field.payload,
                            output_stream.data(), output_stream.size(),
                            stream);
            current_field = TAG_End;
        }

        std::cout << std::dec;
        for (NBTField const& field : field_list)
            PrintNBTField(field, 0);
    }

    return 0;
}
