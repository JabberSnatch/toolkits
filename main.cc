#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>
#include <bitset>
#include <functional>

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

std::vector<uint8_t> inflate(std::uint8_t const* stream)
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
                            uint32_t const base = kLengthBase[litlen_entry.value - 257];
                            uint32_t const extra_bits = kLengthExtraBits[litlen_entry.value - 257];
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

                        uint32_t begin = output_stream.size()-distance;
                        for (uint32_t byte_index = 0; byte_index < length; ++byte_index)
                            output_stream.push_back(
                                output_stream[begin+byte_index]);
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

    return output_stream;
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

    if (argc == 2)
    {
        std::FILE* file = std::fopen(argv[1], "rb");
        std::fseek(file, 0, SEEK_END);
        uint64_t size = std::ftell(file);
        std::fseek(file, 0, SEEK_SET);
        std::string contents(size, '\0');
        std::fread(contents.data(), 1, size, file);
        std::fclose(file);

        std::vector<uint8_t> output_stream = inflate((std::uint8_t const*)contents.data());

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
    }

    return 0;
}
