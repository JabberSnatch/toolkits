#pragma once

#include <cstdint>

namespace bmptk
{

enum class Result
{
    Success,
    UnknownHeaderField,
    UnknownBitmapVersion,
};

enum CompressionMethod
{
    BI_RGB = 0,
    BI_RLE8 = 1,
    BI_RLE4 = 2,
    BI_BITFIELDS = 3,
    BI_JPEG = 4,
    BI_PNG = 5,
    BI_ALPHABITFIELDS = 6,
    BI_CMYK = 11,
    BI_CMYKRLE8 = 12,
    BI_CMYKRLE4 = 13,
};

struct BitmapFile
{
    uint8_t const* memory;
    uint8_t const* pixel_data;
    uint32_t header_size;
    int32_t width;
    int32_t height;
    uint32_t bpp;
    CompressionMethod bi;
    uint32_t size;
};

struct BitmapV1Header
{
    int32_t width;
    int32_t height;
};

union PixelValue
{
    struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };
    uint8_t d[3];
};

Result LoadBMP(uint8_t const* _memory, BitmapFile* _bmpFile);
PixelValue GetPixel(BitmapFile const* _bmpFile, uint32_t x, uint32_t y);

uint32_t AllocSize(BitmapV1Header const* _header);
Result WriteBMP(BitmapV1Header const* _header, PixelValue const* _buffer, uint8_t* _memory);

#ifdef BMPTK_IMPLEMENTATION

#include <cstdlib>

template <typename T> static inline void const*
AdvancePointer(void const* _source, size_t _count = 1) {
    return ((T const*)_source) + _count;
}

template <typename T> static inline void*
AdvancePointer(void* _source, size_t _count = 1) {
    return ((T*)_source) + _count;
}

static inline uint8_t
ReadU8(void const*& _ptr) {
    uint8_t v = *(uint8_t const*)_ptr;
    _ptr = AdvancePointer<uint8_t>(_ptr);
    return v;
}

static inline uint16_t
ReadU16_BE(void const*& _ptr) {
    uint8_t const* base = (uint8_t const*)_ptr;
    _ptr = AdvancePointer<uint16_t>(_ptr);
    return (base[0]<<8) | base[1];
}

static inline uint32_t
ReadU32_BE(void const*& _ptr) {
    uint8_t const* base = (uint8_t const*)_ptr;
    _ptr = AdvancePointer<uint32_t>(_ptr);
    return (base[0]<<24) | (base[1]<<16) | (base[2]<<8) | base[3];
}

static inline uint16_t
ReadU16_LE(void const*& _ptr) {
    uint8_t const* base = (uint8_t const*)_ptr;
    _ptr = AdvancePointer<uint16_t>(_ptr);
    return (base[1]<<8) | base[0];
}

static inline uint32_t
ReadU32_LE(void const*& _ptr) {
    uint8_t const* base = (uint8_t const*)_ptr;
    _ptr = AdvancePointer<uint32_t>(_ptr);
    return (base[3]<<24) | (base[2]<<16) | (base[1]<<8) | base[0];
}

static inline void
WriteU8(void*& _ptr, uint8_t v) {
    *(uint8_t*)_ptr = v;
    _ptr = AdvancePointer<uint8_t>(_ptr);
}

static inline void
WriteU16_BE(void*& _ptr, uint16_t v) {
    uint8_t* base = (uint8_t*)_ptr;
    _ptr = AdvancePointer<uint16_t>(_ptr);
    base[0] = (v >> 8) & 0xff;
    base[1] = v & 0xff;
}

static inline void
WriteU32_BE(void*& _ptr, uint32_t v) {
    uint8_t* base = (uint8_t*)_ptr;
    _ptr = AdvancePointer<uint32_t>(_ptr);
    base[0] = (v >> 24) & 0xff;
    base[1] = (v >> 16) & 0xff;
    base[2] = (v >> 8) & 0xff;
    base[3] = v & 0xff;
}

static inline void
WriteU16_LE(void*& _ptr, uint16_t v) {
    uint8_t* base = (uint8_t*)_ptr;
    _ptr = AdvancePointer<uint16_t>(_ptr);
    base[0] = v & 0xff;
    base[1] = (v >> 8) & 0xff;
}

static inline void
WriteU32_LE(void*& _ptr, uint32_t v) {
    uint8_t* base = (uint8_t*)_ptr;
    _ptr = AdvancePointer<uint32_t>(_ptr);
    base[0] = v & 0xff;
    base[1] = (v >> 8) & 0xff;
    base[2] = (v >> 16) & 0xff;
    base[3] = (v >> 24) & 0xff;
}

Result LoadBMP(uint8_t const* _memory, BitmapFile* _bmpFile)
{
    _bmpFile->memory = _memory;

    void const* ptr = _memory;
    if (ReadU16_BE(ptr) != 0x424D /*BM*/)
        return Result::UnknownHeaderField;

    ReadU32_LE(ptr); // file size
    ReadU32_LE(ptr); // 2x16 reserved bits
    _bmpFile->pixel_data = _bmpFile->memory + ReadU32_LE(ptr);

    uint32_t header_size = ReadU32_LE(ptr);
    _bmpFile->header_size = header_size;

    if (header_size != 40u)
        return Result::UnknownBitmapVersion;

    _bmpFile->width = (int32_t)ReadU32_LE(ptr);
    _bmpFile->height = (int32_t)ReadU32_LE(ptr);

    ptr = AdvancePointer<uint16_t>(ptr);
    _bmpFile->bpp = (uint32_t)ReadU16_LE(ptr);

    _bmpFile->bi = (CompressionMethod)ReadU32_LE(ptr);
    _bmpFile->size = ReadU32_LE(ptr);

    return Result::Success;
}

PixelValue GetPixel(BitmapFile const* _bmpFile, uint32_t x, uint32_t y)
{
    uint8_t const* v = _bmpFile->pixel_data + ((y * _bmpFile->width + x) * (_bmpFile->bpp/8));
    return PixelValue{ v[2], v[1], v[0] };
}

uint32_t AllocSize(BitmapV1Header const* _header)
{
    return 54u + abs((int32_t)_header->width*_header->height*4);
}

Result WriteBMP(BitmapV1Header const* _header, PixelValue const* _buffer, uint8_t* _memory)
{
    void* ptr = _memory;
    WriteU16_BE(ptr, 0x424D);
    WriteU32_LE(ptr, AllocSize(_header));
    WriteU32_LE(ptr, 0u);
    WriteU32_LE(ptr, 54u);

    WriteU32_LE(ptr, 40u);
    WriteU32_LE(ptr, _header->width);
    WriteU32_LE(ptr, _header->height);
    WriteU16_LE(ptr, 1u);
    WriteU16_LE(ptr, 32u);
    WriteU32_LE(ptr, 0u);
    WriteU32_LE(ptr, (uint32_t)abs(_header->width*_header->height*4));
    WriteU32_LE(ptr, 0u);
    WriteU32_LE(ptr, 0u);
    WriteU32_LE(ptr, 0u);
    WriteU32_LE(ptr, 0u);

    uint8_t* pixel_base = _memory + 54u;
    for (uint32_t index = 0u; index < abs(_header->width*_header->height); ++index)
    {
        pixel_base[index * 4] = _buffer[index].blue;
        pixel_base[index * 4 + 1] = _buffer[index].green;
        pixel_base[index * 4 + 2] = _buffer[index].red;
        pixel_base[index * 4 + 3] = '\0';
    }

    return Result::Success;
}


#endif

} // namespace bmptk
