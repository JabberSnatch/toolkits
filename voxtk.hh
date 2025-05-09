#pragma once

#include "numtk.hh"
#include "dstk.hh"

namespace voxtk
{

struct VoxelMask
{
    static constexpr int32_t kSize = 8;
    static constexpr int32_t kLogSize = 3;
    static constexpr uint32_t kSizeMask = 0x7u;
    static constexpr int32_t kVolume = kSize*kSize*kSize;

    static uint64_t PackFillMask(numtk::vec3u const& begin);

    static VoxelMask FillAbove(numtk::vec3u const& begin);
    static VoxelMask FillBelow(numtk::vec3u const& end);
    static VoxelMask FillArea(numtk::vec3u const& begin, numtk::vec3u const& end);

    static uint16_t BitIndex(numtk::vec3u const& point);
    static numtk::vec3u PointLocation(uint16_t index);

    VoxelMask& Clear();
    VoxelMask& BitReverse();

    VoxelMask BitwiseAnd(VoxelMask const& o) const;
    VoxelMask BitwiseOr(VoxelMask const& o) const;
    VoxelMask BitwiseNot() const;

    VoxelMask Shift(numtk::vec3i const& shift);

    bool Test(numtk::vec3u const& point) const;
    uint64_t ExtractKernel(numtk::vec3u const& base) const;

    VoxelMask& Set(numtk::vec3u const& point, bool v);
    VoxelMask& Set(numtk::vec3u const& begin, numtk::vec3u const& end, bool v);

    uint16_t NextIndex(uint16_t index = ~(uint16_t)0) const;

    bool Full() const;
    bool Empty() const;

    uint64_t bits[8];
};


#define DENSE_CHILDREN_ARRAY
template <typename DataType>
struct Region
{
    struct Node;

    Region(numtk::vec3u _size, DataType const& _default_value);
    Region(Region&&) = default;
    Region& operator=(Region&&) = default;
    Region(Region const& _other);

    DataType const& operator[](uint32_t const& index) const;
    DataType const& operator[](numtk::vec3u const& point) const;

    bool Test(numtk::vec3u const& point) const;

    void Set(numtk::vec3u const& point, bool v);
    void Set(numtk::vec3u const& point, DataType const& value);
    void Set(numtk::vec3u const& begin, numtk::vec3u const& end, DataType const& value);
    void Set(numtk::vec3u const& begin, voxtk::Region<DataType> const& region);

    void Clear(numtk::vec3u const& begin, numtk::vec3u const& end);

    Node* FindLeaf(numtk::vec3u const& point) const;
    uint64_t ExtractKernel(numtk::vec3u const& base) const;

    struct Node {
        Node* parent;
        numtk::vec3u location;
        uint32_t depth;

        numtk::vec3u Begin() const { return location * (1u << (3*(depth+1))); }
        numtk::vec3u End() const { return Begin() + numtk::vec3u::Constant(8u); }

        bool Contains(numtk::vec3u const& point) const;
        void SetData(numtk::vec3u const& point, DataType const& value);
        void SetData(numtk::vec3u const& begin, numtk::vec3u const& end, DataType const& value);

        numtk::vec3u LocalPoint(numtk::vec3u const& point) const;
        uint16_t ChildIndex(numtk::vec3u const& child);

        VoxelMask child_mask{};
        VoxelMask data_mask{};
        std::vector<DataType> data{};
#ifndef DENSE_CHILDREN_ARRAY
        dstk::OrderedVector<uint16_t, Node*> children{};
#else
        std::vector<Node*> children = std::vector<Node*>(VoxelMask::kVolume);
#endif
    };

    Node* InsertChild(Node* parent, numtk::vec3u local_point);

    Node* root = nullptr;
    dstk::ObjectPool<Node> node_pool{};
    uint32_t level_count;
    numtk::vec3u size;
    DataType default_value;
};

} // namespace voxtk

namespace voxtk
{

template <typename DataType>
Region<DataType>::Region(numtk::vec3u _size, DataType const& _default_value)
    : level_count{ 0u }
    , size{ _size }
    , default_value{ _default_value }
{
    uint32_t max_size = std::max(std::max(_size.x, _size.y), _size.z);
    uint32_t log_size = numtk::ilogN<VoxelMask::kSize>(max_size);
    if (1u << (VoxelMask::kLogSize * log_size) < max_size)
        ++log_size;
    level_count = std::max(log_size, 1u);

    root = node_pool[node_pool.Reserve()];
    root->parent = nullptr;
    root->location = numtk::vec3u{ 0, 0, 0 };
    root->depth = level_count - 1;
}

template <typename DataType>
Region<DataType>::Region(Region const& _other)
    : Region<DataType>{ _other.size, _other.default_value }
{
    *root = *_other.root;
    std::vector<Node*> node_queue = { root };

    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        uint16_t child_index = current_node->child_mask.NextIndex();
        while (child_index < 512)
        {
            Node* new_node = node_pool[node_pool.Reserve()];
            *new_node = *(current_node->children[child_index]);
            current_node->children[child_index] = new_node;
            node_queue.push_back(new_node);
            child_index = current_node->child_mask.NextIndex(child_index);
        }
    }
}

template <typename DataType>
DataType const& Region<DataType>::operator[](uint32_t const& index) const
{
    numtk::vec3u point{};
    point.x = index % size.x;
    point.y = (index / size.x) % size.y;
    point.z = index / (size.x * size.y);
    return (*this)[point];
}

template <typename DataType>
DataType const& Region<DataType>::operator[](numtk::vec3u const& point) const
{
    if (point.x >= size.x
        || point.y >= size.y
        || point.z >= size.z)
        return default_value;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(point);
        if (current_node->child_mask.Test(local_point))
        {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }
        else if (current_node->data_mask.Test(local_point))
        {
            if (current_node->data.empty())
                return default_value;

            uint16_t const data_index = VoxelMask::BitIndex(local_point);
            return current_node->data[data_index];
        }
        else
            return default_value;
    }

    return default_value;
}

template <typename DataType>
bool Region<DataType>::Test(numtk::vec3u const& point) const
{
    if (point.x >= size.x
        || point.y >= size.y
        || point.z >= size.z)
        return false;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(point);
        if (current_node->child_mask.Test(local_point))
        {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }
        else
            return current_node->data_mask.Test(local_point);
    }

    return false;
}

template <typename DataType>
void Region<DataType>::Set(numtk::vec3u const& point, bool v)
{
    if (point.x >= size.x
        || point.y >= size.y
        || point.z >= size.z)
        return;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(point);

        if (current_node->depth == 0)
        {
            current_node->data_mask.Set(local_point, v);
            break;
        }

        else if (current_node->child_mask.Test(local_point))
        {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }

        else if (current_node->data_mask.Test(local_point) != v)
            current_node = InsertChild(current_node, local_point);

        else
            return;
    }
}

template <typename DataType>
void Region<DataType>::Set(numtk::vec3u const& point, DataType const& value)
{
    if (point.x >= size.x
        || point.y >= size.y
        || point.z >= size.z)
        return;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(point);

        if (current_node->depth == 0)
        {
            current_node->data_mask.Set(local_point, true);
            current_node->SetData(local_point, value);
            break;
        }

        else if (current_node->child_mask.Test(local_point))
        {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }

        else
            current_node = InsertChild(current_node, local_point);

    }
}

template <typename DataType>
void Region<DataType>::Set(numtk::vec3u const& begin, numtk::vec3u const& end, DataType const& value)
{
    if (begin.x >= size.x
        || begin.y >= size.y
        || begin.z >= size.z)
        return;

    std::vector<Node*> node_queue{ root };
    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        if (current_node->depth == 0)
        {
            numtk::vec3u const data_begin =
                numtk::max(begin, current_node->Begin()) - current_node->Begin();
            numtk::vec3u const data_end =
                numtk::min(end, current_node->End()) - current_node->Begin();

            current_node->data_mask.Set(data_begin, data_end, true);
            current_node->SetData(data_begin, data_end, value);
        }
        else
        {
            numtk::vec3u const children_begin =
                numtk::max(begin >> (3*(current_node->depth)),
                           current_node->Begin());
            numtk::vec3u const children_end =
                numtk::min((end >> (3*(current_node->depth)))
                           + numtk::vec3u::Constant(1u),
                           current_node->End());

            for (uint32_t x = children_begin.x; x < children_end.x; ++x)
                for (uint32_t y = children_begin.y; y < children_end.y; ++y)
                    for (uint32_t z = children_begin.z; z < children_end.z; ++z)
                    {
                        numtk::vec3u const child_location{ x, y, z };
                        uint16_t child_index = current_node->ChildIndex(child_location);

                        if (!current_node->child_mask.Test(child_location))
                            InsertChild(current_node, child_location);

                        node_queue.push_back(current_node->children[child_index]);
                    }
        }
    }
}

template <typename DataType>
void Region<DataType>::Set(numtk::vec3u const& begin, voxtk::Region<DataType> const& region)
{
    if (begin.x >= size.x
        || begin.y >= size.y
        || begin.z >= size.z)
        return;
}

template <typename DataType>
void Region<DataType>::Clear(numtk::vec3u const& begin, numtk::vec3u const& end)
{
    if (begin.x >= size.x
        || begin.y >= size.y
        || begin.z >= size.z)
        return;

    if (!root)
        return;

    std::vector<Node*> node_queue{ root };
    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        if (current_node->depth == 0)
        {
            numtk::vec3u const data_begin =
                numtk::max(begin, current_node->Begin()) - current_node->Begin();
            numtk::vec3u const data_end =
                numtk::min(end, current_node->End()) - current_node->Begin();

            current_node->data_mask.Set(data_begin, data_end, false);
        }
        else
        {
            numtk::vec3u const children_begin =
                numtk::max(begin >> (3*(current_node->depth)),
                           current_node->Begin());
            numtk::vec3u const children_end =
                numtk::min((end >> (3*(current_node->depth)))
                           + numtk::vec3u::Constant(1u),
                           current_node->End());

            for (uint32_t x = children_begin.x; x < children_end.x; ++x)
                for (uint32_t y = children_begin.y; y < children_end.y; ++y)
                    for (uint32_t z = children_begin.z; z < children_end.z; ++z)
                    {
                        numtk::vec3u const child{ x, y, z };
                        uint16_t child_index = current_node->ChildIndex(child);
                        if (current_node->child_mask.Test(child))
                            node_queue.push_back(current_node->children[child_index]);
                    }
        }
    }
}

template <typename DataType>
typename Region<DataType>::Node* Region<DataType>::FindLeaf(numtk::vec3u const& point) const
{
    if (point.x >= size.x
        || point.y >= size.y
        || point.z >= size.z)
        return nullptr;

    if (!root)
        return nullptr;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(point);
        if (!current_node->child_mask.Test(local_point))
            break;
        else
        {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }
    }

    return current_node;
}

template <typename DataType>
uint64_t Region<DataType>::ExtractKernel(numtk::vec3u const& base) const
{
    Node const* leaf = FindLeaf(base);
    if (!leaf->depth)
        return leaf->data_mask.ExtractKernel(leaf->LocalPoint(base));
    else
        return leaf->child_mask.Test(leaf->LocalPoint(base))
            ? ~(uint64_t)0
            : 0;
}

template <typename DataType>
typename Region<DataType>::Node* Region<DataType>::InsertChild(Region<DataType>::Node* parent,
                                                               numtk::vec3u local_point)
{
    uint16_t point_index = parent->ChildIndex(local_point);

#ifndef DENSE_CHILDREN_ARRAY
    auto child_node =
        parent->children.insert(point_index, node_pool[node_pool.Reserve()]);

    (*child_node)->parent = parent;
    (*child_node)->location = local_point;
    (*child_node)->depth = parent->depth - 1;
    parent->child_mask.Set(local_point, true);

    return *child_node;
#else
    Node* child_node = node_pool[node_pool.Reserve()];
    parent->children[point_index] = child_node;

    child_node->parent = parent;
    child_node->location = local_point;
    child_node->depth = parent->depth - 1;
    parent->child_mask.Set(local_point, true);

    return child_node;
#endif
}

template <typename DataType>
bool Region<DataType>::Node::Contains(numtk::vec3u const& point) const
{
    numtk::vec3u const begin = Begin();
    numtk::vec3u const end = End();
    return
        point.x >= begin.x
        && point.y >= begin.y
        && point.z >= begin.z
        && point.x < end.x
        && point.y < end.y
        && point.z < end.z;
}

template <typename DataType>
void Region<DataType>::Node::SetData(numtk::vec3u const& point, DataType const& value)
{
    if (data.empty()) data.resize(VoxelMask::kVolume);
    uint16_t const index = VoxelMask::BitIndex(point);
    data[index] = value;
}

template <typename DataType>
void Region<DataType>::Node::SetData(numtk::vec3u const& begin, numtk::vec3u const& end, DataType const& value)
{
    if (data.empty()) data.resize(VoxelMask::kVolume);
    for (uint32_t z = begin.z; z < end.z; ++z)
        for (uint32_t y = begin.y; y < end.y; ++y)
            for (uint32_t x = begin.x; x < end.x; ++x)
            {
                uint16_t const index = VoxelMask::BitIndex(numtk::vec3u{ x, y, z });
                data[index] = value;
            }
}

template <typename DataType>
numtk::vec3u Region<DataType>::Node::LocalPoint(numtk::vec3u const& point) const {
    return (point >> (3 * depth)) & VoxelMask::kSizeMask;
}

template <typename DataType>
uint16_t Region<DataType>::Node::ChildIndex(numtk::vec3u const& child) {
    return VoxelMask::BitIndex(child);
}


} // namespace voxtk

#ifdef VOXTK_IMPLEMENTATION

namespace voxtk
{

uint64_t VoxelMask::PackFillMask(numtk::vec3u const& begin)
{
    uint64_t mask = 0ull;

    if (begin.x == 0 && begin.y == 0 && begin.z == 0)
        mask = ~0ull;
    else if (begin.x == 0 && begin.y == 0)
        mask = ~((1ull << begin.z*16) - 1ull);
    else if (begin.x == 0)
    {
        uint64_t plane = ~((1ull << begin.y*4) - 1ull) & 0xffffull;
        for (uint32_t z = begin.z; z < 4; ++z)
            mask |= plane << (z*16);
    }
    else
    {
        uint64_t row = ~((1ull << begin.x) - 1ull) & 0xfull;
        for (uint32_t z = begin.z; z < 4; ++z)
            for (uint32_t y = begin.y; y < 4; ++y)
                mask |= row << (y*4 + z*16);
    }

    return mask;
}

VoxelMask VoxelMask::FillAbove(numtk::vec3u const& begin)
{
    VoxelMask output = {};

    if (begin.x > 7u || begin.y > 7u || begin.z > 7u)
        return output;

    numtk::vec3u const pack_begin = begin >> 2u;/// 4u;
    numtk::vec3u const bits_begin = begin & 0x3u;//% 4u;

    numtk::vec3u quadrant0 = pack_begin;
    output.bits[quadrant0.x + quadrant0.y*2 + quadrant0.z*4] = PackFillMask(bits_begin);

    if (quadrant0.x == 0)
    {
        numtk::vec3u const quadrant1{ 1u, quadrant0.y, quadrant0.z };
        output.bits[quadrant1.x + quadrant1.y*2 + quadrant1.z*4] =
            PackFillMask(numtk::vec3u{ 0, bits_begin.y, bits_begin.z });
    }

    if (quadrant0.y == 0)
    {
        numtk::vec3u const quadrant2{ quadrant0.x, 1u, quadrant0.z };
        output.bits[quadrant2.x + quadrant2.y*2 + quadrant2.z*4] =
            PackFillMask(numtk::vec3u{ bits_begin.x, 0, bits_begin.z });
    }

    if (quadrant0.z == 0)
    {
        numtk::vec3u const quadrant3{ quadrant0.x, quadrant0.y, 1u };
        output.bits[quadrant3.x + quadrant3.y*2 + quadrant3.z*4] =
            PackFillMask(numtk::vec3u{ bits_begin.x, bits_begin.y, 0 });
    }

    if (quadrant0.x == 0 && quadrant0.y == 0)
    {
        numtk::vec3u const quadrant4{ 1u, 1u, quadrant0.z };
        output.bits[quadrant4.x + quadrant4.y*2 + quadrant4.z*4] =
            PackFillMask(numtk::vec3u{ 0, 0, bits_begin.z });
    }

    if (quadrant0.y == 0 && quadrant0.z == 0)
    {
        numtk::vec3u const quadrant5{ quadrant0.x, 1u, 1u };
        output.bits[quadrant5.x + quadrant5.y*2 + quadrant5.z*4] =
            PackFillMask(numtk::vec3u{ bits_begin.x, 0, 0 });
    }

    if (quadrant0.x == 0 && quadrant0.z == 0)
    {
        numtk::vec3u const quadrant6{ 1u, quadrant0.y, 1u };
        output.bits[quadrant6.x + quadrant6.y*2 + quadrant6.z*4] =
            PackFillMask(numtk::vec3u{ 0, bits_begin.y, 0 });
    }

    if (quadrant0.x == 0 && quadrant0.y == 0 && quadrant0.z == 0)
        output.bits[7] = ~0ull;

    return output;
}

VoxelMask VoxelMask::FillBelow(numtk::vec3u const& end)
{
    if (end.x == 0 || end.y == 0 || end.z == 0)
        return VoxelMask{};

    return FillAbove(numtk::vec3u{ 8u, 8u, 8u, } - end).BitReverse();
}

VoxelMask VoxelMask::FillArea(numtk::vec3u const& begin, numtk::vec3u const& end)
{
    return FillBelow(end).BitwiseAnd(FillAbove(begin));
}

uint16_t VoxelMask::BitIndex(numtk::vec3u const& point)
{
    // 2 wide blocks of 4 wide blocks
    // 1 bit for the toplevel, 2 bits for the bottomlevel
    return (uint16_t)(
        numtk::BitDeposit(point.x, 0b1000011) |
        numtk::BitDeposit(point.y, 0b10001100) |
        numtk::BitDeposit(point.z, 0b100110000));
}

numtk::vec3u VoxelMask::PointLocation(uint16_t index)
{
    return numtk::vec3u{
        numtk::BitExtract((uint32_t)index, 0b1000011),
        numtk::BitExtract((uint32_t)index, 0b100011000),
        numtk::BitExtract((uint32_t)index, 0b100110000)
    };
}

VoxelMask& VoxelMask::Clear()
{
    bits[0] = 0ull;
    bits[1] = 0ull;
    bits[2] = 0ull;
    bits[3] = 0ull;
    bits[4] = 0ull;
    bits[5] = 0ull;
    bits[6] = 0ull;
    bits[7] = 0ull;

    return *this;
}

VoxelMask& VoxelMask::BitReverse()
{
    bits[0] = numtk::BitReverse(bits[0]);
    bits[1] = numtk::BitReverse(bits[1]);
    bits[2] = numtk::BitReverse(bits[2]);
    bits[3] = numtk::BitReverse(bits[3]);
    bits[4] = numtk::BitReverse(bits[4]);
    bits[5] = numtk::BitReverse(bits[5]);
    bits[6] = numtk::BitReverse(bits[6]);
    bits[7] = numtk::BitReverse(bits[7]);
    std::swap(bits[0], bits[7]);
    std::swap(bits[1], bits[6]);
    std::swap(bits[2], bits[5]);
    std::swap(bits[3], bits[4]);

    return *this;
}

VoxelMask VoxelMask::BitwiseAnd(VoxelMask const& o) const
{
    VoxelMask output{};
    output.bits[0] = bits[0] & o.bits[0];
    output.bits[1] = bits[1] & o.bits[1];
    output.bits[2] = bits[2] & o.bits[2];
    output.bits[3] = bits[3] & o.bits[3];
    output.bits[4] = bits[4] & o.bits[4];
    output.bits[5] = bits[5] & o.bits[5];
    output.bits[6] = bits[6] & o.bits[6];
    output.bits[7] = bits[7] & o.bits[7];
    return output;
}

VoxelMask VoxelMask::BitwiseOr(VoxelMask const& o) const
{
    VoxelMask output{};
    output.bits[0] = bits[0] | o.bits[0];
    output.bits[1] = bits[1] | o.bits[1];
    output.bits[2] = bits[2] | o.bits[2];
    output.bits[3] = bits[3] | o.bits[3];
    output.bits[4] = bits[4] | o.bits[4];
    output.bits[5] = bits[5] | o.bits[5];
    output.bits[6] = bits[6] | o.bits[6];
    output.bits[7] = bits[7] | o.bits[7];
    return output;
}

VoxelMask VoxelMask::BitwiseNot() const
{
    VoxelMask output{};
    output.bits[0] = ~bits[0];
    output.bits[1] = ~bits[1];
    output.bits[2] = ~bits[2];
    output.bits[3] = ~bits[3];
    output.bits[4] = ~bits[4];
    output.bits[5] = ~bits[5];
    output.bits[6] = ~bits[6];
    output.bits[7] = ~bits[7];
    return output;
}

VoxelMask VoxelMask::Shift(numtk::vec3i const& shift)
{
    /*
       0  1  2  3
       4  5  6  7
       8  9 10 11
      12 13 14 15

      // X shift
       x  0  1  2
       x  4  5  6
       x  8  9 10
       x 12 13 14

      // Y shift
       x  x  x  x
       0  1  2  3
       4  5  6  7
       8  9 10 11

      // Z shift
       x  x  x  x
       x  x  x  x
       x  x  x  x
       x  x  x  x
     */

    static constexpr uint64_t kBroadcastU4 = 0x1111'1111'1111'1111ull;
    static constexpr uint64_t kBroadcastU16 = 0x0001'0001'0001'0001ull;

    if (shift.x > 0)
    {
        uint64_t const xdstmask = ((0xfull << shift.x) & 0xfull) * kBroadcastU4;
        uint64_t const xsrcmask = (0xfull >> shift.x) * kBroadcastU4;
        uint64_t const xdstcarry = ((0xfull >> (4-shift.x)) & 0xfull) * kBroadcastU4;
        uint64_t const xsrccarry = (0xfull << (4-shift.x)) * kBroadcastU4;

        if (shift.x < 4)
        {
            bits[7] = numtk::BitDeposit(numtk::BitExtract(bits[7], xsrcmask), xdstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[6], xsrccarry), xdstcarry);
            bits[6] = numtk::BitDeposit(numtk::BitExtract(bits[6], xsrcmask), xdstmask);

            bits[5] = numtk::BitDeposit(numtk::BitExtract(bits[5], xsrcmask), xdstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[4], xsrccarry), xdstcarry);
            bits[4] = numtk::BitDeposit(numtk::BitExtract(bits[4], xsrcmask), xdstmask);

            bits[3] = numtk::BitDeposit(numtk::BitExtract(bits[3], xsrcmask), xdstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[2], xsrccarry), xdstcarry);
            bits[2] = numtk::BitDeposit(numtk::BitExtract(bits[2], xsrcmask), xdstmask);

            bits[1] = numtk::BitDeposit(numtk::BitExtract(bits[1], xsrcmask), xdstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[0], xsrccarry), xdstcarry);
            bits[0] = numtk::BitDeposit(numtk::BitExtract(bits[0], xsrcmask), xdstmask);
        }
        else
        {
            bits[7] = numtk::BitDeposit(numtk::BitExtract(bits[6], xsrcmask), xdstmask);
            bits[6] = 0ull;

            bits[5] = numtk::BitDeposit(numtk::BitExtract(bits[4], xsrcmask), xdstmask);
            bits[4] = 0ull;

            bits[3] = numtk::BitDeposit(numtk::BitExtract(bits[2], xsrcmask), xdstmask);
            bits[2] = 0ull;

            bits[1] = numtk::BitDeposit(numtk::BitExtract(bits[0], xsrcmask), xdstmask);
            bits[0] = 0ull;
        }
    }

    if (shift.y > 0)
    {
        uint64_t const ydstmask = ((0xffffull << (shift.y * 4)) & 0xffffull) * kBroadcastU16;
        uint64_t const ysrcmask = (0xffffull >> (shift.y * 4)) * kBroadcastU16;
        uint64_t const ydstcarry = ((0xffffull >> ((4-shift.y) * 4)) & 0xffffull) * kBroadcastU16;
        uint64_t const ysrccarry = (0xffffull << ((4-shift.y) * 4)) * kBroadcastU16;

        if (shift.y < 4)
        {
            bits[7] = numtk::BitDeposit(numtk::BitExtract(bits[7], ysrcmask), ydstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[3], ysrccarry), ydstcarry);

            bits[6] = numtk::BitDeposit(numtk::BitExtract(bits[6], ysrcmask), ydstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[2], ysrccarry), ydstcarry);

            bits[5] = numtk::BitDeposit(numtk::BitExtract(bits[5], ysrcmask), ydstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[1], ysrccarry), ydstcarry);

            bits[4] = numtk::BitDeposit(numtk::BitExtract(bits[4], ysrcmask), ydstmask)
                | numtk::BitDeposit(numtk::BitExtract(bits[0], ysrccarry), ydstcarry);

            bits[3] = numtk::BitDeposit(numtk::BitExtract(bits[3], ysrcmask), ydstmask);
            bits[2] = numtk::BitDeposit(numtk::BitExtract(bits[2], ysrcmask), ydstmask);
            bits[1] = numtk::BitDeposit(numtk::BitExtract(bits[1], ysrcmask), ydstmask);
            bits[0] = numtk::BitDeposit(numtk::BitExtract(bits[0], ysrcmask), ydstmask);
        }
        else
        {
            bits[7] = numtk::BitDeposit(numtk::BitExtract(bits[3], ysrcmask), ydstmask);
            bits[6] = numtk::BitDeposit(numtk::BitExtract(bits[2], ysrcmask), ydstmask);
            bits[5] = numtk::BitDeposit(numtk::BitExtract(bits[1], ysrcmask), ydstmask);
            bits[4] = numtk::BitDeposit(numtk::BitExtract(bits[0], ysrcmask), ydstmask);

            bits[3] = 0ull;
            bits[2] = 0ull;
            bits[1] = 0ull;
            bits[0] = 0ull;
        }

    }

    if (shift.z > 0)
    {
        bits[0] = (shift.z < 4) ? bits[0] << (shift.z * 16) : 0ull;
        bits[1] = (shift.z < 4) ? bits[1] << (shift.z * 16) : 0ull;
        bits[2] = (shift.z < 4) ? bits[2] << (shift.z * 16) : 0ull;
        bits[3] = (shift.z < 4) ? bits[3] << (shift.z * 16) : 0ull;
        bits[4] = (shift.z < 4) ? bits[4] << (shift.z * 16) : 0ull;
        bits[5] = (shift.z < 4) ? bits[5] << (shift.z * 16) : 0ull;
        bits[6] = (shift.z < 4) ? bits[6] << (shift.z * 16) : 0ull;
        bits[7] = (shift.z < 4) ? bits[7] << (shift.z * 16) : 0ull;
    }

    return *this;
}

bool VoxelMask::Test(numtk::vec3u const& point) const
{
    uint16_t bit_index = BitIndex(point);
    uint16_t pack_index = bit_index / 64;
    bit_index = bit_index & 63;
    return !!((bits[pack_index] >> bit_index) & 1);
}

uint64_t VoxelMask::ExtractKernel(numtk::vec3u const& base) const
{
    uint16_t pack_index = BitIndex(base) / 64;
    return bits[pack_index];
}

VoxelMask& VoxelMask::Set(numtk::vec3u const& point, bool v)
{
    uint16_t bit_index = BitIndex(point);
    uint16_t pack_index = bit_index / 64;
    bit_index = bit_index & 63;

    if (v)
        bits[pack_index] |= (1ull << bit_index);
    else
        bits[pack_index] &= ~(1ull << bit_index);

    return *this;
}

VoxelMask& VoxelMask::Set(numtk::vec3u const& begin, numtk::vec3u const& end, bool v)
{
    VoxelMask mask{};
    mask.FillArea(begin, end);
    if (v)
        *this = BitwiseOr(mask);
    else
    {
        mask = mask.BitwiseNot();
        *this = BitwiseAnd(mask);
    }

    return *this;
}

uint16_t VoxelMask::NextIndex(uint16_t index) const
{
    ++index;
    uint16_t first_pack = index / 64;

    while (first_pack < 8 && !(bits[first_pack] >> (index % 64))){
        ++first_pack;
        index = first_pack * 64;
    }
    if (first_pack >= 8)
        return 512;

    uint16_t tzc = numtk::BitTrailingZeroCount(bits[first_pack] >> (index % 64));
    return index + tzc;
}

bool VoxelMask::Full() const
{
    return
        bits[0] == 0xffffffffffffffffull
        && bits[1] == 0xffffffffffffffffull
        && bits[2] == 0xffffffffffffffffull
        && bits[3] == 0xffffffffffffffffull
        && bits[4] == 0xffffffffffffffffull
        && bits[5] == 0xffffffffffffffffull
        && bits[6] == 0xffffffffffffffffull
        && bits[7] == 0xffffffffffffffffull;
}

bool VoxelMask::Empty() const
{
    return
        !bits[0]
        && !bits[1]
        && !bits[2]
        && !bits[3]
        && !bits[4]
        && !bits[5]
        && !bits[6]
        && !bits[7];
}

} // namespace voxtk

#endif
