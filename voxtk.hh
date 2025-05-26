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

    static constexpr VoxelMask kFullMask() { return VoxelMask{ ~0ull, ~0ull, ~0ull, ~0ull, ~0ull, ~0ull, ~0ull, ~0ull }; }
    static constexpr VoxelMask kEmptyMask() { return VoxelMask{ 0ull, 0ull, 0ull, 0ull, 0ull, 0ull, 0ull, 0ull }; }

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

    VoxelMask SelectOctant(numtk::vec3u const& junction, uint32_t index) const;

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

    static numtk::vec3u CellLocation(numtk::vec3u const& point) { return point / VoxelMask::kSize; }
    static numtk::vec3u CellBegin(numtk::vec3u const& cell) { return cell * VoxelMask::kSize; }

    Region(numtk::vec3u _size, DataType const& _default_value);
    Region(Region&&) = default;
    Region& operator=(Region&&) = default;
    Region(Region const& _other);
    Region& operator=(Region const&);

    DataType const& operator[](uint32_t const& index) const;
    DataType const& operator[](numtk::vec3u const& point) const;

    bool Test(numtk::vec3u const& point) const;

    void Set(numtk::vec3u const& point, bool v);
    void Set(numtk::vec3u const& point, DataType const& value);
    void Set(numtk::vec3u const& begin, numtk::vec3u const& end, DataType const& value);
    void Set(numtk::vec3u const& begin, voxtk::Region<DataType> const& region);

    void Clear() { Clear(numtk::vec3u::Constant(0), size); }
    void Clear(numtk::vec3u const& begin, numtk::vec3u const& end);

    Region BitwiseAnd(Region const& other, numtk::vec3u const& offset) const;

    Region Shift(numtk::vec3u const& offset) const;

    Node* MakeCell(numtk::vec3u const& cell);
    VoxelMask GetCell(numtk::vec3u const& cell) const;
    void SetCell(numtk::vec3u const& cell, VoxelMask const& mask);

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
Region<DataType>& Region<DataType>::operator=(Region const& _other)
{
    Clear();
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

    return *this;
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
Region<DataType> Region<DataType>::BitwiseAnd(Region const& other, numtk::vec3u const& offset) const
{
    Region<DataType> output(size, default_value);

    numtk::bounds3u const src_bounds{ numtk::vec3u::Constant(0), size };
    numtk::bounds3u const dst_bounds{ offset, other.size };
    numtk::bounds3u const op_bounds = src_bounds.Intersection(dst_bounds);

    // align begin + compute offset
    numtk::vec3u const first_cell = Region<DataType>::CellLocation(op_bounds.min);
    numtk::vec3u const last_cell =
        Region<DataType>::CellLocation(op_bounds.min + op_bounds.extent + numtk::vec3u::Constant(VoxelMask::kSize-1));
    numtk::vec3i const bit_offset = (op_bounds.min - Region<DataType>::CellBegin(first_cell)).cast<int32_t>();

    // step through all cells
    numtk::vec3u const dst_first_cell = first_cell;
    numtk::vec3u const src_first_cell = numtk::vec3u::Constant(0);
    numtk::vec3u const dst_last_cell = Region<DataType>::CellLocation(other.size + numtk::vec3u::Constant(VoxelMask::kSize-1));

    numtk::vec3u const cell_extent = last_cell - first_cell;
    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                // get voxel mask from src
                numtk::vec3u const src_cell = src_first_cell + cell_index;
                VoxelMask src_mask = other.GetCell(src_cell);
                // get voxel mask from dst
                numtk::vec3u const dst_cell = dst_first_cell + cell_index;
                VoxelMask dst_mask = GetCell(dst_cell);
                // align and mask
                src_mask = src_mask.Shift(bit_offset).BitwiseAnd(VoxelMask::FillAbove(bit_offset.cast<uint32_t>()));
                // apply operator
                output.SetCell(dst_cell, dst_mask.BitwiseAnd(src_mask));
            }

    // update hierarchy
    // ???
    // PROFIT

    return output;
}

template <typename DataType>
Region<DataType> Region<DataType>::Shift(numtk::vec3u const& offset) const
{
    Region output{ size + offset, default_value };

    numtk::vec3i const bit_offset = offset.cast<int32_t>() % VoxelMask::kSize;
    numtk::vec3i const cell_offset = offset.cast<int32_t>() / VoxelMask::kSize;

    numtk::vec3u const cell_extent = Region::CellLocation(size + numtk::vec3u::Constant(VoxelMask::kSize-1));
    numtk::vec3u const output_max_cell = Region::CellLocation(output.size + numtk::vec3u::Constant(VoxelMask::kSize-1));
    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                numtk::vec3u const dst_cell_base = cell_index + cell_offset.cast<uint32_t>();
                VoxelMask src_mask = GetCell(cell_index).Shift(bit_offset);
                for (uint32_t mask_index = 0; mask_index < 8; ++mask_index)
                {
                    numtk::vec3u const mask_offset{ mask_index & 1, (mask_index >> 1) & 1, mask_index >> 2 };
                    numtk::vec3u const dst_cell = dst_cell_base + mask_offset;
                    if (dst_cell.x >= output_max_cell.x
                        || dst_cell.y >= output_max_cell.y
                        || dst_cell.z >= output_max_cell.z)
                        continue;

                    VoxelMask dst_mask = output.GetCell(dst_cell);
                    dst_mask = dst_mask.BitwiseOr(src_mask.SelectOctant(bit_offset.cast<uint32_t>(), (~mask_index) & 0x7));
                    output.SetCell(dst_cell, dst_mask);
                }
            }

    return output;
}

template <typename DataType>
typename Region<DataType>::Node* Region<DataType>::MakeCell(numtk::vec3u const& cell)
{
    numtk::vec3u cell_begin = Region<DataType>::CellBegin(cell);
    Node* current_node = FindLeaf(cell_begin);
    while (current_node->depth)
        current_node = InsertChild(current_node, current_node->LocalPoint(cell_begin));
    return current_node;
}

template <typename DataType>
VoxelMask Region<DataType>::GetCell(numtk::vec3u const& cell) const
{
    numtk::vec3u cell_begin = Region<DataType>::CellBegin(cell);
    Node const* leaf = FindLeaf(cell_begin);
    if (!leaf)
        return VoxelMask::kEmptyMask();

    if (!leaf->depth)
        return leaf->data_mask;
    else
        return leaf->child_mask.Test(leaf->LocalPoint(cell_begin))
            ? VoxelMask::kFullMask()
            : VoxelMask::kEmptyMask();
}

template <typename DataType>
void Region<DataType>::SetCell(numtk::vec3u const& cell, VoxelMask const& mask)
{
    Node* node = MakeCell(cell);
    node->data_mask = mask;
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

VoxelMask VoxelMask::SelectOctant(numtk::vec3u const& junction, uint32_t index) const
{
    VoxelMask x_mask = VoxelMask::FillAbove({ junction.x, 0, 0 });
    if (!(index & 1))
        x_mask = x_mask.BitwiseNot();

    VoxelMask y_mask = VoxelMask::FillAbove({ 0, junction.y, 0 });
    if (!((index >> 1) & 1))
        y_mask = y_mask.BitwiseNot();

    VoxelMask z_mask = VoxelMask::FillAbove({ 0, 0, junction.z });
    if (!((index >> 2) & 1))
        z_mask = z_mask.BitwiseNot();

    return BitwiseAnd(x_mask.BitwiseAnd(y_mask).BitwiseAnd(z_mask));
}

VoxelMask VoxelMask::Shift(numtk::vec3i const& shift)
{
    VoxelMask output = *this;

    static constexpr uint64_t kBroadcastU4 = 0x1111'1111'1111'1111ull;
    static constexpr uint64_t kBroadcastU16 = 0x0001'0001'0001'0001ull;

    if (shift.x != 0)
    {
        VoxelMask copy = output;

        uint32_t const bitshift = shift.x % 4;
        uint32_t const blockshift = (shift.x / 4) % 2;

        uint64_t const xdstmask = ((0xfull << bitshift) & 0xfull) * kBroadcastU4;
        uint64_t const xsrcmask = (0xfull >> bitshift) * kBroadcastU4;
        uint64_t const xdstcarry = (0xfull >> (4-bitshift)) * kBroadcastU4;
        uint64_t const xsrccarry = ((0xfull << (4-bitshift)) & 0xfull) * kBroadcastU4;

        output.bits[7] = numtk::BitDeposit(numtk::BitExtract(copy.bits[7 - blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[6 + blockshift], xsrccarry), xdstcarry);
        output.bits[6] = numtk::BitDeposit(numtk::BitExtract(copy.bits[6 + blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[7 - blockshift], xsrccarry), xdstcarry);

        output.bits[5] = numtk::BitDeposit(numtk::BitExtract(copy.bits[5 - blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[4 + blockshift], xsrccarry), xdstcarry);
        output.bits[4] = numtk::BitDeposit(numtk::BitExtract(copy.bits[4 + blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[5 - blockshift], xsrccarry), xdstcarry);

        output.bits[3] = numtk::BitDeposit(numtk::BitExtract(copy.bits[3 - blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[2 + blockshift], xsrccarry), xdstcarry);
        output.bits[2] = numtk::BitDeposit(numtk::BitExtract(copy.bits[2 + blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[3 - blockshift], xsrccarry), xdstcarry);

        output.bits[1] = numtk::BitDeposit(numtk::BitExtract(copy.bits[1 - blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[0 + blockshift], xsrccarry), xdstcarry);
        output.bits[0] = numtk::BitDeposit(numtk::BitExtract(copy.bits[0 + blockshift], xsrcmask), xdstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[1 - blockshift], xsrccarry), xdstcarry);
    }

    if (shift.y != 0)
    {
        VoxelMask copy = output;

        uint32_t const bitshift = (shift.y % 4) * 4;
        uint32_t const blockshift = ((shift.y / 4) % 2) * 2;

        uint64_t const ydstmask = ((0xffffull << bitshift) & 0xffffull) * kBroadcastU16;
        uint64_t const ysrcmask = (0xffffull >> bitshift) * kBroadcastU16;
        uint64_t const ydstcarry = (0xffffull >> (16 - bitshift)) * kBroadcastU16;
        uint64_t const ysrccarry = ((0xffffull << (16 - bitshift)) & 0xffffull) * kBroadcastU16;

        output.bits[7] = numtk::BitDeposit(numtk::BitExtract(copy.bits[7 - blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[5 + blockshift], ysrccarry), ydstcarry);
        output.bits[5] = numtk::BitDeposit(numtk::BitExtract(copy.bits[5 + blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[7 - blockshift], ysrccarry), ydstcarry);

        output.bits[6] = numtk::BitDeposit(numtk::BitExtract(copy.bits[6 - blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[4 + blockshift], ysrccarry), ydstcarry);
        output.bits[4] = numtk::BitDeposit(numtk::BitExtract(copy.bits[4 + blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[6 - blockshift], ysrccarry), ydstcarry);

        output.bits[3] = numtk::BitDeposit(numtk::BitExtract(copy.bits[3 - blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[1 + blockshift], ysrccarry), ydstcarry);
        output.bits[1] = numtk::BitDeposit(numtk::BitExtract(copy.bits[1 + blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[3 - blockshift], ysrccarry), ydstcarry);

        output.bits[2] = numtk::BitDeposit(numtk::BitExtract(copy.bits[2 - blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[0 + blockshift], ysrccarry), ydstcarry);
        output.bits[0] = numtk::BitDeposit(numtk::BitExtract(copy.bits[0 + blockshift], ysrcmask), ydstmask)
            | numtk::BitDeposit(numtk::BitExtract(copy.bits[2 - blockshift], ysrccarry), ydstcarry);
    }

    if (shift.z != 0)
    {
        VoxelMask copy = output;

        uint32_t const bitshift = (shift.z % 4) * 16;
        uint32_t const blockshift = ((shift.z / 4) % 2) * 4;
        uint32_t const bitcarry = (64 - bitshift);

        if (bitcarry != 64)
        {
            output.bits[0] = (copy.bits[0 + blockshift] << bitshift) | (copy.bits[4 - blockshift] >> bitcarry);
            output.bits[4] = (copy.bits[4 - blockshift] << bitshift) | (copy.bits[0 + blockshift] >> bitcarry);

            output.bits[1] = (copy.bits[1 + blockshift] << bitshift) | (copy.bits[5 - blockshift] >> bitcarry);
            output.bits[5] = (copy.bits[5 - blockshift] << bitshift) | (copy.bits[1 + blockshift] >> bitcarry);

            output.bits[2] = (copy.bits[2 + blockshift] << bitshift) | (copy.bits[6 - blockshift] >> bitcarry);
            output.bits[6] = (copy.bits[6 - blockshift] << bitshift) | (copy.bits[2 + blockshift] >> bitcarry);

            output.bits[3] = (copy.bits[3 + blockshift] << bitshift) | (copy.bits[7 - blockshift] >> bitcarry);
            output.bits[7] = (copy.bits[7 - blockshift] << bitshift) | (copy.bits[3 + blockshift] >> bitcarry);
        }
        else
        {
            output.bits[0] = copy.bits[0 + blockshift];
            output.bits[4] = copy.bits[4 - blockshift];
            output.bits[1] = copy.bits[1 + blockshift];
            output.bits[5] = copy.bits[5 - blockshift];
            output.bits[2] = copy.bits[2 + blockshift];
            output.bits[6] = copy.bits[6 - blockshift];
            output.bits[3] = copy.bits[3 + blockshift];
            output.bits[7] = copy.bits[7 - blockshift];
        }
    }

    return output;
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
    VoxelMask mask = VoxelMask::FillArea(begin, end);
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
