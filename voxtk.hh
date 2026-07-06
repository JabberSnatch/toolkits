#pragma once

#include "numtk.hh"
#include "dstk.hh"

#define INLINE_PDEP

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

#ifndef INLINE_PDEP
    static uint16_t BitIndex(numtk::vec3u const& point);
    static numtk::vec3u PointLocation(uint16_t index);

#else

    inline static uint16_t BitIndex(numtk::vec3u const& point)
    {
        // 2 wide blocks of 4 wide blocks
        // 1 bit for the toplevel, 2 bits for the bottomlevel
        return (uint16_t)(
            numtk::BitDeposit(point.x, 0b1000011) |
            numtk::BitDeposit(point.y, 0b10001100) |
            numtk::BitDeposit(point.z, 0b100110000));
    }

    inline static numtk::vec3u PointLocation(uint16_t index)
    {
        return numtk::vec3u{
            numtk::BitExtract((uint32_t)index, 0b1000011),
            numtk::BitExtract((uint32_t)index, 0b100011000),
            numtk::BitExtract((uint32_t)index, 0b100110000)
        };
    }
#endif

    VoxelMask& Clear();
    VoxelMask& BitReverse();

    VoxelMask BitwiseAnd(VoxelMask const& o) const;
    VoxelMask BitwiseOr(VoxelMask const& o) const;
    VoxelMask BitwiseNot() const;

    VoxelMask SelectOctant(numtk::vec3u const& junction, uint32_t index) const;

    VoxelMask Shift(numtk::vec3i shift);

#ifndef INLINE_PDEP
    bool Test(numtk::vec3u const& point) const;

#else

    inline bool Test(numtk::vec3u const& point) const
    {
        uint16_t bit_index = BitIndex(point);
        return Test(bit_index);
    }
#endif

    bool Test(uint16_t bit_index) const {
        uint16_t pack_index = bit_index / 64;
        bit_index = bit_index & 63;
        return !!((bits[pack_index] >> bit_index) & 1);
    }

    uint64_t ExtractKernel(numtk::vec3u const& base) const;

    VoxelMask& Set(uint16_t bit_index, bool v);
    VoxelMask& Set(numtk::vec3u const& point, bool v);
    VoxelMask& Set(numtk::vec3u const& begin, numtk::vec3u const& end, bool v);

    uint16_t NextIndex(uint16_t index = ~(uint16_t)0) const;

    bool Full() const;
    bool Empty() const;

    uint64_t bits[8];
};

#define DENSE_CHILDREN_ARRAY

struct BinaryRegion
{
    static numtk::vec3u CellLocation(numtk::vec3u const& _point) { return _point / VoxelMask::kSize; }
    static numtk::vec3u CellBegin(numtk::vec3u const& _cell_location) { return _cell_location * VoxelMask::kSize; }

    BinaryRegion() : BinaryRegion(numtk::vec3u::Constant(0)) {};
    BinaryRegion(numtk::vec3u _size);
    BinaryRegion(BinaryRegion&&) = default;
    BinaryRegion(BinaryRegion const&);
    BinaryRegion& operator=(BinaryRegion&&) = default;
    BinaryRegion& operator=(BinaryRegion const&);

    bool Contains(numtk::vec3u const& _point) const { return _point.x < size.x && _point.y < size.y && _point.z < size.z; }

    bool Test(numtk::vec3u const& _point) const;

    void Set(numtk::vec3u const& _point, bool _v);
    void Set(numtk::bounds3u const& _bounds, bool _v);
    void Set(numtk::vec3i const& _begin, BinaryRegion const& _v);

    BinaryRegion Shift(numtk::vec3i const& _offset) const;
    BinaryRegion Crop(numtk::bounds3u const& _bounds) const;

    BinaryRegion BitwiseAnd(numtk::vec3i const& _begin, BinaryRegion const& _o) const;
    BinaryRegion BitwiseOr(numtk::vec3i const& _begin, BinaryRegion const& _o) const;

    void Fill(numtk::bounds3u const& _bounds) { Set(_bounds, true); }

    void Clear() { Clear({ numtk::vec3u::Constant(0), size }); }
    void Clear(numtk::bounds3u const& _bounds) { Set(_bounds, false); }

    struct Node {
        static constexpr int32_t kChildCount = VoxelMask::kVolume;

        Node* parent;
        numtk::vec3u location;
        uint32_t depth;

        VoxelMask child_mask{};
        VoxelMask data_mask{};
#ifndef DENSE_CHILDREN_ARRAY
        dstk::FlatMap<uint16_t, Node*> children{};
#else
        std::vector<Node*> children = std::vector<Node*>(VoxelMask::kVolume);
#endif

        numtk::bounds3u Bounds() const {
            return numtk::bounds3u::MinExtent(
                location * (1u << (3*(depth+1))),
                numtk::vec3u::Constant(VoxelMask::kSize)
            );
        }
        numtk::vec3u LocalPoint(numtk::vec3u const& _point) const {
            return (_point >> (3 * depth)) & VoxelMask::kSizeMask;
        }
        uint16_t ChildIndex(numtk::vec3u const& _child) const {
            return VoxelMask::BitIndex(_child);
        }
    };

    Node* MakeCell(numtk::vec3u const& _cell_location);
    VoxelMask GetCellMask(numtk::vec3u const& _cell_location) const;
    void SetCellMask(numtk::vec3u const& _cell_location, VoxelMask const& _mask);

    uint64_t GetEnclosingKernel(numtk::vec3u const& _point) const;

    Node* FindDeepestNode(numtk::vec3u const& _point) const;
    Node* InsertChild(Node* _parent, numtk::vec3u const& _local_point);

    Node* root = nullptr;
    dstk::ObjectPool<Node> node_pool{};
    numtk::vec3u size;
    numtk::vec3u Size() const { return size; }
    uint32_t level_count;

    bool HasLayer(uint8_t _id) const {
        return layers.count(_id);
    }

    template <typename T>
    void DeclareLayer(uint8_t _id, T&& _default_value) {
        using DataType = std::remove_cvref<T>::type;

        static auto const PayloadDtor = [](void* ptr) {
            ((DataType*)ptr)->~DataType();
        };
        static auto const PayloadCopy = [](void* dst, void const* src) {
            *(DataType*)dst = *(DataType const*)src;
        };

        if (layers.count(_id))
            return;

        auto layer_it = layers.emplace(_id, DataLayer{
            sizeof(DataType), {}, PayloadDtor, PayloadCopy
        });
        layer_it.first->second.default_value.reset(new uint8_t[sizeof(DataType)]);
        *(DataType*)layer_it.first->second.default_value.get() = _default_value;
    }

    template <typename T>
    void StoreData(numtk::vec3u const& _point, uint8_t _id, T&& _data) {
        using DataType = std::remove_cvref<T>::type;
        Set(_point, true);

        if (!layers.count(_id))
            return;
        DataLayer const& layer = layers.at(_id);
        if (layer.payload_size != sizeof(DataType))
            return;

        Node const* node = FindDeepestNode(_point);
        uint64_t node_data_id = NodeDataLayerID(node, _id);

        if (!node_data.count(node_data_id))
            node_data.emplace(node_data_id,
                              std::unique_ptr<uint8_t[]>{
                                  new uint8_t[layer.payload_size * Node::kChildCount]
                              }
            );

        DataType* data_store = (DataType*)node_data.at(node_data_id).get();
        data_store[node->ChildIndex(node->LocalPoint(_point))] = _data;
    }

    template <typename T>
    std::remove_cvref<T>::type const& LoadData(numtk::vec3u const& _point, uint8_t _id) const {
        using DataType = std::remove_cvref<T>::type;
        static DataType const kDefaultValue = DataType{};

        if (!layers.count(_id))
            return kDefaultValue;
        DataLayer const& layer = layers.at(_id);
        if (layer.payload_size != sizeof(DataType))
            return kDefaultValue;

        if (!Test(_point))
            return *(DataType const*)layer.default_value.get();

        Node const* node = FindDeepestNode(_point);
        uint64_t node_data_id = NodeDataLayerID(node, _id);

        if (!node_data.count(node_data_id))
            return *(DataType const*)layer.default_value.get();

        DataType const* data_store = (DataType const*)node_data.at(node_data_id).get();
        return data_store[node->ChildIndex(node->LocalPoint(_point))];
    }

    static uint64_t NodeDataLayerID(Node const* _node, uint8_t _id) {
        assert(!((uint64_t)_node & ~0x0000ffffffffffffull));
        return ((uint64_t)_node & 0x0000ffffffffffffull) | ((uint64_t)_id << 56);
    }

    struct DataLayer {
        size_t payload_size;
        std::unique_ptr<uint8_t[]> default_value;
        void(*dtor)(void*);
        void(*copy)(void*, void const*);
    };
    std::unordered_map<uint8_t, DataLayer> layers{};
    std::unordered_map<uint64_t, std::unique_ptr<uint8_t[]>> node_data{};
};

struct VoxelField
{
    VoxelField() {}

    void Set(numtk::vec3i const& _point, bool _v);
    void SetVolume(numtk::bounds3i const& _bounds, bool _v);
    bool Test(numtk::vec3i const& _point) const;
    uint64_t TestKernel(numtk::vec3i const& _point) const;

    struct Node {
        static constexpr uint32_t kChildCount = VoxelMask::kVolume;
        static uint32_t DepthSize(uint32_t _depth) {
            return 1u << (VoxelMask::kLogSize * (_depth+1));
        }

        static numtk::vec3i DepthMap(numtk::vec3i const& _target, uint32_t _depth) {
            return _target >> (VoxelMask::kLogSize * _depth);
        }

        static numtk::vec3i DepthInvMap(numtk::vec3i const& _target, uint32_t _depth) {
            return _target << (VoxelMask::kLogSize * _depth);
        }

        static numtk::vec3i AlignedBase(numtk::vec3i const& _target, uint32_t _depth) {
            return _target & ~(DepthSize(_depth) - 1);
        }

        static numtk::bounds3i AlignedBounds(numtk::vec3i const& _target, uint32_t _depth) {
            return numtk::bounds3i::MinExtent(
                AlignedBase(_target, _depth),
                numtk::vec3i::Constant(DepthSize(_depth))
            );
        }

        static numtk::bounds3i AlignedBounds(numtk::bounds3i const& _target, uint32_t _depth) {
            return numtk::bounds3i{
                AlignedBase(_target.min, _depth),
                AlignedBase(_target.max + numtk::vec3i::Constant(DepthSize(_depth)-1), _depth)
            };
        }

        Node(Node* _parent, numtk::vec3i const& _base, uint32_t _depth = 0, numtk::vec3u const& _offset = {})
            : depth{ _depth }
            , parent{ _parent }
        {
            point = DepthMap(_base, depth+1);
            offset = (_base - Base()).cast<uint32_t>();

            if (_parent)
                _parent->BindChild(this);
        }

        numtk::vec3i Base() const {
            return point * (1u << (VoxelMask::kLogSize * (depth+1)));
        }

        numtk::bounds3i Bounds() const {
            return numtk::bounds3i::MinExtent(
                Base() + offset.cast<int32_t>(),
                numtk::vec3i::Constant(1u << (VoxelMask::kLogSize * (depth+1)))
            );
        }

        numtk::vec3u LocalPoint(numtk::vec3i const& _point) const {
            return (DepthMap(_point - offset.cast<int32_t>(), depth)
                    & VoxelMask::kSizeMask)
                .cast<uint32_t>();
        }

        uint16_t ChildIndex(numtk::vec3u const& _child) const{
            return VoxelMask::BitIndex(_child);
        }

        bool Contains(numtk::vec3i const& _point) const {
            numtk::vec3i local_point =
                DepthMap(_point - offset.cast<int32_t>(), depth)
                - (point << VoxelMask::kLogSize);
            return local_point.x >= 0 && local_point.x < VoxelMask::kSize
                && local_point.y >= 0 && local_point.y < VoxelMask::kSize
                && local_point.z >= 0 && local_point.z < VoxelMask::kSize;
        }

        void BindChild(Node* _child) {
            assert(_child);
            assert(depth);
            assert(Contains(_child->Base()));
            assert(_child->depth == depth-1);

            uint16_t child_index = ChildIndex(LocalPoint(_child->Base()));
            children[child_index] = _child;
            child_mask.Set(child_index, true);
            _child->parent = this;
        }

        uint32_t depth;
        numtk::vec3i point;
        numtk::vec3u offset;

        VoxelMask child_mask{};
        VoxelMask data_mask{};

        Node const* parent;
        std::vector<Node*> children = std::vector<Node*>(kChildCount);
    };

    Node* root = nullptr;
    dstk::ObjectPool<Node> node_pool{};
    numtk::bounds3i bounds;

    Node* EmplaceLeaf(numtk::vec3i const& _global_point);
    Node* LookupNode(numtk::vec3i const& _global_point) const;
    template <typename T> void RelocateRoot(T const& _global_bounds);

    // =========================================================================
    // DATA LAYERS
    // =========================================================================
    bool HasLayer(uint8_t _id) const {
        return layers.count(_id);
    }

    template <typename T>
    void DeclareLayer(uint8_t _id, T&& _default_value) {
        using DataType = std::remove_cvref<T>::type;

        static auto const PayloadDtor = [](void* ptr) {
            ((DataType*)ptr)->~DataType();
        };
        static auto const PayloadCopy = [](void* dst, void const* src) {
            *(DataType*)dst = *(DataType const*)src;
        };

        if (layers.count(_id))
            return;

        auto layer_it = layers.emplace(_id, DataLayer{
            sizeof(DataType), {}, PayloadDtor, PayloadCopy
        });
        layer_it.first->second.default_value.reset(new uint8_t[sizeof(DataType)]);
        *(DataType*)layer_it.first->second.default_value.get() = _default_value;
    }

    template <typename T>
    void StoreData(numtk::vec3i const& _point, uint8_t _id, T&& _data) {
        using DataType = std::remove_cvref<T>::type;
        Set(_point, true);

        if (!layers.count(_id))
            return;
        DataLayer const& layer = layers.at(_id);
        if (layer.payload_size != sizeof(DataType))
            return;

        Node const* node = LookupNode(_point);
        uint64_t node_data_id = NodeDataLayerID(node, _id);

        if (!node_data.count(node_data_id))
            node_data.emplace(node_data_id,
                              std::unique_ptr<uint8_t[]>{
                                  new uint8_t[layer.payload_size * Node::kChildCount]
                              }
            );

        DataType* data_store = (DataType*)node_data.at(node_data_id).get();
        data_store[node->ChildIndex(node->LocalPoint(_point))] = _data;
    }

    template <typename T>
    std::remove_cvref<T>::type const& LoadData(numtk::vec3i const& _point, uint8_t _id) const {
        using DataType = std::remove_cvref<T>::type;
        static DataType const kDefaultValue = DataType{};

        if (!layers.count(_id))
            return kDefaultValue;
        DataLayer const& layer = layers.at(_id);
        if (layer.payload_size != sizeof(DataType))
            return kDefaultValue;

        if (!Test(_point))
            return *(DataType const*)layer.default_value.get();

        Node const* node = LookupNode(_point);
        uint64_t node_data_id = NodeDataLayerID(node, _id);

        if (!node_data.count(node_data_id))
            return *(DataType const*)layer.default_value.get();

        DataType const* data_store = (DataType const*)node_data.at(node_data_id).get();
        return data_store[node->ChildIndex(node->LocalPoint(_point))];
    }

    static uint64_t NodeDataLayerID(Node const* _node, uint8_t _id) {
        assert(!((uint64_t)_node & ~0x0000ffffffffffffull));
        return ((uint64_t)_node & 0x0000ffffffffffffull) | ((uint64_t)_id << 56);
    }

    struct DataLayer {
        size_t payload_size;
        std::unique_ptr<uint8_t[]> default_value;
        void(*dtor)(void*);
        void(*copy)(void*, void const*);
    };
    std::unordered_map<uint8_t, DataLayer> layers{};
    std::unordered_map<uint64_t, std::unique_ptr<uint8_t[]>> node_data{};
};

struct VoxelFieldCompat : public VoxelField
{
    VoxelFieldCompat() = default;
    VoxelFieldCompat(VoxelFieldCompat&&) = default;
    VoxelFieldCompat(VoxelFieldCompat const&) = delete;
    VoxelFieldCompat& operator=(VoxelFieldCompat&&) = default;
    VoxelFieldCompat& operator=(VoxelFieldCompat const&) = delete;

    VoxelFieldCompat(numtk::vec3u const& _size) {
        uint32_t max_size = std::max(std::max(_size.x, _size.y), _size.z);
        uint32_t log_size = numtk::ilogN<VoxelMask::kSize>(max_size);
        if (1u << (VoxelMask::kLogSize * log_size) < max_size)
            ++log_size;

        root = node_pool[node_pool.Emplace(
                nullptr,
                numtk::vec3i{ 0, 0, 0 },
                std::max(log_size, 1u))];
        bounds = root->Bounds();
        RelocateRoot(numtk::bounds3i{ { 0, 0, 0 }, _size.cast<int32_t>() });
        bounds = root->Bounds();
    }
    void Set(numtk::vec3u const& _point, bool _v) {
        VoxelField::Set(_point.cast<int32_t>(), _v);
    }
    void Set(numtk::bounds3u const& _bounds, bool _v) {
        VoxelField::SetVolume({ _bounds.min.cast<int32_t>(),
            _bounds.max.cast<int32_t>() }, _v);
    }

    bool Test(numtk::vec3u const& _point) const {
        return VoxelField::Test(_point.cast<int32_t>()); }
    uint64_t GetEnclosingKernel(numtk::vec3u const& _point) const {
        return VoxelField::TestKernel(_point.cast<int32_t>()); }

    template <typename T>
    void StoreData(numtk::vec3u const& _point, uint8_t _id, T&& _data) {
        VoxelField::StoreData(_point.cast<int32_t>(), _id, _data);
    }
    template <typename T>
    std::remove_cvref<T>::type const& LoadData(numtk::vec3u const& _point, uint8_t _id) const {
        return VoxelField::LoadData<T>(_point.cast<int32_t>(), _id); }

    numtk::vec3u Size() const { return bounds.extent().cast<uint32_t>(); }
};

} // namespace voxtk

#ifdef VOXTK_IMPLEMENTATION

namespace voxtk
{

void
VoxelField::Set(numtk::vec3i const& _point, bool _v)
{
    Node* current_node = LookupNode(_point);

    if (!current_node ||
        (current_node->depth
         && current_node->data_mask.Test(current_node->LocalPoint(_point)) != _v))
        current_node = EmplaceLeaf(_point);

    if(current_node->depth == 0)
        current_node->data_mask.Set(current_node->LocalPoint(_point), _v);
}

void
VoxelField::SetVolume(numtk::bounds3i const& _bounds, bool _v)
{
    if (!root)
    {
        root = node_pool[node_pool.Emplace(
                nullptr,
                Node::AlignedBase(_bounds.min, 0),
                0u)];
        bounds = root->Bounds();
    }

    if (!root->Contains(_bounds.min) || !root->Contains(_bounds.max - numtk::vec3i::Constant(1)))
        RelocateRoot(_bounds);

    std::vector<Node*> node_queue{ root };
    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        numtk::bounds3i const node_bounds = current_node->Bounds();

        if (!current_node->depth)
        {
            numtk::vec3u const data_begin =
                (numtk::max(_bounds.min, node_bounds.min) - node_bounds.min).cast<uint32_t>();
            numtk::vec3u const data_end =
                (numtk::min(_bounds.max, node_bounds.max) - node_bounds.min).cast<uint32_t>();
            current_node->data_mask.Set(data_begin, data_end, _v);
        }

        else
        {
            numtk::vec3i bounds_min = Node::DepthMap(_bounds.min, current_node->depth);
            numtk::vec3i node_min = Node::DepthMap(node_bounds.min, current_node->depth);
            numtk::vec3i bounds_max =
                Node::DepthMap(_bounds.max
                               + numtk::vec3i::Constant(Node::DepthSize(current_node->depth-1) - 1)
                               , current_node->depth);
            numtk::vec3i node_max = Node::DepthMap(node_bounds.max, current_node->depth);

            numtk::vec3u const children_begin =
                (numtk::max(bounds_min, node_min) - node_min)
                .cast<uint32_t>();
            numtk::vec3u const children_end =
                (numtk::min(bounds_max, node_max) - node_min)
                .cast<uint32_t>();

            for (uint32_t z = children_begin.z; z < children_end.z; ++z)
                for (uint32_t y = children_begin.y; y < children_end.y; ++y)
                    for (uint32_t x = children_begin.x; x < children_end.x; ++x)
                    {
                        numtk::vec3u const child{ x, y, z };
                        uint16_t child_index = current_node->ChildIndex(child);

                        if (!current_node->child_mask.Test(child))
                        {
                            numtk::vec3i const child_begin = Node::DepthInvMap(
                                child.cast<int32_t>() + node_min,
                                current_node->depth
                            );
                            current_node->children[child_index] =
                                node_pool[node_pool.Emplace(
                                    current_node,
                                    child_begin,
                                    current_node->depth-1)];
                        }

                        node_queue.push_back(current_node->children[child_index]);
                    }
        }
    }
}

bool
VoxelField::Test(numtk::vec3i const& _point) const
{
    Node* current_node = LookupNode(_point);
    return current_node && current_node->data_mask.Test(current_node->LocalPoint(_point));
}

uint64_t
VoxelField::TestKernel(numtk::vec3i const& _point) const
{
    Node const* node = LookupNode(_point);
    if (!node) return 0;
    numtk::vec3u local_point = node->LocalPoint(_point);
    if (!node->depth)
        return node->data_mask.ExtractKernel(local_point);
    else
        return node->child_mask.Test(local_point)
            ? ~(uint64_t)0
            : 0;
}

VoxelField::Node*
VoxelField::EmplaceLeaf(numtk::vec3i const& _global_point)
{
    if (!root)
    {
        root = node_pool[node_pool.Emplace(
                nullptr,
                Node::AlignedBase(_global_point, 0),
                0u)];
        bounds = root->Bounds();
        return root;
    }

    Node* current_node = LookupNode(_global_point);
    assert(!current_node || current_node->depth);

    if (!current_node)
    {
        RelocateRoot(_global_point);
        current_node = root;
        assert(LookupNode(_global_point) == root);
        assert(current_node && current_node->depth);
    }

    while (current_node->depth)
    {
        uint32_t depth = current_node->depth-1;
        current_node = node_pool[node_pool.Emplace(
                current_node,
                Node::AlignedBase(_global_point, depth),
                depth)];
    }

    bounds = bounds.Expand(current_node->Bounds());
    return current_node;
}

VoxelField::Node*
VoxelField::LookupNode(numtk::vec3i const& _global_point) const
{
    if (!root
        || !root->Contains(_global_point))
        return nullptr;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(_global_point);
        if (!current_node->child_mask.Test(local_point))
            break;
        else {
            uint16_t point_index = current_node->ChildIndex(local_point);
            current_node = current_node->children[point_index];
        }
    }

    return current_node;
}

template <typename T> void
VoxelField::RelocateRoot(T const& _global_bounds)
{
    numtk::bounds3i aligned_bounds = Node::AlignedBounds(_global_bounds, root->depth-1);
    numtk::bounds3i expanded_bounds = bounds.Expand(aligned_bounds);
    numtk::vec3i extent = expanded_bounds.extent();
    uint32_t node_size = Node::DepthSize(root->depth);

    // Expanding up from root until it can accomodate our new leaf
    // Preserves overall root offset, up to depth alignment
    while (!(extent.x <= node_size
             && extent.y <= node_size
             && extent.z <= node_size))
    {
        if (root->offset == numtk::vec3u::Constant(0))
        {
            // Aligned case is only a matter of inserting root into its new parent
            Node* next = node_pool[node_pool.Emplace(
                    nullptr,
                    Node::AlignedBase(root->Base(), root->depth+1),
                    root->depth+1)];
            next->BindChild(root);
            root = next;
        }
        else
        {
            // Unaligned case requires us to rebind all of the old root's children
            // They are expected to be aligned so we don't have to go through the entire
            // hierarchy.

            Node tmp = std::move(*root);
            *root = Node{
                nullptr,
                Node::AlignedBase(root->Base(), root->depth),
                root->depth+1
            };

            for (Node* child : tmp.children)
            {
                if (!child) continue;
                assert(child->offset == numtk::vec3u::Constant(0));

                Node* parent = root;
                numtk::vec3i child_base = child->Base();
                while (parent->depth > child->depth+1)
                {
                    uint32_t depth = parent->depth-1;
                    numtk::vec3u local_point = parent->LocalPoint(child_base);
                    uint16_t child_index = parent->ChildIndex(local_point);
                    if (!parent->child_mask.Test(child_index))
                    {
                        Node* next = node_pool[node_pool.Emplace(
                                parent,
                                Node::AlignedBase(child_base, depth),
                                depth)];
                        parent = next;
                    }
                    else
                        parent = parent->children[child_index];
                }

                parent->BindChild(child);
            }
        }

        node_size = Node::DepthSize(root->depth);
    }

    // Refreshing context in case we cycled through the previous loop
    aligned_bounds = Node::AlignedBounds(_global_bounds, root->depth-1);
    expanded_bounds = Node::AlignedBounds(bounds.Expand(aligned_bounds), root->depth-1);
    extent = expanded_bounds.extent();

    assert(extent.x <= node_size
           && extent.y <= node_size
           && extent.z <= node_size);

    // Root can take new node in, only needs to be shifted
    {
        assert(root->depth);
        Node tmp = std::move(*root);
        *root = Node{ nullptr, expanded_bounds.min, root->depth };

        for (Node* child : tmp.children)
            if (child)
                root->BindChild(child);
    }
}


} // namespace voxtk

namespace voxtk
{

BinaryRegion::BinaryRegion(numtk::vec3u _size)
    : level_count{ 0u }
    , size{ _size}
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

BinaryRegion::BinaryRegion(BinaryRegion const& _other)
    : BinaryRegion{ _other.size }
{
    *root = *_other.root;
    std::vector<Node*> node_queue = { root };

    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        uint16_t child_index = current_node->child_mask.NextIndex();
        while (child_index < Node::kChildCount)
        {
            Node* new_node = node_pool[node_pool.Reserve()];
            *new_node = *(current_node->children[child_index]);
            current_node->children[child_index] = new_node;
            node_queue.push_back(new_node);
            child_index = current_node->child_mask.NextIndex(child_index);
        }
    }
}

BinaryRegion& BinaryRegion::operator=(BinaryRegion const& _other)
{
    *this = BinaryRegion{ _other.size };
    *root = *_other.root;
    std::vector<Node*> node_queue = { root };

    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        uint16_t child_index = current_node->child_mask.NextIndex();
        while (child_index < Node::kChildCount)
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

bool
BinaryRegion::Test(numtk::vec3u const& _point) const
{
    Node const* node = FindDeepestNode(_point);
    if (!node)
        return false;
    else if (!node->depth)
        return node->data_mask.Test(node->LocalPoint(_point));
    else
        return node->child_mask.Test(node->LocalPoint(_point));
}

void
BinaryRegion::Set(numtk::vec3u const& _point, bool _v)
{
    Node* node = FindDeepestNode(_point);
    if (!node)
        return;

    if ((node->depth && (node->data_mask.Test(node->LocalPoint(_point))) != _v))
        node = MakeCell(BinaryRegion::CellLocation(_point));

    if (!node->depth)
        node->data_mask.Set(node->LocalPoint(_point), _v);
}

void
BinaryRegion::Set(numtk::bounds3u const& _bounds, bool _v)
{
    numtk::vec3u const set_end = _bounds.max;

    std::vector<Node*> node_queue{ root };
    while (!node_queue.empty())
    {
        Node* current_node = node_queue.back();
        node_queue.pop_back();

        numtk::bounds3u const node_bounds = current_node->Bounds();

        if (!current_node->depth)
        {
            numtk::vec3u const data_begin =
                numtk::max(_bounds.min, node_bounds.min) - node_bounds.min;
            numtk::vec3u const data_end =
                numtk::min(set_end, node_bounds.max) - node_bounds.min;

            current_node->data_mask.Set(data_begin, data_end, _v);
        }
        else
        {
            numtk::vec3u const children_begin =
                numtk::max(_bounds.min >> (3*(current_node->depth)),
                           node_bounds.min);
            numtk::vec3u const children_end =
                numtk::min((set_end >> (3*(current_node->depth)))
                           + numtk::vec3u::Constant(1u),
                           node_bounds.max);

            for (uint32_t z = children_begin.z; z < children_end.z; ++z)
                for (uint32_t y = children_begin.y; y < children_end.y; ++y)
                    for (uint32_t x = children_begin.x; x < children_end.x; ++x)
                    {
                        numtk::vec3u const child{ x, y, z };
                        uint16_t child_index = current_node->ChildIndex(child);

                        if (!current_node->child_mask.Test(child))
                            InsertChild(current_node, child);

                        node_queue.push_back(current_node->children[child_index]);
                    }
        }
    }
}

BinaryRegion
BinaryRegion::Shift(numtk::vec3i const& _offset) const
{
    if (-_offset.x > (int32_t)size.x
        || -_offset.y > (int32_t)size.y
        || -_offset.z > (int32_t)size.z)
        return {};

    BinaryRegion output{ (size.cast<int32_t>() + _offset).cast<uint32_t>() };

    numtk::vec3i const bit_offset = _offset & VoxelMask::kSizeMask;
    numtk::vec3i const cell_offset = _offset >> VoxelMask::kLogSize;

    numtk::vec3u const cell_extent = BinaryRegion::CellLocation(size + numtk::vec3u::Constant(VoxelMask::kSize-1));
    numtk::vec3u const output_max_cell = BinaryRegion::CellLocation(output.size + numtk::vec3u::Constant(VoxelMask::kSize-1));

    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                numtk::vec3u const dst_cell_base = cell_index + cell_offset.cast<uint32_t>();
                VoxelMask src_mask = GetCellMask(cell_index).Shift(bit_offset);
                for (uint32_t mask_index = 0; mask_index < 8; ++mask_index)
                {
                    numtk::vec3u const mask_offset{ mask_index & 1, (mask_index >> 1) & 1, mask_index >> 2 };
                    numtk::vec3u const dst_cell = dst_cell_base + mask_offset;
                    if (dst_cell.x >= output_max_cell.x
                        || dst_cell.y >= output_max_cell.y
                        || dst_cell.z >= output_max_cell.z
                        || dst_cell.x < 0
                        || dst_cell.y < 0
                        || dst_cell.z < 0)
                        continue;

                    VoxelMask dst_mask = output.GetCellMask(dst_cell);
                    dst_mask = dst_mask.BitwiseOr(src_mask.SelectOctant(bit_offset.cast<uint32_t>(), (~mask_index) & 0x7));
                    output.SetCellMask(dst_cell, dst_mask);
                }
            }

    return output;
}

BinaryRegion
BinaryRegion::Crop(numtk::bounds3u const& _bounds) const
{
    if (!Contains(_bounds.min))
        return {};

    BinaryRegion shifted = Shift(-(_bounds.min.cast<int32_t>()));

    BinaryRegion output{ _bounds.extent() };
    numtk::vec3u const cell_extent = BinaryRegion::CellLocation(output.size + numtk::vec3u::Constant(VoxelMask::kSize-1));
    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                output.SetCellMask(cell_index, shifted.GetCellMask(cell_index));
            }

    return output;
}

BinaryRegion
BinaryRegion::BitwiseAnd(numtk::vec3i const& _begin, BinaryRegion const& _o) const
{
    BinaryRegion output{ size };
    BinaryRegion rhs = _o.Shift(_begin);

    numtk::vec3u const cell_extent = numtk::min(
        BinaryRegion::CellLocation(size + numtk::vec3u::Constant(VoxelMask::kSize-1)),
        BinaryRegion::CellLocation(rhs.size + numtk::vec3u::Constant(VoxelMask::kSize-1)));

    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                VoxelMask output_mask = GetCellMask(cell_index).BitwiseAnd(rhs.GetCellMask(cell_index));
                output.SetCellMask(cell_index, output_mask);
            }

    return output;
}

BinaryRegion
BinaryRegion::BitwiseOr(numtk::vec3i const& _begin, BinaryRegion const& _o) const
{
    BinaryRegion output{ size };
    BinaryRegion rhs = _o.Shift(_begin);

    numtk::vec3u const cell_extent = numtk::min(
        BinaryRegion::CellLocation(size + numtk::vec3u::Constant(VoxelMask::kSize-1)),
        BinaryRegion::CellLocation(rhs.size + numtk::vec3u::Constant(VoxelMask::kSize-1)));

    for (uint32_t cell_z = 0u; cell_z < cell_extent.z; ++cell_z)
        for (uint32_t cell_y = 0u; cell_y < cell_extent.y; ++cell_y)
            for (uint32_t cell_x = 0u; cell_x < cell_extent.x; ++cell_x)
            {
                numtk::vec3u const cell_index{ cell_x, cell_y, cell_z };
                VoxelMask output_mask = GetCellMask(cell_index).BitwiseOr(rhs.GetCellMask(cell_index));
                output.SetCellMask(cell_index, output_mask);
            }

    return output;
}

BinaryRegion::Node*
BinaryRegion::MakeCell(numtk::vec3u const& _cell_location)
{
    numtk::vec3u cell_begin = BinaryRegion::CellBegin(_cell_location);
    Node* current_node = FindDeepestNode(cell_begin);
    while (current_node->depth)
        current_node = InsertChild(current_node, current_node->LocalPoint(cell_begin));
    return current_node;
}

VoxelMask
BinaryRegion::GetCellMask(numtk::vec3u const& _cell_location) const
{
    numtk::vec3u cell_begin = BinaryRegion::CellBegin(_cell_location);
    Node const* node = FindDeepestNode(cell_begin);
    if (!node)
        return VoxelMask::kEmptyMask();

    if (!node->depth)
        return node->data_mask;
    else
        return node->child_mask.Test(node->LocalPoint(cell_begin))
            ? VoxelMask::kFullMask()
            : VoxelMask::kEmptyMask();
}

void
BinaryRegion::SetCellMask(numtk::vec3u const& _cell_location, VoxelMask const& _mask)
{
    Node* node = MakeCell(_cell_location);
    node->data_mask = _mask;
}

uint64_t
BinaryRegion::GetEnclosingKernel(numtk::vec3u const& _point) const
{
    Node const* node = FindDeepestNode(_point);
    if (!node) return 0;

    numtk::vec3u local_point = node->LocalPoint(_point);
    if (!node->depth)
        return node->data_mask.ExtractKernel(local_point);
    else
        return node->child_mask.Test(local_point)
            ? ~(uint64_t)0
            : 0;
}

BinaryRegion::Node*
BinaryRegion::FindDeepestNode(numtk::vec3u const& _point) const
{
    if (!Contains(_point)) return nullptr;

    Node* current_node = root;
    while (current_node)
    {
        numtk::vec3u local_point = current_node->LocalPoint(_point);
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

BinaryRegion::Node*
BinaryRegion::InsertChild(Node* _parent, numtk::vec3u const& _local_point)
{
    uint16_t point_index = _parent->ChildIndex(_local_point);

#ifndef DENSE_CHILDREN_ARRAY
    auto child_node =
        _parent->children.insert(point_index, node_pool[node_pool.Reserve()]);

    (*child_node)->parent = _parent;
    (*child_node)->location = _local_point;
    (*child_node)->depth = _parent->depth - 1;
    _parent->child_mask.Set(_local_point, true);

    return *child_node;
#else
    Node* child_node = node_pool[node_pool.Reserve()];
    _parent->children[point_index] = child_node;

    child_node->parent = _parent;
    child_node->location = _local_point;
    child_node->depth = _parent->depth - 1;
    _parent->child_mask.Set(_local_point, true);

    return child_node;
#endif
}

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

#ifndef INLINE_PDEP
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
#endif

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

VoxelMask VoxelMask::Shift(numtk::vec3i shift)
{
    VoxelMask output = *this;

    static constexpr uint64_t kBroadcastU4 = 0x1111'1111'1111'1111ull;
    static constexpr uint64_t kBroadcastU16 = 0x0001'0001'0001'0001ull;

    shift &= (int32_t)VoxelMask::kSizeMask;

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

#ifndef INLINE_PDEP
bool VoxelMask::Test(numtk::vec3u const& point) const
{
    uint16_t bit_index = BitIndex(point);
    uint16_t pack_index = bit_index / 64;
    bit_index = bit_index & 63;
    return !!((bits[pack_index] >> bit_index) & 1);
}
#endif

uint64_t VoxelMask::ExtractKernel(numtk::vec3u const& base) const
{
    uint16_t pack_index = BitIndex(base) / 64;
    return bits[pack_index];
}

VoxelMask& VoxelMask::Set(uint16_t bit_index, bool v)
{
    uint16_t pack_index = bit_index / 64;
    bit_index = bit_index & 63;

    if (v)
        bits[pack_index] |= (1ull << bit_index);
    else
        bits[pack_index] &= ~(1ull << bit_index);

    return *this;
}

VoxelMask& VoxelMask::Set(numtk::vec3u const& point, bool v)
{
    uint16_t bit_index = BitIndex(point);
    return Set(bit_index, v);
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
        return VoxelMask::kVolume;

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
