#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace dstk
{

struct FreeList
{
    using Index = uint64_t;
    struct Range { Index begin, end; };

    FreeList() : indices{ Range{ 0, UINT64_MAX } } {}
    FreeList(FreeList const&) = default;
    FreeList(FreeList&&) = default;
    FreeList& operator=(FreeList const&) = default;
    FreeList& operator=(FreeList&&) = default;

    Index Reserve() {
        Index result = indices.front().begin;
        ++indices.front().begin;
        if (indices.front().begin == indices.front().end)
            indices.erase(indices.begin());
        return result;
    }

    void Release(Index handle) {
        auto range_iterator = std::upper_bound(
            indices.begin(), indices.end(), handle,
            [](Index handle, Range const& range) { return handle < range.begin; });

        bool location_found = false;
        bool has_prev = (range_iterator != indices.begin());
        bool has_value = (range_iterator != indices.end());

        if (has_prev && handle < std::prev(range_iterator)->end)
            return;

        if (has_prev && handle == std::prev(range_iterator)->end)
        {
            std::prev(range_iterator)->end++;
            location_found = true;
        }
        else if (has_value && handle == range_iterator->begin - 1)
        {
            range_iterator->begin--;
            location_found = true;
        }

        if (!location_found)
            range_iterator = std::next(indices.insert(range_iterator, { handle, handle+1 }));

        if (has_prev && has_value && std::prev(range_iterator)->end == range_iterator->begin)
        {
            std::prev(range_iterator)->end = range_iterator->end;
            indices.erase(range_iterator);
        }
    }

    uint64_t RequiredSize() const
    {
        return indices.back().begin;
    }

    std::vector<Range> indices;
};

struct BlockVector
{
    BlockVector(uint64_t _object_size, uint64_t _block_size)
        : object_size{ _object_size }
        , block_size{ _block_size }
    {}

#if 0
    BlockVector(BlockVector const& o)
        : object_size{ o.object_size }
        , block_size{ o.block_size }
        , blocks{}
    {
        blocks.reserve(o.blocks.size());
        for (std::unique_ptr<uint8_t[]> const& block : o.blocks)
        {
            blocks.emplace_back(new uint8_t[object_size * block_size]);
            uint8_t* dst = blocks.back().get();
            uint8_t const* src = block.get();
            std::copy(src, src+object_size*block_size, dst);
        }
    }

    BlockVector const& operator=(BlockVector const& o)
    {
        object_size = o.object_size;
        block_size = o.block_size;

        blocks.clear();
        blocks.reserve(o.blocks.size());
        for (std::unique_ptr<uint8_t[]> const& block : o.blocks)
        {
            blocks.emplace_back(new uint8_t[object_size * block_size]);
            uint8_t* dst = blocks.back().get();
            uint8_t const* src = block.get();
            std::copy(src, src+object_size*block_size, dst);
        }

        return *this;
    }
#endif

    void Expand(uint64_t object_count) {
        if (object_count <= Capacity())
            return;

        uint64_t delta = object_count - Capacity();
        uint64_t block_count = (delta + object_size-1) / object_size;
        for (uint64_t index = 0; index < block_count; ++index)
            blocks.emplace_back(new uint8_t[object_size * block_size]);
    }

    void* operator[](uint64_t index) const {
        if (index >= Capacity())
            return nullptr;

        uint64_t block_index = index / block_size;
        uint64_t object_index = index - (block_index * block_size);
        return (void*)(blocks[block_index].get() + object_index * object_size);
    }

    uint64_t Find(void const* mem) const {
        auto block_it = std::find_if(
            blocks.begin(), blocks.end(),
            [this, mem](auto const& block){
                std::ptrdiff_t offset = (uint8_t const*)mem - block.get();
                return offset >= 0 && offset < (std::ptrdiff_t)(object_size * block_size);
            });

        if (block_it == blocks.end())
            return ~0ull;

        return (uint64_t)((uint8_t const*)mem - block_it->get());
    }

    uint64_t Capacity() const { return block_size * blocks.size(); }

    uint64_t object_size; // bytes
    uint64_t block_size; // object count
    std::vector<std::unique_ptr<uint8_t[]>> blocks{};
};

template <typename T>
struct ObjectPool
{
    using Handle = uint64_t;
    static constexpr Handle kNullHandle = Handle(0);

    void Expand(uint64_t required_size)
    {
        items.Expand(required_size);
    }

    Handle Reserve() {
        uint64_t index = free_list.Reserve();
        items.Expand(free_list.RequiredSize());
        new (items[index]) T{};
        return index+1;
    }

    T* operator[](Handle handle) const {
        return (T*)items[handle-1];
    }

    void Release(Handle handle) {
        ((T*)items[handle])->~T();
        free_list.Release(handle-1);
    }

    Handle Find(T const* item) const {
        uint64_t index = items.Find(item);
        return index + 1;
    }

    BlockVector items{ sizeof(T), 256ull };
    FreeList free_list{};
};

template <typename Key, typename Value>
struct FlatMap
{
    FlatMap() = default;
    FlatMap(FlatMap const&) = default;
    FlatMap(FlatMap&&) = default;
    FlatMap& operator=(FlatMap const&) = default;
    FlatMap& operator=(FlatMap&&) = default;

    std::vector<Value>::iterator insert(Key const& k, Value const& v)
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        auto item_position = std::next(items.begin(), std::distance(keys.begin(), key_position));
        if (key_position != keys.end() && *key_position == k)
        {
            *item_position = v;
            return item_position;
        }
        else
        {
            keys.insert(key_position, k);
            return items.insert(item_position, v);
        }
    }

    void erase(Key const& k)
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        auto item_position = std::next(items.begin(), std::distance(keys.begin(), key_position));
        if (key_position != keys.end() && *key_position == k)
        {
            keys.erase(key_position);
            items.erase(item_position);
        }
    }

    bool contains(Key const& k) const {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        return key_position != keys.end() && *key_position == k;
    }

    Value const& operator[](Key const& k) const
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        if (key_position != keys.end() && *key_position == k)
            return *std::next(items.begin(), std::distance(keys.begin(), key_position));
        else
            throw std::out_of_range{"Invalid key"};
    }

    Value& operator[](Key const& k)
    {
        return const_cast<Value&>(
            const_cast<FlatMap<Key, Value> const*>(this)->operator[](k)
        );
    }

    std::vector<Value> items{};
    std::vector<Key> keys{};
};

template <typename Key, typename Value>
struct FlatMultimap
{
    FlatMultimap() = default;
    FlatMultimap(FlatMultimap const&) = default;
    FlatMultimap(FlatMultimap&&) = default;
    FlatMultimap& operator=(FlatMultimap const&) = default;
    FlatMultimap& operator=(FlatMultimap&&) = default;

    std::vector<Value>::iterator insert(Key const& k, Value const& v)
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        while (key_position != keys.end() && *key_position == k)
            ++key_position;
        auto item_position = std::next(items.begin(), std::distance(keys.begin(), key_position));
        keys.insert(key_position, k);
        return items.insert(item_position, v);
    }

    std::size_t key_index(Key const& k)
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        if (key_position != keys.end() && *key_position == k)
            return std::distance(keys.begin(), key_position);
        return ~(std::size_t)0;
    }

    std::vector<Value> items{};
    std::vector<Key> keys{};
};

template <typename LeftKey, typename RightKey>
struct BijectiveMap
{
    BijectiveMap() = default;
    BijectiveMap(BijectiveMap const&) = default;
    BijectiveMap(BijectiveMap&&) = default;
    BijectiveMap& operator=(BijectiveMap const&) = default;
    BijectiveMap& operator=(BijectiveMap&&) = default;

    auto emplace(LeftKey const& lk, RightKey const& rk) {
        left.emplace(lk, rk);
        return right.emplace(rk, lk);
    }

    auto emplace(RightKey const& rk, LeftKey const& lk) {
        right.emplace(rk, lk);
        return left.emplace(lk, rk);
    }

    void erase(LeftKey const& lk) {
        if (!left.contains(lk))
            return;
        RightKey const& rk = left[lk];
        left.erase(lk);
        right.erase(rk);
    }

    void erase(RightKey const& rk) {
        if (!right.contains(rk))
            return;
        LeftKey const& lk = right[rk];
        right.erase(rk);
        left.erase(lk);
    }

    bool contains(LeftKey const& lk) const {
        return left.contains(lk);
    }

    bool contains(RightKey const& rk) const {
        return right.contains(rk);
    }

    std::unordered_map<LeftKey, RightKey> left;
    std::unordered_map<RightKey, LeftKey> right;
};

struct Registry
{
    using Node = FreeList::Index;
    using ComponentID = uint64_t;
    using ComponentTag = uint64_t;

    struct ComponentBinding {
        ComponentID type;
        ComponentTag tag;
    };

    struct ComponentStorage {
        FreeList indices;
        BlockVector data;
        BlockVector bindings;
    };

    std::unordered_map<ComponentID, ComponentStorage> components;
    std::unordered_map<Node, std::vector<ComponentBinding>> nodes;
    FreeList handle_pool;

    Node MakeNode() {
        Node node = handle_pool.Reserve();
        nodes[node] = {};
        return node;
    }

    static ComponentID DeclareComponent() {
        static ComponentID next_component = 0;
        return next_component++;
    }

    template <typename T>
    static ComponentID ComponentImpl() {
        static ComponentID const tag = DeclareComponent();
        return tag;
    }

    template <typename T>
    static ComponentID Component() {
        return ComponentImpl<typename std::remove_cvref<T>::type>();
    }

    template <typename ArgType> void BindComponent(Node node, ArgType&& component)
    {
        using CType = typename std::remove_cvref<ArgType>::type;
        ComponentID const component_id = Component<CType>();

        if (components.count(component_id) == 0)
            components.emplace(component_id, ComponentStorage{
                {}, BlockVector{ sizeof(CType), 256 }, BlockVector{ sizeof(uint64_t), 2048 }
            });

        ComponentStorage& storage = components.at(component_id);
        ComponentTag component_tag = storage.indices.Reserve();
        storage.data.Expand(storage.indices.RequiredSize());
        storage.bindings.Expand(storage.indices.RequiredSize());

        new (storage.data[component_tag]) CType(std::forward<ArgType>(component));
        *(uint64_t*)storage.bindings[component_tag] = node;

        nodes[node].push_back({ component_id, component_tag });
    }

    template <typename CType> CType* ComponentLookup(Node n) {
        return (CType*)((Registry const*)this)->ComponentLookup<CType>(n);
    }

    template <typename CType> CType const* ComponentLookup(Node n) const {
        ComponentID const component_id = Component<CType>();

        std::vector<ComponentBinding> const& bindings = nodes.at(n);
        auto binding_it = std::find_if(
            bindings.begin(), bindings.end(),
            [](ComponentBinding const& binding){
                return binding.type == Component<CType>();
            });
        if (binding_it == bindings.end())
            return nullptr;

        ComponentStorage const& storage = components.at(component_id);
        return (CType const*)storage.data[binding_it->tag];
    }
};

}
