#pragma once

#include <algorithm>
#include <iterator>
#include <cassert>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace dstk
{

struct FreeList
{
    using Index = uint64_t;
    static constexpr Index kInvalidIndex = ~0ull;
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

    uint64_t RequiredSize() const {
        return indices.back().begin;
    }

    struct IndexIterator {
        IndexIterator& operator++() {
            if (next_range_index >= list->indices.size())
                return *this;

            ++index;
            if (index >= list->indices[next_range_index].begin)
                index = list->indices[next_range_index++].end;
            return *this;
        }

        bool operator==(IndexIterator const& rhs) { return list == rhs.list && index == rhs.index; }
        bool operator!=(IndexIterator const& rhs) { return list != rhs.list || index != rhs.index; }

        Index operator*() const { return index; }

        Index index;
        std::size_t next_range_index;
        FreeList const* list;
    };

    IndexIterator begin() const { return IndexIterator{ 0, 0, this }; }
    IndexIterator end() const { return IndexIterator{ kInvalidIndex, indices.size(), this }; }

    std::vector<Range> indices;
};

struct BlockVector
{
    BlockVector(uint64_t _object_size, uint64_t _block_size)
        : object_size{ _object_size }
        , block_size{ _block_size }
    {}

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

        uint64_t offset = (uint64_t)((uint8_t const*)mem - block_it->get());
        assert(offset % object_size == 0);
        return offset / object_size;
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

    ObjectPool() = default;
    ObjectPool(uint64_t block_size) : items{ sizeof(T), block_size } {}

    void Expand(uint64_t required_size)
    {
        items.Expand(required_size);
    }

    Handle Reserve() { return Emplace(); }

    template <typename ... Args>
    Handle Emplace(Args&&... args) {
        uint64_t index = free_list.Reserve();
        items.Expand(free_list.RequiredSize());
        new (items[index]) T{ args... };
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

    struct ValueIteratorInterface
    {
        std::vector<Value>& items;
        auto begin() { return items.begin(); }
        auto end() { return items.end(); }
    };

    ValueIteratorInterface ItemsRange() { return ValueIteratorInterface{ items }; }

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

    std::size_t key_index(Key const& k) const
    {
        auto key_position = std::lower_bound(keys.begin(), keys.end(), k);
        if (key_position != keys.end() && *key_position == k)
            return std::distance(keys.begin(), key_position);
        return ~(std::size_t)0;
    }

    std::size_t count(Key const& k) const
    {
        std::size_t index = key_index(k);
        if (index >= keys.size())
            return 0;

        std::size_t count = 0;
        while (index+count++ < keys.size()-1
               && keys[index+count] == k);
        return count;
    }

    std::size_t pair_index(Key const& k, Value const& v) const
    {
        std::size_t index = key_index(k);
        if (index >= keys.size())
            return ~(std::size_t)0;

        while(index < keys.size()
              && keys[index] == k)
            if (v == items[index++])
                return index-1;

        return ~(std::size_t)0;
    }

    std::vector<Value> items{};
    std::vector<Key> keys{};
};

template <typename LeftKey, typename RightKey>
struct BijectiveMap
{
    template <typename T>
    using Hash = std::hash<T>;

#if 0
    template <typename T>
    struct NoHash {
        std::size_t operator()(T const&) { return 0; }
    };
    template <typename T>
    using Hash = NoHash<T>;
#endif

    BijectiveMap() = default;
    BijectiveMap(BijectiveMap const&) = default;
    BijectiveMap(BijectiveMap&&) = default;
    BijectiveMap& operator=(BijectiveMap const&) = default;
    BijectiveMap& operator=(BijectiveMap&&) = default;

    void emplace(LeftKey const& lk, RightKey const& rk)
    {
        std::size_t lk_hash = Hash<LeftKey>{}(lk);
        std::size_t rk_hash = Hash<RightKey>{}(rk);
        left.insert(lk_hash, pairs.size());
        right.insert(rk_hash, pairs.size());
        pairs.emplace_back(lk, rk);
    }

    template <typename DstKey, typename SrcKey>
    bool key_query(SrcKey const& src,
                   FlatMultimap<std::size_t, std::size_t> const& src_map,
                   DstKey const** dst) const
    {
        std::size_t src_hash = Hash<SrcKey>{}(src);
        std::size_t match_count = src_map.count(src_hash);
        if (!match_count)
            return false;

        std::size_t src_index = src_map.key_index(src_hash);
        if (match_count == 1)
        {
            if (dst)
            {
                *dst = &std::get<DstKey>(pairs[src_map.items[src_index]]);
                return true;
            }
            else
                return std::get<SrcKey>(pairs[src_map.items[src_index]]) == src;
        }

        while (src_index < src_map.keys.size()
               && src_map.keys[src_index] == src_hash)
        {
            std::size_t pair_index = src_map.items[src_index];
            if (std::get<SrcKey>(pairs[pair_index]) == src)
            {
                if (dst)
                    *dst = &std::get<DstKey>(pairs[pair_index]);
                return true;
            }
            ++src_index;
        }

        return false;
    }

    RightKey const& operator[](LeftKey const& lk) const {
        RightKey const* output = {};
        if (!key_query(lk, left, &output))
            throw std::out_of_range{"Invalid key"};
        return *output;
    }

    LeftKey const& operator[](RightKey const& rk) const {
        LeftKey const* output = {};
        if (!key_query(rk, right, &output))
            throw std::out_of_range{"Invalid key"};
        return *output;
    }

    bool contains(LeftKey const& lk) const {
        return key_query<RightKey>(lk, left, nullptr);
    }

    bool contains(RightKey const& rk) const {
        return key_query<LeftKey>(rk, right, nullptr);
    }

    std::size_t size() const { return pairs.size(); }

    FlatMultimap<std::size_t, std::size_t> left;
    FlatMultimap<std::size_t, std::size_t> right;
    std::vector<std::pair<LeftKey, RightKey>> pairs;
};

struct Registry
{
    using Node = FreeList::Index;
    using ComponentID = uint64_t;
    using ComponentTag = uint64_t;
    using ComponentKey = uint32_t;
    static constexpr Node kInvalidNode = FreeList::kInvalidIndex;

    struct ComponentBinding {
        ComponentID type;
        ComponentTag index;
        ComponentKey order_key;
    };

    struct ComponentStorage {
        std::string type_name;
        FreeList indices;
        BlockVector data;
        BlockVector bindings;
        std::function<void(void*)> dtor;
    };

    Registry() = default;
    Registry(Registry const&) = delete;
    Registry(Registry&&) = default;
    Registry& operator=(Registry const&) = delete;
    Registry& operator=(Registry&&) = default;
    ~Registry() {
        for (auto&& pair : components)
        {
            for (FreeList::Index index : pair.second.indices)
            {
                void* component = pair.second.data[index];
                pair.second.dtor(component);
            }
        }
    }

    std::unordered_map<ComponentID, ComponentStorage> components;
    std::unordered_map<Node, std::vector<ComponentBinding>> nodes;
    FreeList handle_pool;

    Node MakeNode() {
        Node node = handle_pool.Reserve();
        nodes[node] = {};
        return node;
    }

    void EraseNode(Node node) {
        std::vector<ComponentBinding> bindings = nodes[node];
        for (ComponentBinding const& binding : bindings)
            ComponentErase(binding);
        handle_pool.Release(node);
    }

    static void InsertComponentBinding(std::vector<ComponentBinding>& components,
                                       ComponentBinding const& binding) {
        auto binding_it = std::lower_bound(
            components.begin(), components.end(), binding,
            [](ComponentBinding const& lhs, ComponentBinding const& rhs) {
                return lhs.type < rhs.type ||
                    (lhs.type == rhs.type && lhs.order_key < rhs.order_key);
            });

        components.insert(binding_it, binding);
    }

    static ComponentBinding const* LookupComponentBinding(std::vector<ComponentBinding> const& components,
                                                          ComponentID type,
                                                          ComponentKey order_key) {
        auto binding_it = std::lower_bound(
            components.begin(), components.end(), std::make_pair(type, order_key),
            [](ComponentBinding const& lhs, auto const& rhs) {
                return lhs.type < rhs.first ||
                    (lhs.type == rhs.first && lhs.order_key < rhs.second);
            });
        if (binding_it != components.end()
            && binding_it->type == type
            && binding_it->order_key == order_key)
            return &*binding_it;
        return nullptr;
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

    template <typename T>
    static std::function<void(void*)> ComponentDtor() {
        return [](void* ptr){ ((T*)ptr)->~T(); };
    }

    template <typename ArgType> auto BindComponent(Node node, ArgType&& component)
    {
        static_assert(!std::is_integral_v<std::remove_cvref_t<ArgType>>,
                      "Binding integral without tag?");
        return BindComponent(node, 0, std::forward<ArgType>(component));
    }

    template <typename ArgType> auto BindComponent(Node node, uint32_t key, ArgType&& component)
    {
        using CType = typename std::remove_cvref<ArgType>::type;
        ComponentID const component_id = Component<CType>();

        if (components.count(component_id) == 0)
            components.emplace(component_id, ComponentStorage{
                typeid(std::declval<CType>()).name(),
                FreeList{},
                BlockVector{ sizeof(CType), 256 },
                BlockVector{ sizeof(uint64_t), 2048 },
                ComponentDtor<CType>(),
            });

        ComponentStorage& storage = components.at(component_id);
        ComponentTag component_index = storage.indices.Reserve();
        storage.data.Expand(storage.indices.RequiredSize());
        storage.bindings.Expand(storage.indices.RequiredSize());

        new (storage.data[component_index]) CType(std::forward<ArgType>(component));
        *(uint64_t*)storage.bindings[component_index] = node;

        InsertComponentBinding(nodes[node], { component_id, component_index, key });

        return (CType*)storage.data[component_index];
    }

    template <typename CType> CType* ComponentLookup(Node n) {
        return (CType*)((Registry const*)this)->ComponentLookup<CType>(n);
    }

    template <typename CType> CType const* ComponentLookup(Node n) const {
        ComponentID const component_id = Component<CType>();

        std::vector<ComponentBinding> const& bindings = nodes.at(n);

        ComponentBinding const* binding = LookupComponentBinding(bindings, component_id, 0);
        if (!binding)
            return nullptr;

        ComponentStorage const& storage = components.at(component_id);
        return (CType const*)storage.data[binding->index];
    }

    template <typename CType> CType* ComponentLookup(Node n, uint32_t key) {
        return (CType*)((Registry const*)this)->ComponentLookup<CType>(n, key);
    }

    template <typename CType> CType const* ComponentLookup(Node n, uint32_t key) const {
        ComponentID const component_id = Component<CType>();

        std::vector<ComponentBinding> const& bindings = nodes.at(n);

        ComponentBinding const* binding = LookupComponentBinding(bindings, component_id, key);
        if (!binding)
            return nullptr;

        ComponentStorage const& storage = components.at(component_id);
        return (CType const*)storage.data[binding->index];
    }

    template <typename CType> std::vector<CType const*> ComponentRangeLookup(Node n) const {
        ComponentID const component_id = Component<CType>();
        if (components.count(component_id) == 0)
            return {};

        std::vector<ComponentBinding> const& bindings = nodes.at(n);
        auto range_begin = std::lower_bound(
            bindings.begin(), bindings.end(), component_id,
            [](ComponentBinding const& lhs, ComponentID rhs) { return lhs.type < rhs; });
        auto range_end = std::upper_bound(
            bindings.begin(), bindings.end(), component_id,
            [](ComponentID lhs, ComponentBinding const& rhs) { return lhs < rhs.type; });

        std::vector<CType const*> result = {};
        ComponentStorage const& storage = components.at(component_id);
        std::transform(range_begin, range_end, std::back_inserter(result),
                       [&storage](ComponentBinding const& binding) {
                           return (CType const*)storage.data[binding.index];
                       });

        return result;
    }

    template <typename CType> std::vector<CType const*> ComponentGlobalLookup() const {
        ComponentID const component_id = Component<CType>();
        if (components.count(component_id) == 0)
            return {};

        std::vector<CType const*> result = {};
        ComponentStorage const& storage = components.at(component_id);
        std::transform(storage.indices.begin(), storage.indices.end(), std::back_inserter(result),
                       [&storage](FreeList::Index index) {
                           return (CType const*)storage.data[index];
                       });

        return result;
    }

    template <typename CType> CType& ComponentRequest(Node n) {
        CType* component = ComponentLookup<CType>(n);
        if (!component)
            component = BindComponent(n, CType{});
        return *component;
    }

    template <typename CType> CType& ComponentRequest(Node n, uint32_t key) {
        CType* component = ComponentLookup<CType>(n, key);
        if (!component)
            component = BindComponent(n, key, CType{});
        return *component;
    }

    template <typename CType> void ComponentRangeErase(Node n, uint32_t key_begin = 0, uint32_t key_end = UINT32_MAX) {
        ComponentID const component_id = Component<CType>();
        if (components.count(component_id) == 0)
            return;

        std::vector<ComponentBinding>& bindings = nodes.at(n);

        auto range_begin = std::lower_bound(
            bindings.begin(), bindings.end(), std::make_pair(component_id, key_begin),
            [](ComponentBinding const& lhs, auto rhs) {
                return lhs.type < rhs.first ||
                    (lhs.type == rhs.first && lhs.order_key < rhs.second);
            });
        auto range_end = std::upper_bound(
            bindings.begin(), bindings.end(), std::make_pair(component_id, key_end),
            [](auto lhs, ComponentBinding const& rhs) {
                return lhs.first < rhs.type ||
                    (lhs.first == rhs.type && lhs.second < rhs.order_key);
            });

        ComponentStorage& storage = components.at(component_id);
        for (auto component = range_begin; component != range_end; ++component)
        {
            storage.dtor(storage.data[component->index]);
            storage.indices.Release(component->index);
        }
        bindings.erase(range_begin, range_end);
    }

    template <typename CType> void ComponentErase(Node n, CType const* component) {
        ComponentID const component_id = Component<CType>();

        std::vector<ComponentBinding>& bindings = nodes.at(n);
        ComponentStorage& storage = components.at(component_id);
        auto binding_it = std::find_if(
            bindings.begin(), bindings.end(),
            [&storage, component, component_id](ComponentBinding const& binding) {
                return component_id == binding.type && storage.data[binding.index] == component;
            });

        if (binding_it == bindings.end())
            return;

        ComponentErase(*binding_it);
        bindings.erase(binding_it);
    }

    void ComponentErase(ComponentBinding const& binding) {
        ComponentStorage& storage = components.at(binding.type);
        storage.dtor(storage.data[binding.index]);
        storage.indices.Release(binding.index);
    }

    template <typename CType> Node NodeLookup(CType const* component) const {
        ComponentID const component_id = Component<CType>();

        ComponentStorage const& storage = components.at(component_id);
        uint64_t index = storage.data.Find(component);
        return *(Node*)storage.bindings[index];
    }
};

struct DirectedGraph
{
    enum EdgeTag {
        EdgeTag_Invalid,
        EdgeTag_Primary,
        EdgeTag_Secondary,
    };

    struct Edge {
        EdgeTag tag;
        Registry::Node source;
        Registry::Node sink;
    };

    struct NodeAdjacency {
        std::vector<Registry::Node> in;
        std::vector<Registry::Node> out;
    };

    DirectedGraph(Registry* _registry = nullptr)
        : registry{ _registry }
    {}

    DirectedGraph(DirectedGraph const&) = delete;
    DirectedGraph& operator=(DirectedGraph const&) = delete;

    DirectedGraph(DirectedGraph&& _o) = default;
    DirectedGraph& operator=(DirectedGraph&&) = default;

    ~DirectedGraph() {
        if (!registry)
            return;

        for (Registry::Node edge : edges)
            registry->EraseNode(edge);
        for (Registry::Node node : nodes)
            registry->EraseNode(node);
    }

    Registry::Node NodeCreate(std::string const& name = "") {
        Registry::Node node = registry->MakeNode();

        auto location = std::lower_bound(nodes.begin(), nodes.end(), node);
        assert(location == nodes.end() || *location != node);
        nodes.insert(location, node);

        if (name != "" && !name_lookup.contains(name))
            name_lookup.emplace(node, name);
        return node;
    }

    size_t NodeIndex(Registry::Node node) const {
        auto location = std::lower_bound(nodes.begin(), nodes.end(), node);
        assert(location != nodes.end() && *location == node);
        return std::distance(nodes.begin(), location);
    }

    std::vector<Registry::Node> const& NodeInputs(Registry::Node node) const {
        static std::vector<Registry::Node> const empty{};
        NodeAdjacency* adjacency = registry->ComponentLookup<NodeAdjacency>(node);
        return adjacency != nullptr ? adjacency->in : empty;
    }

    std::vector<Registry::Node> const& NodeOutputs(Registry::Node node) const {
        static std::vector<Registry::Node> const empty{};
        NodeAdjacency* adjacency = registry->ComponentLookup<NodeAdjacency>(node);
        return adjacency != nullptr ? adjacency->out : empty;
    }

    Registry::Node EdgeCreate(std::string const& source,
                              std::string const& sink,
                              EdgeTag tag = EdgeTag_Primary) {
        Registry::Node source_node = name_lookup[source];
        Registry::Node sink_node = name_lookup[sink];

        Registry::Node edge = registry->MakeNode();
        edges.push_back(edge);

        {
            NodeAdjacency& adjacency = registry->ComponentRequest<NodeAdjacency>(source_node);
            adjacency.out.push_back(edge);
        }
        {
            NodeAdjacency& adjacency = registry->ComponentRequest<NodeAdjacency>(sink_node);
            adjacency.in.push_back(edge);
        }

        registry->BindComponent(edge, Edge{ tag, source_node, sink_node });
        return edge;
    }

    Edge EdgeLookup(Registry::Node edge) const {
        return *registry->ComponentLookup<Edge>(edge);
    }

    std::vector<Registry::Node> PrimaryInputs() const {
        std::vector<Registry::Node> result = {};
        for (Registry::Node node : nodes) {
            NodeAdjacency* adjacency = registry->ComponentLookup<NodeAdjacency>(node);
            if (!adjacency)
                continue;

            std::vector<Registry::Node> const& edges = adjacency->in;
            bool is_input = edges.empty() ||
                std::all_of(edges.begin(), edges.end(), [this](Registry::Node edge) {
                    return EdgeLookup(edge).tag == EdgeTag_Secondary;
                });

            if (is_input)
                result.push_back(node);
        }
        return result;
    }

    std::vector<Registry::Node> PrimaryOutputs() const {
        std::vector<Registry::Node> result = {};
        for (Registry::Node node : nodes) {
            NodeAdjacency* adjacency = registry->ComponentLookup<NodeAdjacency>(node);
            if (!adjacency)
                continue;

            std::vector<Registry::Node> const& edges = adjacency->out;
            bool is_output = edges.empty() ||
                std::all_of(edges.begin(), edges.end(), [this](Registry::Node edge) {
                    return EdgeLookup(edge).tag == EdgeTag_Secondary;
                });

            if (is_output)
                result.push_back(node);
        }
        return result;
    }

    Registry::Node operator[](std::string const& name) {
        dstk::Registry::Node node = dstk::Registry::kInvalidNode;
        if (!name_lookup.contains(name))
            node = NodeCreate(name);
        else
            node = name_lookup[name];
        return node;
    }

    Registry::Node operator[](std::string const& name) const {
        if (!name_lookup.contains(name))
            return dstk::Registry::kInvalidNode;
        return name_lookup[name];
    }

    struct ForwardMixin {
        static std::vector<Registry::Node> StackInit(DirectedGraph const* _graph) {
            return _graph->PrimaryInputs(); }
        static std::vector<Registry::Node> EdgeTraversal(DirectedGraph const* _graph,
                                                         Registry::Node _node) {
            return _graph->NodeOutputs(_node); }
        static Registry::Node EdgeDirection(DirectedGraph::Edge const& _edge) {
            return _edge.sink; }
    };

    struct BackwardMixin {
        static std::vector<Registry::Node> StackInit(DirectedGraph const* _graph) {
            return _graph->PrimaryOutputs(); }
        static std::vector<Registry::Node> EdgeTraversal(DirectedGraph const* _graph,
                                                         Registry::Node _node) {
            return _graph->NodeInputs(_node); }
        static Registry::Node EdgeDirection(DirectedGraph::Edge const& _edge) {
            return _edge.source; }
    };

    template <typename DirectionMixin>
    struct NodeIterator {
        NodeIterator(nullptr_t) {}
        NodeIterator(DirectedGraph const* _graph)
            : graph{ _graph }
            , node_index{ 0 }
            , repeat_index{ 0 }
            , node_stack{ DirectionMixin::StackInit(graph) }
            , queued_nodes(graph->nodes.size(), false)
        {
            for (Registry::Node queued_node : node_stack)
                queued_nodes[graph->NodeIndex(queued_node)] = true;
        }

        Registry::Node operator*() {
            return !node_stack.empty() ? node_stack[node_index] : Registry::kInvalidNode;
        }

        bool operator==(NodeIterator const& _o) const {
            return graph == _o.graph
                && node_index == _o.node_index
                && repeat_index == _o.repeat_index;
        }

        NodeIterator& operator++() {
            Registry::Node current_node = **this;
            if (current_node == Registry::kInvalidNode)
                return *this;

            ++node_index;

            for (Registry::Node edge_node : DirectionMixin::EdgeTraversal(graph, current_node))
            {
                DirectedGraph::Edge edge = graph->EdgeLookup(edge_node);
                if (edge.tag != DirectedGraph::EdgeTag_Primary)
                    continue;

                Registry::Node next_node = DirectionMixin::EdgeDirection(edge);

                size_t next_index = graph->NodeIndex(next_node);
                if (!queued_nodes[next_index])
                {
                    node_stack.push_back(next_node);
                    queued_nodes[next_index] = true;
                }
            }

            if (node_index == node_stack.size())
                *this = NodeIterator{ nullptr };

            return *this;
        }

        DirectedGraph const* graph = nullptr;
        size_t node_index = 0;
        uint32_t repeat_index = 0;
        std::vector<Registry::Node> node_stack = {};
        std::vector<bool> queued_nodes = {};
    };

    using ForwardNodeIterator = NodeIterator<ForwardMixin>;
    using BackwardNodeIterator = NodeIterator<BackwardMixin>;

    ForwardNodeIterator ForwardTraversal_begin() const { return ForwardNodeIterator{ this }; }
    ForwardNodeIterator ForwardTraversal_end() const { return ForwardNodeIterator{ nullptr }; }
    BackwardNodeIterator BackwardTraversal_begin() const { return BackwardNodeIterator{ this }; }
    BackwardNodeIterator BackwardTraversal_end() const { return BackwardNodeIterator{ nullptr }; }

    Registry* registry;
    BijectiveMap<Registry::Node, std::string> name_lookup;
    std::vector<Registry::Node> nodes;
    std::vector<Registry::Node> edges;
};

struct NodeTree
{
    void ChildrenInsert(dstk::Registry::Node parent, std::vector<dstk::Registry::Node> children);

    std::vector<dstk::Registry::Node> nodes;
    std::vector<uint32_t> offsets;
};

struct LayeredGraph
{
    DirectedGraph* root;
    NodeTree hierarchy;

    DirectedGraph& SubgraphCreate(std::string const& parent_node) {
        dstk::Registry::Node node = root->operator[](parent_node);
        DirectedGraph* output =
            root->registry->BindComponent(node, DirectedGraph{ root->registry });
        root->registry->BindComponent(node, output);
        return *output;
    }

    void SubgraphBind(std::string const& parent_node, DirectedGraph&& graph);
    void SubgraphBind(Registry::Node node, DirectedGraph* graph) {
        root->registry->ComponentRequest<DirectedGraph*>(node) = graph;

        DirectedGraph* stored_graph = root->registry->ComponentLookup<DirectedGraph>(node);
        if (stored_graph)
            root->registry->ComponentErase(node, stored_graph);
    }
    DirectedGraph const& SubgraphLookup(std::string const& name) const;

    struct NodeView {
        operator Registry::Node() const { return node; }
        NodeView operator[](std::string const& name) const;
        DirectedGraph const* graph;
        Registry::Node node;
    };

    struct ForwardMixin {
        using IteratorType = DirectedGraph::ForwardNodeIterator;
        static IteratorType begin(DirectedGraph const* _graph) {
            return _graph->ForwardTraversal_begin(); }
        static IteratorType end(DirectedGraph const* _graph) {
            return _graph->ForwardTraversal_end(); }
    };

    struct BackwardMixin {
        using IteratorType = DirectedGraph::BackwardNodeIterator;
        static IteratorType begin(DirectedGraph const* _graph) {
            return _graph->BackwardTraversal_begin(); }
        static IteratorType end(DirectedGraph const* _graph) {
            return _graph->BackwardTraversal_end(); }
    };

    template <typename DirectionMixin>
    struct NodeIterator {
        NodeIterator(nullptr_t) {}
        NodeIterator(LayeredGraph const* _graph)
            : graph{ _graph }
            , iterator_stack{ DirectionMixin::begin(graph->root) }
        {}

        Registry::Node operator*() {
            return !iterator_stack.empty() ? *iterator_stack.back() : Registry::kInvalidNode;
        }

        bool operator==(NodeIterator const& _o) const {
            return graph == _o.graph
                && iterator_stack == _o.iterator_stack;
        }

        NodeIterator& operator++() {
            typename DirectionMixin::IteratorType& iterator = iterator_stack.back();

            Registry::Node current_node = *iterator;
            DirectedGraph** subgraph =
                iterator.graph->registry->ComponentLookup<DirectedGraph*>(current_node);

            if (subgraph)
                iterator_stack.emplace_back(*subgraph);
            else
                while (!iterator_stack.empty()
                       && ++iterator_stack.back() == DirectionMixin::end(iterator.graph))
                    iterator_stack.pop_back();

            if (iterator_stack.empty())
                *this = NodeIterator{ nullptr };

            return *this;
        }

        uint32_t Depth() const { return (uint32_t)iterator_stack.size(); }
        LayeredGraph const* graph = nullptr;
        std::vector<typename DirectionMixin::IteratorType> iterator_stack = {};
    };

    using ForwardNodeIterator = NodeIterator<ForwardMixin>;
    using BackwardNodeIterator = NodeIterator<BackwardMixin>;

    ForwardNodeIterator ForwardTraversal_begin() const { return ForwardNodeIterator{ this }; }
    ForwardNodeIterator ForwardTraversal_end() const { return ForwardNodeIterator{ nullptr }; }
    BackwardNodeIterator BackwardTraversal_begin() const { return BackwardNodeIterator{ this }; }
    BackwardNodeIterator BackwardTraversal_end() const { return BackwardNodeIterator{ nullptr }; }
};

}

namespace std {

template <>
struct iterator_traits<dstk::FreeList::IndexIterator>
{
    using iterator_category = std::forward_iterator_tag;
};

}
