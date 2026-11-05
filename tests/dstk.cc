#include "dstk.hh"

#include <iostream>

int main()
{
    dstk::Registry registry = {};
    dstk::DirectedGraph root_graph{ &registry };
    root_graph.NodeCreate("A");
    root_graph.NodeCreate("B");
    root_graph.NodeCreate("C");
    root_graph.NodeCreate("D");
    root_graph.NodeCreate("E");
    root_graph.NodeCreate("F");

    root_graph.EdgeCreate("A", "C");
    root_graph.EdgeCreate("B", "C");
    root_graph.EdgeCreate("C", "D");
    root_graph.EdgeCreate("D", "E");
    root_graph.EdgeCreate("D", "F");

    for (auto it = root_graph.begin(); it != root_graph.end(); ++it)
        std::cout << root_graph.name_lookup[*it] << std::endl;
    std::cout << std::endl;

    for (auto it = root_graph.rbegin(); it != root_graph.rend(); ++it)
        std::cout << root_graph.name_lookup[*it] << std::endl;
    std::cout << std::endl;

    dstk::LayeredGraph test_graph0{ std::move(root_graph) };

    dstk::DirectedGraph& subgraph0 = test_graph0["C"].SubgraphCreate();
    subgraph0.NodeCreate("G");
    subgraph0.NodeCreate("H");
    subgraph0.NodeCreate("I");
    subgraph0.EdgeCreate("G", "I");
    subgraph0.EdgeCreate("H", "I");

    for (auto it = test_graph0.begin(); it != test_graph0.end(); ++it) {
        std::cout << std::string(it.Depth() - 1, ' ');
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    }
    std::cout << std::endl;

    for (auto it = test_graph0.rbegin(); it != test_graph0.rend(); ++it) {
        std::cout << std::string(it.Depth() - 1, ' ');
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    }
    std::cout << std::endl;

    {
        dstk::LayeredGraph const& const_graph = test_graph0;
        dstk::LayeredGraph::ConstNodeView node_c = const_graph["C"];
        dstk::LayeredGraph::ConstNodeView node_g = node_c["G"];
    }

    test_graph0["A"] = dstk::DirectedGraph{
        &registry,
        { "J", "K", "L" },
        { { "J", "K" }, { "J", "L" } }
    };

    test_graph0["A"]["K"] = dstk::DirectedGraph{
        &registry,
        { "M", "N" },
        {}
    };

    for (auto it = test_graph0.begin(); it != test_graph0.end(); ++it) {
        std::cout << std::string(it.Depth() - 1, ' ');
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    }
    std::cout << std::endl;

    dstk::LayeredGraph test_graph1{
        {
            &registry,
            { "O", "P", "Q", "R" },
            { { "O", "P" }, { "O", "Q" }, { "P", "R" }, { "Q", "R" } }
        }
    };

    test_graph1["P"] = std::move(test_graph0);
    test_graph1["P"].NodeRepeatCount() = 3;

    for (auto it = test_graph1.begin(); it != test_graph1.end(); ++it)
    {
        std::cout << std::string(it.Depth() - 1, ' ');
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    }
    std::cout << std::endl;

    for (auto it = test_graph1.rbegin(); it != test_graph1.rend(); ++it)
    {
        std::cout << std::string(it.Depth() - 1, ' ');
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    }
    std::cout << std::endl;

    for (auto name : registry.ComponentGlobalLookup<std::string>())
        std::cout << *name << std::endl;
    std::cout << std::endl;

    dstk::LayeredGraph test_graph2{
        { &registry, { "A", "B", "C", "F" }, { { "A", "B" }, { "B", "C" }, { "C", "F" } } },
    };
    test_graph2["B"] = dstk::DirectedGraph{ &registry, { "D", "E" }, { { "D", "E" } } };

#if 0
    dstk::NodeTree tree2{ test_graph2 };
    dstk::NodeTree tree1{ test_graph1 };
#endif

    return 0;
}
