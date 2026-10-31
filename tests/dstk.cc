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

    for (auto it = root_graph.ForwardTraversal_begin();
         it != root_graph.ForwardTraversal_end(); ++it)
        std::cout << root_graph.name_lookup[*it] << std::endl;
    std::cout << std::endl;

    for (auto it = root_graph.BackwardTraversal_begin();
         it != root_graph.BackwardTraversal_end(); ++it)
        std::cout << root_graph.name_lookup[*it] << std::endl;
    std::cout << std::endl;

    dstk::LayeredGraph layered_graph{};
    layered_graph.root = &root_graph;
    dstk::DirectedGraph& subgraph0 = layered_graph.SubgraphCreate("C");
    subgraph0.NodeCreate("G");
    subgraph0.NodeCreate("H");
    subgraph0.NodeCreate("I");
    subgraph0.EdgeCreate("G", "I");
    subgraph0.EdgeCreate("H", "I");

    for (auto it = layered_graph.ForwardTraversal_begin();
         it != layered_graph.ForwardTraversal_end(); ++it)
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    std::cout << std::endl;

    for (auto it = layered_graph.BackwardTraversal_begin();
         it != layered_graph.BackwardTraversal_end(); ++it)
        std::cout << it.iterator_stack.back().graph->name_lookup[*it] << std::endl;
    std::cout << std::endl;

    return 0;
}
