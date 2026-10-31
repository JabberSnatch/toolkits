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

    for (auto it = root_graph.BackwardTraversal_begin();
         it != root_graph.BackwardTraversal_end(); ++it)
        std::cout << root_graph.name_lookup[*it] << std::endl;

    return 0;
}
