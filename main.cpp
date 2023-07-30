#include <iostream>
#include "GraphClass.h"

int main()
{
    Graph<char> graph;
    graph.addConnection('A', 'B', 1.5);
    graph.addConnection('A', 'C', 2.0);
    graph.addConnection('B', 'D', 3.1);
    graph.addConnection('C', 'E', 2.5);
    graph.addConnection('C', 'F', 1.8);

    std::cout << "Graph:" << std::endl;
    graph.printGraph();

    char startNode = 'A';
    char destinationNode = 'F';

    std::map<char, float> distances = graph.dijkstra(startNode, destinationNode);

    float shortestDistance = distances[destinationNode];

    if (shortestDistance != std::numeric_limits<float>::infinity())
        std::cout << "La distancia mas corta del nodo " << startNode << " hacia el nodo " << destinationNode << ": " << shortestDistance << std::endl;
    else
        std::cout << "No se encontro un camino desde el nodo " << startNode << " hacia el nodo " << destinationNode << std::endl;

    return 0;
}











