#ifndef GRAPHCLASS_H
#define GRAPHCLASS_H

#include <iostream>
#include <set>
#include <vector>
#include <queue>
#include <limits>
#include <map>
#include <stack>
#include <algorithm>

template <typename T>
class Graph 
{
    private:
        std::map<T, std::vector<std::pair<T, float>>> adjList;

    public:

        void addConnection(T vertex, T neighbor, float weight)
        {
            adjList[vertex].push_back(std::make_pair(neighbor, weight));
            adjList[neighbor].push_back(std::make_pair(vertex, weight));
        }

        void deleteConnection(T vertex, T neighbor)
        {
            auto it = adjList.find(vertex);
            if (it != adjList.end())
            {
                std::vector<std::pair<T, float>>& neighbors = it->second;
                for (auto neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt)
                {
                    if (neighborIt->first == neighbor)
                    {
                        neighbors.erase(neighborIt);
                        break;
                    }
                }
            }
        }

        void printGraph() 
        {
            for (const auto& entry : adjList) 
            {
                std::cout << entry.first << " -> ";
                for (const auto& neighbor : entry.second) 
                    std::cout << neighbor.first << "(" << neighbor.second << ") ";
                std::cout << std::endl;
            }
        }

        void dfs(T node)
        {
            std::set<T> visited;
            std::stack<T> stack;

            stack.push(node);

            while (!stack.empty())
            {
                T currentNode = stack.top();
                stack.pop();

                if (visited.find(currentNode) == visited.end())
                {
                    std::cout << currentNode << " ";
                    visited.insert(currentNode);

                    for (const auto& neighbor : adjList[currentNode])
                    {
                        if (visited.find(neighbor.first) == visited.end())
                            stack.push(neighbor.first);
                    }
                }
            }
        }

        void bfs(T node)
        {   
            std::set<T> visited;
            std::queue<T> queue;

            queue.push(node);

            while (!queue.empty())
            {
                T currentNode = queue.front();
                queue.pop();

                if (visited.find(currentNode) == visited.end())
                {
                    std::cout << currentNode << " ";
                    visited.insert(currentNode);

                    for (const auto& neighbor : adjList[currentNode])
                    {
                        if (visited.find(neighbor.first) == visited.end())
                            queue.push(neighbor.first);
                    }
                }
            }
        }

        std::map<T, float> dijkstra(T start, T destination)
        {
            std::map<T, float> distance;
            std::set<T> visited; 

            for (const auto& entry : adjList)
            {
                distance[entry.first] = std::numeric_limits<float>::infinity();
            }

            distance[start] = 0.0;

            std::priority_queue<std::pair<float, T>, std::vector<std::pair<float, T>>, std::greater<std::pair<float, T>>> pq;
            pq.push(std::make_pair(0.0, start));

            while (!pq.empty())
            {
                T current = pq.top().second;
                float dist = pq.top().first;
                pq.pop();

                if (current == destination)
                    return distance;

                if (visited.find(current) != visited.end())
                    continue;

                visited.insert(current);

                for (const auto& neighbor : adjList[current])
                {
                    T next = neighbor.first;
                    float weight = neighbor.second;

                    if (distance[current] + weight < distance[next])
                    {
                        distance[next] = distance[current] + weight;
                        pq.push(std::make_pair(distance[next], next));
                    }
                }
            }

            return distance;
        }
};

#endif //GRAPHCLASS_H