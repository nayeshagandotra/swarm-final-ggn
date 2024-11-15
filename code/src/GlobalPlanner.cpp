#include "../include/GlobalPlanner.h"
#include <math.h>
#include <vector>
#include <queue>
#include <chrono>
#include <iostream>  // For std::cout
#include <fstream>
#include <limits>
#include <thread>
#include <unordered_map>
#include "load_map.h"

// Constructor
GlobalPlanner::GlobalPlanner(int num_agents, std::unordered_map<int,std::shared_ptr<Block>> large_gridmap, int largemap_xsize, int largemap_ysize)
    : num_agents_(num_agents), large_gridmap_(large_gridmap) {}

// Destructor
GlobalPlanner::~GlobalPlanner() {}

// Method to check if a cell is within bounds and free
bool isFree(Block& p){
    if (p.obsCost > 0){
        return false;
    }
    return true;
}

int euclidean(Block& p1, Block& p2){
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

int manhattan(Block& p1, Block& p2){
    return abs((p1.x - p2.x)) + abs((p1.y - p2.y));
}

struct BlockComparator {
    bool operator()(const Block& s1, const Block& s2) const {
        return (s1.g + s1.h) > (s2.g + s2.h);  // min heap
    }
};

std::vector<Block&> GlobalPlanner::getSuccessors(Block& p) {
        std::vector<Block&> successors;
        int x = p.x;
        int y = p.y;

        GETMAPINDEX()
        // Check the four possible directions and add valid neighbors
        successors.push_back(large_gridmap_[x - 1][y]); // Up
        successors.push_back(large_gridmap_[x + 1][y]); // Down
        successors.push_back(large_gridmap_[x][y - 1]); // Left
        successors.push_back(large_gridmap_[x][y + 1]); // Right

        return successors;
}

// Method to plan a path using a basic A* algorithm
std::vector<std::pair<int, int>> GlobalPlanner::planPath(Block& start, Block& goal) {

    std::priority_queue<Block&, std::vector<Block&>, BlockComparator> open;
    start.g = 0;
    start.h = manhattan(start, goal);
    open.push(start);

    bool found = false;

    // BFS for finding the path
    while (!open.empty() && !found) {
        auto current = open.top();
        current.closed_astar = true;
        open.pop();

        for (const auto& successor : getSuccessors(current)) {
            int newX = current.first + direction.first;
            int newY = current.second + direction.second;

            if (isFree(newX, newY) && !visited[newX][newY]) {
                queue.push({newX, newY});
                visited[newX][newY] = true;
                parent[newX][newY] = current;

                if (newX == goal.first && newY == goal.second) {
                    found = true;
                    break;
                }
            }
        }
    }

    if (found) {
        // Reconstruct the path from goal to start
        std::pair<int, int> step = goal;
        while (step != start) {
            path.push_back(step);
            step = parent[step.first][step.second];
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
    }

    return path;
}
