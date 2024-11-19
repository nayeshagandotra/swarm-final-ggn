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
GlobalPlanner::GlobalPlanner(int num_agents, std::unordered_map<int,std::shared_ptr<Block>> large_gridmap, 
                             int largemap_xsize, int largemap_ysize)
    : num_agents_(num_agents), large_gridmap_(large_gridmap), largemap_xsize_(largemap_xsize), 
    largemap_ysize_(largemap_ysize) {}

// Destructor
GlobalPlanner::~GlobalPlanner() {}

// Method to check if a cell is within bounds and free
bool GlobalPlanner::isFree(std::shared_ptr<Block> p) const {
    // if (p->obsCost > 0){
    //     return false;
    // }
    // return true;
    return true; //imagine an empty map for now
}

int euclidean(std::shared_ptr<Block> p1, std::shared_ptr<Block> p2){
    return sqrt((p1->x - p2->x)*(p1->x - p2->x) + (p1->y - p2->y)*(p1->y - p2->y));
}

int manhattan(std::shared_ptr<Block> p1, std::shared_ptr<Block> p2){
    return abs((p1->x - p2->x)) + abs((p1->y - p2->y));
}

struct BlockComparator {
    bool operator()(const std::shared_ptr<Block> s1, const std::shared_ptr<Block> s2) const {
        return (s1->g + s1->h) > (s2->g + s2->h);  // min heap
    }
};

// Function to get successors of a block in a 4-connected grid
std::vector<std::shared_ptr<Block>> GlobalPlanner::getSuccessors(std::shared_ptr<Block> p) {
    std::vector<std::shared_ptr<Block>> successors;

    // Define the 4 possible movement directions: up, down, left, right
    std::vector<std::pair<int, int>> directions = {
        {0, 1},  // Right
        {1, 0},  // Down
        {0, -1}, // Left
        {-1, 0}  // Up
    };

    // Loop through each direction
    for (const auto& [dx, dy] : directions) {
        int newX = p->x + dx;
        int newY = p->y + dy;

        // Check if the new position is within bounds
        if (newX >= 0 && newX < largemap_xsize_ && newY >= 0 && newY < largemap_ysize_) {
            successors.push_back(large_gridmap_[GETMAPINDEX(newX, newY, largemap_xsize_, largemap_ysize_)]);
        }
    }
    return successors;
}

std::vector<std::pair<int, int>> reconstructPath(std::shared_ptr<Block> start, std::shared_ptr<Block> goal) {
    // returns the sequence of x and y positions the swarm should take

    std::vector<std::pair<int, int>> path;
    std::shared_ptr<Block> next = goal;
    std::pair<int, int> step = {goal->x, goal->y};
    while (step != std::make_pair(start->x, start->y)) {
        path.push_back(step);
        next = next->parent;
        step = std::make_pair(next->x, next->y);
    }
    path.push_back(std::make_pair(start->x, start->y));
    std::reverse(path.begin(), path.end());
    return path;
}


// Method to plan a path using a basic A* algorithm
std::vector<std::pair<int, int>> GlobalPlanner::planPath(std::shared_ptr<Block> start, std::shared_ptr<Block> goal) {

    std::priority_queue<std::shared_ptr<Block>, std::vector<std::shared_ptr<Block>>, BlockComparator> open;
    start->g = 0;
    start->h = manhattan(start, goal);
    open.push(start);

    bool found = false;

    // BFS for finding the path
    while (!open.empty() && !goal->closed_astar) {
        auto current = open.top();
        current->closed_astar = true;
        open.pop();

        // successors is a vector of blocks
        for (const auto& successor : getSuccessors(current)) {

            if (isFree(successor) && !successor->closed_astar) {
                if (successor->g > current->g + successor->obsCost){
                    successor->g = current->g + successor->obsCost;
                    successor->parent = current;
                    open.push(successor);
                }
            }
        }
    }

    // now reconstruct (backtrack)
    std::vector<std::pair<int, int>> path = reconstructPath(start, goal);

    return path;
}
