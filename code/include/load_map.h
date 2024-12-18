
#ifndef LOAD_MAP_H
#define LOAD_MAP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <memory>
#include <unordered_map>

// Macro to calculate 1D index from 2D coordinates
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y) * XSIZE + (X))

// Define directory for maps
#define MAPS_DIR "maps"

// Structure to represent a block in the coarse graph
struct Block {
    int x;        // X-coordinate of the block
    int y;        // Y-coordinate of the block
    int f;
    int g;
    int h;
    bool closed_astar = false;
    int obsCost;  // Obstacle cost for the block
    int* subMap;  // Submap representing the block
    int size;     // Size of the block (swarm size)
    std::shared_ptr<Block> parent = nullptr;

    // Constructor
    Block(int size, int x, int y, int obsCost, int* subMap);
};

// Function to generate a coarse graph from a fine graph
std::unordered_map<int, std::shared_ptr<Block>> makeCoarseGraph(
    int* fineGraph,
    int x_size,
    int y_size,
    int swarm_size);

#endif // LOAD_MAP_H
