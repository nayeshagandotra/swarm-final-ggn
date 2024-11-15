#include "load_map.h"

// Constructor implementation for Block
Block::Block(int size, int x, int y, int obsCost, int* subMap)
    : size(size), x(x), y(y), obsCost(obsCost), subMap(subMap) {}

// Function implementation for makeCoarseGraph
std::unordered_map<int, std::shared_ptr<Block>> makeCoarseGraph(
    int* fineGraph,
    int x_size,
    int y_size,
    int swarm_size) 
    {
    int num_blocks_x = x_size / swarm_size;
    int num_blocks_y = y_size / swarm_size;
    std::unordered_map<int, std::shared_ptr<Block>> coarseMap;

    for (int block_y = 0; block_y < num_blocks_y; ++block_y) {
        for (int block_x = 0; block_x < num_blocks_x; ++block_x) {
            int* subMap = new int[swarm_size * swarm_size];
            int obsCost = 0;

            // Process the cells within the current block
            for (int i = 0; i < swarm_size; ++i) {
                for (int j = 0; j < swarm_size; ++j) {
                    int x = block_x * swarm_size + j; // Absolute x-coordinate
                    int y = block_y * swarm_size + i; // Absolute y-coordinate
                    if (x < x_size && y < y_size) {
                        obsCost += fineGraph[GETMAPINDEX(x, y, x_size, y_size)];
                        subMap[GETMAPINDEX(j, i, swarm_size, swarm_size)] =
                            fineGraph[GETMAPINDEX(x, y, x_size, y_size)];
                    }
                }
            }

            std::shared_ptr<Block> curBlock =
                std::make_shared<Block>(swarm_size, block_x, block_y, obsCost, subMap);
            coarseMap.insert({GETMAPINDEX(block_x, block_y, num_blocks_x, num_blocks_y), curBlock});
        }
    }
    return coarseMap;
}
