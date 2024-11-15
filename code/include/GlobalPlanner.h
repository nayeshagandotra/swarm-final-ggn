#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

#include <vector>
#include <utility>
#include <unordered_map>

class GlobalPlanner {
public:
    // GlobalPlanner::GlobalPlanner(int width, int height) 
    // : width_(width), height_(height), grid_(width, std::vector<bool>(height, true)) {}
    GlobalPlanner(int num_agents, std::unordered_map<int,std::shared_ptr<Block>> large_gridmap, int largemap_xsize, int largemap_ysize);
    ~GlobalPlanner();

    // Method to plan a path from start to goal
    std::vector<Block&> getSuccessors(Block& p);
    std::vector<std::pair<int, int>> planPath(Block& start, Block& goal);


private:
    int num_agents_;         // Number of agents
    int* large_gridmap_;      // Pointer to a grid map
    int largemap_xsize;
    int largemap_ysize;
    std::unordered_map<int,std::shared_ptr<Block>> large_gridmap_; // 2D grid for obstacles

    // Helper function to check if a cell is within bounds and free
    bool isFree(int x, int y) const;
};

#endif // GLOBALPLANNER_H
