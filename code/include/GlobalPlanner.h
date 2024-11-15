#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

#include <vector>
#include <utility>

class GlobalPlanner {
public:
    // GlobalPlanner::GlobalPlanner(int width, int height) 
    // : width_(width), height_(height), grid_(width, std::vector<bool>(height, true)) {}
    GlobalPlanner(int num_agents, std::vector<std::vector<bool>> large_gridmap);
    ~GlobalPlanner();

    // Method to plan a path from start to goal
    std::vector<std::pair<int, int>> planPath(const std::pair<int, int>& start, const std::pair<int, int>& goal);

private:
    int num_agents_;         // Number of agents
    int* large_gridmap_;      // Pointer to a grid map
    std::vector<std::vector<bool>> large_gridmap_; // 2D grid for obstacles

    // Helper function to check if a cell is within bounds and free
    bool isFree(int x, int y) const;
};

#endif // GLOBALPLANNER_H
