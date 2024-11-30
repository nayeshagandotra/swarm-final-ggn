// globalplanner.h
#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

#include "../include/MapMakerFine.h"
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

#define COLLISION_THRESH 0.5

class GlobalPlanner {
public:
    // Constructor and destructor
    GlobalPlanner(int num_agents, NodeMap nodemap, int x_size, int y_size);
    ~GlobalPlanner();

    // Planning methods
    void distBWDijkstra(std::shared_ptr<Node> goal);
    void calculateRectSum();
    
    int swarm_size_;
    NodeMap nodemap_;
    int x_size_;
    int y_size_;

    // Helper methods
    bool isFree(std::shared_ptr<Node> p) const;
    std::vector<std::shared_ptr<Node>> getSuccessors(std::shared_ptr<Node> p);
};

// Helper functions
int euclidean(std::shared_ptr<Node> p1, std::shared_ptr<Node> p2);
int manhattan(std::shared_ptr<Node> p1, std::shared_ptr<Node> p2);

#endif