#ifndef PLANNER_H
#define PLANNER_H

int calculateCentroidIndex(const std::vector<int>& goal_positions, int x_size);

void planner(
    int* map,
    int x_size,
    int y_size,
    int swarm_size,
    std::vector<int> start_positions,
    std::vector<int> goal_positions
    );

#endif // PLANNER_H