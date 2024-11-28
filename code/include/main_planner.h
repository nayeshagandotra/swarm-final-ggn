#ifndef PLANNER_H
#define PLANNER_H

// Declare the plan function
void planner(
    int* map,
    int x_size,
    int y_size
    );

void cost_maker(
    int* map,
    int x_size,
    int y_size,
    int swarm_size,
    int goal_index
    );

#endif // PLANNER_H