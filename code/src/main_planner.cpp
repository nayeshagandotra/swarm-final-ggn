#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib> 
#include <string>  
#include <memory>
#include <unordered_map>
#define MAPS_DIR "maps"
#include "../include/MapMakerFine.h"
#include "../include/GlobalPlanner.h"
#include "../include/PIBT.h"
#include "../include/main_planner.h"
#include <math.h>


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

int calculateCentroidIndex(const std::vector<int>& goal_positions, int x_size) {
    int sum_x = 0;
    int sum_y = 0;
    int n = goal_positions.size();
    
    // Convert indices to x,y coordinates and sum
    for (int pos : goal_positions) {
        int y = pos / x_size;
        int x = pos % x_size;
        sum_x += x;
        sum_y += y;
    }
    
    // Calculate average x,y coordinates
    int centroid_x = sum_x / n;
    int centroid_y = sum_y / n;
    
    // Convert back to index
    return centroid_y * x_size + centroid_x;
}


void planner(
    int* map,
    int x_size,
    int y_size,
    int swarm_size,
    std::vector<int> start_positions,
    std::vector<int> goal_positions
    )
{

    // calculate the centroid of goal_positions (goal_index)
    int goal_index = calculateCentroidIndex(goal_positions, x_size);

    // Create vector of shared_ptr Nodes
    auto nodemap = make_node_map(map, x_size, y_size);

    // create the GlobalPlanner instance-> this helps us manage graph level things
    GlobalPlanner planner(swarm_size, nodemap, x_size, y_size);

    // Create PIBT instance using the planner- this helps us plan each indiv agent
    PIBT pibt(&planner, start_positions, goal_positions);

    // Initialize PIBT with the agents
    pibt.initialize_pibt();

    // run a backward dijkstra search with distance as cost to get d heur
    planner.distBWDijkstra(nodemap[goal_index]);

    // run a second bw dj search with obscost
    planner.calculateRectSum();

    // print nodemap (distance only, change to loop through)
    print_node_map(nodemap, x_size, y_size, "node_map_costs.txt", "h", 0);

    // run pibt
    bool success = pibt.runPIBT();
   
    return;
}

