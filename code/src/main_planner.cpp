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
#include "../include/main_planner.h"
#include <math.h>


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

void planner(
    int* map,
    int x_size,
    int y_size
    )
{
    
    return;
}

void cost_maker(
    int* map,
    int x_size,
    int y_size,
    int swarm_size,
    int goal_index
    )
{

    // Create vector of shared_ptr Nodes
    auto nodemap = make_node_map(map, x_size, y_size);

    // create the GlobalPlanner instance
    GlobalPlanner planner(swarm_size, nodemap, x_size, y_size);

    // run a backward dijkstra search with distance as cost to get d heur
    planner.distBWDijkstra(nodemap[goal_index]);

    // run a second bw dj search with obscost
    planner.calculateRectSum();

    // print nodemap (distance only, change to loop through)
    print_node_map(nodemap, x_size, y_size, "node_map_costs.txt", "h", 1);


   
    return;
}

