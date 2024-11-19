/*=================================================================
 *
 * runtest.cpp
 *
 *=================================================================*/
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

#include "../include/main_planner.h"
#include "../include/load_map.h"
#include "../include/GlobalPlanner.h"

#ifndef OUTPUT_DIR
#define OUTPUT_DIR "output"
#endif

int main(int argc, char *argv[])
{
    std::string mapFilePath = argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;

    int swarm_size = std::stoi(argv[2]);

    std::ifstream myfile;
    myfile.open(mapFilePath);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file:" << mapFilePath << std::endl;
        return -1;
    }
    // read map size
    char letter;
    std::string line;
    int x_size, y_size;

    myfile >> letter;
    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> y_size >> letter >> x_size;
    std:: cout << "map size: " << x_size << letter << y_size << std::endl;
    // read map
    int* map = new int[x_size*y_size];
    for (size_t i=0; i<x_size; i++)
    {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t j=0; j<y_size; j++)
        {
            double value;
            ss >> value;

            map[j*x_size+i] = (int) value;
            if (j != x_size-1) ss.ignore();
        }
    }
    std::unordered_map<int,std::shared_ptr<Block>> c_map = makeCoarseGraph(map, x_size, y_size, swarm_size);

    myfile.close();
    std::cout << "\nRunning planner" << std::endl;

    // After creating c_map, instantiate the planner
    GlobalPlanner planner(swarm_size, c_map, x_size, y_size);

    // Create start and goal blocks
    std::shared_ptr<Block> start = c_map[GETMAPINDEX(0, 0, x_size, y_size)];
    std::shared_ptr<Block> goal = c_map[GETMAPINDEX(10, 1, x_size, y_size)];

    // Get the path
    std::vector<std::pair<int, int>> path = planner.planPath(start, goal);

    return 0;
}