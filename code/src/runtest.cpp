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
#include "../include/MapMakerFine.h"
#include "../include/GlobalPlanner.h"

#ifndef OUTPUT_DIR
#define OUTPUT_DIR "output"
#endif

int main(int argc, char *argv[]) {
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
    if (letter != 'N') {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> x_size >> letter >> y_size;
    std::cout << "map size: " << x_size << letter << y_size << std::endl;

    // Read start positions
    myfile >> letter;
    if (letter != 'S') {
        std::cout << "error parsing start positions" << std::endl;
        return -1;
    }

    std::vector<int> start_positions;
    std::getline(myfile, line); // consume the newline
    std::getline(myfile, line);
    std::stringstream ss(line);
    std::string value;

    while (std::getline(ss, value, ',')) {
        int x = std::stoi(value);
        std::getline(ss, value, ',');
        int y = std::stoi(value);
        start_positions.push_back(y*x_size + x);
    }

    // Read goal positions
    myfile >> letter;
    if (letter != 'G') {
        std::cout << "error parsing goal positions" << std::endl;
        return -1;
    }

    std::vector<int> goal_positions;
    std::getline(myfile, line); // consume the newline
    std::getline(myfile, line);
    std::stringstream ss_goal(line);

    while (std::getline(ss_goal, value, ',')) {
        int x = std::stoi(value);
        std::getline(ss_goal, value, ',');
        int y = std::stoi(value);
        goal_positions.push_back(y*x_size + x);
    }

    // Read map
    myfile >> letter;
    if (letter != 'M') {
        std::cout << "error parsing map" << std::endl;
        return -1;
    }

    // read map - corrected version
    int* map = new int[x_size*y_size];
    std::getline(myfile, line); // consume the newline
    
    for (size_t j = 0; j < y_size; j++) {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t i = 0; i < x_size; i++) {
            double valued;
            if (i < x_size-1) {
                std::string val;
                std::getline(ss, val, ',');
                valued = std::stod(val);
            } else {
                ss >> value;
            }
            map[j*x_size + i] = (int)valued;
        }
    }
    
    // call cost_maker/ planner
    planner(map, x_size, y_size, swarm_size, start_positions, goal_positions);
    
    myfile.close();
    delete[] map;
    return 0;
}