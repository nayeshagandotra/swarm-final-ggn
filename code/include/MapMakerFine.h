// mapmakerfine.h
#ifndef MAPMAKERFINE_H
#define MAPMAKERFINE_H

#include <limits>
#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <array>

struct Node {
    int x = 0;
    int y = 0;
    int mapvalue = 0;
    std::array<float, 3> h = {std::numeric_limits<float>::max() - 1,
                           0,
                           0};  // [hd, hobs, hform]
    std::array<bool, 3> closed_dj = {false, false, false};  // Fixed size array of 3 booleans
};

// Using vector of shared_ptr to Node
using NodeMap = std::shared_ptr<Node>*;

NodeMap make_node_map(int* map, int x_size, int y_size);
void print_node_map(const NodeMap& nodeMap, int x_size, int y_size, 
                   const std::string& filename, std::string value_type, int h_index = 0);
#endif