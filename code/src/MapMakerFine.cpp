#include "../include/MapMakerFine.h"

NodeMap make_node_map(int* map, int x_size, int y_size) {
    auto nodeMap = new std::shared_ptr<Node>[x_size * y_size];
    
    // Fill array with 1-based indexing to match GETMAPINDEX
    for (size_t i = 1; i <= x_size; i++) {
        for (size_t j = 1; j <= y_size; j++) {
            int idx = GETMAPINDEX(i, j, x_size, y_size);
            nodeMap[idx] = std::make_shared<Node>();
            nodeMap[idx]->x = i-1;  // Store 0-based coordinates
            nodeMap[idx]->y = j-1;
            // Convert from 1-based to 0-based for map access
            nodeMap[idx]->mapvalue = map[((j-1)*x_size + (i-1))];
        }
    }
    return nodeMap;
}

void print_node_map(const NodeMap& nodeMap, int x_size, int y_size,
                   const std::string& filename, std::string value_type, int h_index) {
    std::string output_dir = "code/output/";
    std::string full_filename = output_dir + filename;
    
    std::ofstream outFile(full_filename);
    if (!outFile) {
        std::cerr << "Error opening file: " << full_filename << "\n";
        return;
    }

    outFile.precision(1);
    outFile << std::fixed;
    outFile << "N" << "\n";
    outFile << x_size << "," << y_size << "\n";

    // Print using 1-based indexing to match GETMAPINDEX
    for (int j = 1; j <= y_size; j++) {
        for (int i = 1; i <= x_size; i++) {
            auto node = nodeMap[GETMAPINDEX(i, j, x_size, y_size)];
            
            if (value_type == "h") {
                outFile << static_cast<float>(node->h[h_index]);
            } else if (value_type == "map") {
                outFile << static_cast<float>(node->mapvalue);
            }
            
            if (i < x_size) outFile << ",";
        }
        outFile << "\n";
    }
    outFile.close();
}