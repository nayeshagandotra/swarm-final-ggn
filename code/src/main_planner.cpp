#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib> 
#include <string>  
#include <memory>
#include <unordered_map>
#define MAPS_DIR "maps"
#include "load_map.h"



int main(int argc, char *argv[])
{

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
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

}