#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib> 
#include <string>  
#include <memory>
#include <unordered_map>
#define MAPS_DIR "maps"
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X))

struct Block{
    int x;
    int y;
    int obsCost;
    int* subMap;
    int size;
    Block(int size, int x, int y, int obsCost, int* subMap)
    : size(size),x(x),y(y),obsCost(obsCost),subMap(subMap)
    {}
};

std::unordered_map<int,std::shared_ptr<Block>> makeCoarseGraph(int* fineGraph, int x_size, int y_size, int swarm_size){
    int num_blocks_x = x_size / swarm_size;
    int num_blocks_y = y_size / swarm_size;
    std::unordered_map<int,std::shared_ptr<Block>> coarseMap;
     for (int block_y = 0; block_y < num_blocks_y; ++block_y) {
        for (int block_x = 0; block_x < num_blocks_x; ++block_x) {
            int* subMap = new int[swarm_size*swarm_size];
            int obsCost = 0;
            // Process the cells within the current block
            for (int i = 0; i < swarm_size; ++i) {
                for (int j = 0; j < swarm_size; ++j) {
                    int x = block_x * swarm_size + j; // Absolute x-coordinate
                    int y = block_y * swarm_size + i; // Absolute y-coordinate
                    if (x < x_size && y < y_size) {
                        obsCost += fineGraph[GETMAPINDEX(x,y,x_size,y_size)];
                        subMap[GETMAPINDEX(j,i,swarm_size,swarm_size)]=fineGraph[GETMAPINDEX(x,y,x_size,y_size)];
                    }
                }
            }
            std::shared_ptr<Block> curBlock = std::make_shared<Block>(swarm_size,block_x,block_y,obsCost,subMap);
            coarseMap.insert({GETMAPINDEX(block_x,block_y,num_blocks_x,num_blocks_y),curBlock});
        }
    }
    return coarseMap;
}


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