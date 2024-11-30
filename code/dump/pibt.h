#ifndef PIBT_H
#define PIBT_H

#include "../include/GlobalPlanner.h"
#include <math.h>
#include <vector>
#include <queue>
#include <chrono>
#include <iostream>  // For std::cout
#include <fstream>
#include <limits>
#include <thread>
#include <unordered_map>
#include "load_map.h"

class PIBT{
public:
    PIBT();
    ~PIBT();


    void initialize(const std::vector<std::vector<int>>& map, int numAgents);

    bool planPaths(std::vector<std::vector<int>>& agentPaths);

    std::vector<int> getAgentState(int agentID) const;

    void setGoal(int agentID, const std::vector<int>& goal);

    void displayPaths() const;

    static std::string toKey(int x, int y, int t); // Now static
    

private:
    struct Agent {
            int id;
            std::vector<int> position;
            std::vector<int> goal;
            std::vector<std::vector<int>> path;
        };
    // Method to plan a path from start to goal
    std::vector<std::vector<int>> map_; // Grid map for the environment
    std::vector<Agent> agents_;         // List of agents
    std::unordered_map<int, Agent> agentMap_; // Map for quick agent lookup

    // Helper methods
    bool isValidPosition(const std::vector<int>& position) const;
    double calculateHeuristic(const std::vector<int>& current, const std::vector<int>& goal) const;
    void updateAgentPath(Agent& agent);

    struct Compare {
        bool operator()(const std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>& a,
                        const std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>& b) const {
            return std::get<1>(a) > std::get<1>(b);
        }
    };

};


#endif