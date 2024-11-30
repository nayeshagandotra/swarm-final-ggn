#include "pibt.h"

PIBT::PIBT() {
    // Constructor implementation
    std::cout << "PIBT planner initialized." << std::endl;
}

PIBT::~PIBT() {
    // Destructor implementation
    std::cout << "PIBT planner destroyed." << std::endl;
}

void PIBT::initialize(const std::vector<std::vector<int>>& map, int numAgents) {
    if (map.empty() || map[0].empty()) {
        std::cerr << "Error: Map cannot be empty." << std::endl;
        return;
    }

    map_ = map;
    agents_.clear();
    agentMap_.clear();

    for (int i = 0; i < numAgents; ++i) {
        Agent agent;
        agent.id = i;
        agent.position = {0, 0}; // Start at (0, 0) or customize based on your logic
        agent.goal = {static_cast<int>(map.size()) - 1, static_cast<int>(map[0].size()) - 1}; // Bottom-right corner
        agent.path.clear(); // Initialize path as an empty 2D vector
        agents_.push_back(agent);
        agentMap_[i] = agent;
    }

    std::cout << "Initialized with " << numAgents << " agents." << std::endl;
}



bool PIBT::planPaths(std::vector<std::vector<int>>& agentPaths) {
    agentPaths.clear(); // Ensure the output vector is empty before populating

    for (auto& agent : agents_) {
        updateAgentPath(agent);
        if (!agent.path.empty()) {
            agentPaths.push_back(agent.path.back()); // Add the last step of the path to agentPaths
        } else {
            std::cerr << "Error: Agent " << agent.id << " has no valid path." << std::endl;
            return false;
        }
    }

    return true;
}


std::vector<int> PIBT::getAgentState(int agentID) const {
    auto it = agentMap_.find(agentID);
    if (it != agentMap_.end()) {
        return it->second.position;
    }
    return {};
}

void PIBT::setGoal(int agentID, const std::vector<int>& goal) {
    auto it = agentMap_.find(agentID);
    if (it != agentMap_.end()) {
        it->second.goal = goal;
    }
}

void PIBT::displayPaths() const {
    for (const auto& agent : agents_) {
        std::cout << "Agent " << agent.id << ": ";
        for (const auto& step : agent.path) {
            std::cout << "(" << step[0] << ", " << step[1] << ") ";
        }
        std::cout << std::endl;
    }
}

bool PIBT::isValidPosition(const std::vector<int>& position) const {
    int x = position[0], y = position[1];
    return x >= 0 && y >= 0 && x < map_.size() && y < map_[0].size() && map_[x][y] == 0;
}

double PIBT::calculateHeuristic(const std::vector<int>& current, const std::vector<int>& goal) const {
    return std::sqrt(std::pow(current[0] - goal[0], 2) + std::pow(current[1] - goal[1], 2));
}


std::string PIBT::toKey(int x, int y, int t) {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(t);
}

// Custom comparator for the priority queue (can be refined for better heuristics)
struct Compare {
    bool operator()(const std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>& a,
                    const std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>& b) {
        return std::get<1>(a) > std::get<1>(b); // Compare based on time (lower is better)
    }
};



void PIBT::updateAgentPath(Agent& agent) {
    const int maxTime = 100; // Maximum timesteps for planning

    // 8 possible movements: right, down, left, up, and diagonals
    const int directions[8][2] = {
        {1, 0},   // Right
        {0, 1},   // Down
        {-1, 0},  // Left
        {0, -1},  // Up
        {1, 1},   // Down-Right
        {-1, 1},  // Down-Left
        {-1, -1}, // Up-Left
        {1, -1}   // Up-Right
    };

    // Priority Queue for BFS-like exploration (position, time, path-so-far)
    std::priority_queue<std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>,
                        std::vector<std::tuple<std::vector<int>, int, std::vector<std::vector<int>>>>,
                        Compare> pq;

    // Reserve time-space slots
    std::unordered_map<std::string, bool> reserved; // Key: "x,y,t"

    // Initialize queue with the agent's starting position
    pq.push({agent.position, 0, {agent.position}});

    while (!pq.empty()) {
        auto [currentPos, currentTime, currentPath] = pq.top();
        pq.pop();

        // Check if goal is reached
        if (currentPos == agent.goal) {
            agent.path = currentPath;
            agent.position = agent.goal;
            return;
        }

        // Explore neighbors in all 8 directions
        for (const auto& dir : directions) {
            int nx = currentPos[0] + dir[0];
            int ny = currentPos[1] + dir[1];
            int nextTime = currentTime + 1;

            // Check if the position is valid
            if (!isValidPosition({nx, ny}) || reserved[toKey(nx, ny, nextTime)]) {
                continue; // Skip invalid or reserved cells
            }

            // Reserve this cell for the agent at this time
            reserved[toKey(nx, ny, nextTime)] = true;

            // Add the neighbor to the priority queue
            auto newPath = currentPath;
            newPath.push_back({nx, ny});
            pq.push({{nx, ny}, nextTime, newPath});
        }

        // If no valid moves, allow waiting in place
        int waitTime = currentTime + 1;
        if (!reserved[toKey(currentPos[0], currentPos[1], waitTime)]) {
            reserved[toKey(currentPos[0], currentPos[1], waitTime)] = true;
            auto newPath = currentPath;
            newPath.push_back(currentPos);
            pq.push({currentPos, waitTime, newPath});
        }
    }

    // If no path is found
    std::cerr << "Error: Agent " << agent.id << " cannot find a path to the goal." << std::endl;
    agent.path = {};
}

// Helper method: Check if a position is valid on the map
bool PIBT::isValidPosition(const std::vector<int>& position) const {
    int x = position[0], y = position[1];
    return x >= 0 && y >= 0 && x < map_.size() && y < map_[0].size() && map_[x][y] == 0;
}




