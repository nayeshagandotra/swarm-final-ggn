std::unordered_map<int,std::shared_ptr<Block>> c_map = makeCoarseGraph(map, x_size, y_size, swarm_size);

std::cout << "\nRunning planner" << std::endl;

// After creating c_map, instantiate the planner
GlobalPlanner planner(swarm_size, c_map, x_size/swarm_size, y_size/swarm_size);

// Create start and goal blocks
std::shared_ptr<Block> start = c_map[GETMAPINDEX(0, 0, x_size, y_size)];
std::shared_ptr<Block> goal = c_map[GETMAPINDEX(10, 1, x_size, y_size)];

// Get the path
std::vector<std::pair<int, int>> path = planner.planPath(start, goal);

    void updatePriorities();

void PIBT::updatePriorities() {
    for (auto& agent : agents) {
        // Priority decreases with time and increases with distance to goal
        agent.priority = agent.priority * 0.99 + 
            (agent.current_pos == agent.goal_pos ? 0 : 1);
    }
}


// After reading start positions, add:

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
std::string value;

while (std::getline(ss_goal, value, ',')) {
    int x = std::stoi(value);
    std::getline(ss_goal, value, ',');
    int y = std::stoi(value);
    goal_positions.push_back(GETMAPINDEX(x, y, x_size, y_size));
}

// Read map
myfile >> letter;
if (letter != 'M') {
    std::cout << "error parsing map" << std::endl;
    return -1;
}

// Then continue with existing map reading code:
int* map = new int[x_size*y_size];
std::getline(myfile, line); // consume the newline