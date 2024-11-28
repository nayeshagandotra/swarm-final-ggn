std::unordered_map<int,std::shared_ptr<Block>> c_map = makeCoarseGraph(map, x_size, y_size, swarm_size);

std::cout << "\nRunning planner" << std::endl;

// After creating c_map, instantiate the planner
GlobalPlanner planner(swarm_size, c_map, x_size/swarm_size, y_size/swarm_size);

// Create start and goal blocks
std::shared_ptr<Block> start = c_map[GETMAPINDEX(0, 0, x_size, y_size)];
std::shared_ptr<Block> goal = c_map[GETMAPINDEX(10, 1, x_size, y_size)];

// Get the path
std::vector<std::pair<int, int>> path = planner.planPath(start, goal);