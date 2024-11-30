#include "PIBT.h"
#include <algorithm>
#include <limits>

PIBT::PIBT(GlobalPlanner* global_planner, std::vector<int>& start_positions, std::vector<int>& goal_positions) : global_costplan(global_planner), start_positions(start_positions), goal_positions(goal_positions) {};

PIBT::~PIBT() {
    // Destructor implementation
    std::cout << "PIBT planner destroyed." << std::endl;
}

void PIBT::refresh_lists(){
    for (auto a : agents){
        occupied_now[a->cpy*global_costplan->x_size_ + a->cpx] = a;   //that index is now occupied 
        undecided.insert(a);
    }
}

void PIBT::initialize_pibt() {
    x_size_ = global_costplan->x_size_;
    y_size_ = global_costplan->y_size_;
    for (int i = 0; i < global_costplan->swarm_size_; i++) {
        auto start_node = global_costplan->nodemap_[start_positions[i]];     //pointer to the actual node
        auto goal_node = global_costplan->nodemap_[goal_positions[i]];      //pointer to the actual node
        
        // Create agent with priority set to euclidean distance
        auto agent = std::make_shared<Agent>(Agent{
            i,
            start_positions[i]%x_size_,
            start_positions[i]/x_size_,
            goal_positions[i]%x_size_,
            goal_positions[i]/x_size_,
            euclidean(start_node, goal_node)
        });
        
        agent->random_priority = dis(gen);
        agents.push_back(agent);
    }
    // refresh_lists();
}


void PIBT::print_agent_positions(const std::string& filename) {
    std::string output_dir = "code/output/";
    std::string full_filename = output_dir + filename;
    
    std::ofstream outFile(full_filename, std::ios::app);  // Open in append mode
    if (!outFile) {
        std::cerr << "Error opening file: " << full_filename << "\n";
        return;
    }

    // Write start positions
    outFile << "S" << "\n";
    for (size_t i = 0; i < agents.size(); i++) {
        int x = agents[i]->cpx;
        int y = agents[i]->cpy;
        outFile << x << "," << y;
        if (i < agents.size() - 1) outFile << ",";
    }
    outFile << "\n";

    // Write goal positions
    outFile << "G" << "\n";
    for (size_t i = 0; i < agents.size(); i++) {
        int x = agents[i]->gpx;
        int y = agents[i]->gpy;
        outFile << x << "," << y;
        if (i < agents.size() - 1) outFile << ",";
    }
    outFile << "\n";

    outFile.close();
}

std::vector<std::pair<int, int>> directions = {
        {0, 1},   // Right
        {1, 1},   // Down-Right
        {1, 0},   // Down
        {1, -1},  // Down-Left
        {0, -1},  // Left
        {-1, -1}, // Up-Left
        {-1, 0},  // Up
        {-1, 1},   // Up-Right
        {0, 0}    //stay in place
    };

int PIBT::getFormationScore(int vertext_id){
    // TODO
    return 0;
}




std::priority_queue<std::shared_ptr<Vertex>, 
        std::vector<std::shared_ptr<Vertex>>, 
        VertexComparator> PIBT::getSuccessors(std::shared_ptr<Agent> p) {
    std::priority_queue<std::shared_ptr<Vertex>, 
                       std::vector<std::shared_ptr<Vertex>>, 
                       VertexComparator> successors;

    for (const auto& dir : directions) {
        int newX = p->cpx + dir.first;
        int newY = p->cpy + dir.second;

        if (newX >= 0 && newX < x_size_ && newY >= 0 && newY < y_size_) {
            auto node = global_costplan->nodemap_[newY * x_size_ + newX];
            if (node->mapvalue < COLLISION_THRESH) {
                auto vertex = std::make_shared<Vertex>();
                vertex->idx = newY * x_size_ + newX;
                vertex->n = node;
                
                float w1 = 1.0;
                float w2 = 0.0;
                float w3 = 0.0;
                
                vertex->f = w1 * node->h[0] + 
                           w2 * node->h[1] + 
                           w3 * getFormationScore(vertex->idx);
                
                successors.push(vertex);
            }
        }
    }
    return successors;
}

void PIBT::plan_one_step(){
    // initializes the undecided pq with all agents (priority will be updated when?????)
    refresh_lists();

    while (!undecided.empty()) {
        auto a = *undecided.begin();  // Get highest priority agent
        funcPIBT(a);
    }

}

bool PIBT::funcPIBT(std::shared_ptr<Agent> ai, std::shared_ptr<Agent> aj){
    // this is the main PIBT function as described in the paper. To be called recursively as required.
    undecided.erase(ai);
    // get candidate next vertices
    auto C = getSuccessors(ai);  

    // loop through available vertices
    while (!C.empty()){   //each v will be a vertex
        auto vi_star = C.top();
        C.pop();

        // Skip if vertex is occupied by aj or in occupied maps
        if (aj && (vi_star->idx == (aj->cpy * x_size_ + aj->cpx))) continue;
        if (occupied_next.find(vi_star->idx) != occupied_next.end()) continue;

        // Remove from occupied_now before adding to occupied_next
        occupied_now.erase(ai->cpy * x_size_ + ai->cpx);
        // Add to occupied_next
        occupied_next[vi_star->idx] = ai;
        
        // Check if any undecided agent is at this vertex
        std::shared_ptr<Agent> ak = nullptr;
        auto conflict_it = std::find_if(undecided.begin(), undecided.end(),
        [&](const auto& agent) {
            return (agent->cpy * x_size_ + agent->cpx) == vi_star->idx;
        });

        if (conflict_it != undecided.end()) {
            ak = *conflict_it;
        }
            
        if (ak) {
            // Recursive call with priority inheritance
            if (funcPIBT(ak, ai)) {
                // Update agent position for next timestep
                ai->cpx = vi_star->idx % x_size_;
                ai->cpy = vi_star->idx / x_size_;
                ai->path.push_back({ai->cpx, ai->cpy});  // Record path
                return true;
            } else {
                // Put back in occupied_now since move failed
                occupied_now[ai->cpy * x_size_ + ai->cpx] = ai;
                // Remove from occupied_next and continue to next vertex
                occupied_next.erase(vi_star->idx);
                undecided.insert(ak);
                continue;
            }
        } else {
            // No conflict, update position and return valid
            ai->cpx = vi_star->idx % x_size_;
            ai->cpy = vi_star->idx / x_size_;
            ai->path.push_back({ai->cpx, ai->cpy});  // Record path
            ai->priority = 0;
            return true;
        }
    }
    // If no valid move found, stay in place
    occupied_next[ai->cpy * x_size_ + ai->cpx] = ai;
    ai->path.push_back({ai->cpx, ai->cpy});  // Record staying in place
    ai->priority += 1; //increase priority so this doesn't get stuck always
    return false;
}

bool PIBT::isComplete() {
    for (const auto& agent : agents) {
        if (agent->cpx != agent->gpx || agent->cpy != agent->gpy) {
            return false;
        }
    }
    return true;
}

bool PIBT::runPIBT(){
    // give it a timeout
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::seconds(10);  // 60 second timeout

    while (!isComplete()){

        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout_duration) {
            return false;  // Timeout reached
        }
        print_agent_positions("node_map_costs.txt");
        plan_one_step(); 
    }
    return true;  // Successfully completed

}