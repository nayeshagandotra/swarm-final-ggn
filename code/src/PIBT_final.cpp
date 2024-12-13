#include "pibt.h"
#include <algorithm>
#include <limits>
// #include "../include/eigen/Eigen/Dense"
#include <chrono>

PIBT::PIBT(GlobalPlanner* global_planner, std::vector<int>& start_positions, std::vector<int>& goal_positions) : global_costplan(global_planner), start_positions(start_positions), goal_positions(goal_positions) {};

PIBT::~PIBT() {
    // Destructor implementation
    std::cout << "PIBT planner destroyed." << std::endl;
}

void PIBT::refresh_lists(){
    bool fform = false;
    for (auto a : agents){
        occupied_now[a->cpy*global_costplan->x_size_ + a->cpx] = a;   //that index is now occupied
        if (global_costplan->nodemap_[a->cpy*x_size_ + a->cpx]->h[1] > 0){
            fform = true;
        } 
    }
    occupied_next.clear();
    update_weights(fform);
    if (w3 != 0){
        getAllFormationScore(); //update afs for all agents
    }
    for (auto a : agents){
        update_agent_priority(a);
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
            goal_positions[i]/x_size_});
        
        agent->random_priority = dis(gen);
        agents.push_back(agent);
    }
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

// Eigen::MatrixXd translateMatrixToOrigin(const Eigen::MatrixXd& A) {
//     Eigen::VectorXd offset = A.rowwise().mean(); // Calculate the centroid
//     return A.colwise() - offset;         // Subtract the centroid from each point
// }

// double computeRMSE(const Eigen::MatrixXd& P_transformed, const Eigen::MatrixXd& Q) {
//     Eigen::MatrixXd diff = P_transformed - Q;
//     double mse = diff.squaredNorm() / Q.cols(); // Mean Squared Error
//     return std::sqrt(mse); // Root Mean Squared Error
// }

// double PIBT::getFormationScore(std::shared_ptr<Agent> p,std::shared_ptr<Vertex> vertex){
//     std::pair<int,int> curDir = {vertex->n->x - p->cpx,vertex->n->y - p->cpy};
//     // Create a 2xN matrix
//     int swarm_size = global_costplan->swarm_size_;
//     Eigen::MatrixXd currentPositions(2, swarm_size);
//     Eigen::MatrixXd goalPositions(2,swarm_size);
//     int i = 0;
//     for(auto agent : occupied_next){
//         currentPositions(0, i) = agent.second->cpx; // x-coordinate
//         currentPositions(1, i) = agent.second->cpy; // y-coordinate
//         goalPositions(0, i) = agent.second->gpx; // x-coordinate
//         goalPositions(1, i) = agent.second->gpy; // y-coordinate  
//         i++;
//     }
//     for(auto agent : occupied_now){
//         if(agent.second->id==p->id){
//             currentPositions(0, i) = vertex->n->x; // x-coordinate
//             currentPositions(1, i) = vertex->n->y; // y-coordinate
//             goalPositions(0, i) = agent.second->gpx; // x-coordinate
//             goalPositions(1, i) = agent.second->gpy; // y-coordinate  
//             i++;
//             continue;
//         }
//         currentPositions(0, i) = agent.second->cpx+curDir.first; // x-coordinate
//         currentPositions(1, i) = agent.second->cpy+curDir.second; // y-coordinate
//         goalPositions(0, i) = agent.second->gpx; // x-coordinate
//         goalPositions(1, i) = agent.second->gpy; // y-coordinate
//         i++;
//     }
//     // Translate both matrices to the origin
//     Eigen::MatrixXd current_bar = translateMatrixToOrigin(currentPositions);
//     Eigen::MatrixXd goal_bar = translateMatrixToOrigin(goalPositions);

//     // Compute the cross-covariance matrix S
//     Eigen::MatrixXd S = current_bar * goal_bar.transpose();

//     // Singular Value Decomposition (SVD)
//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
//     Eigen::MatrixXd U = svd.matrixU();
//     Eigen::MatrixXd V = svd.matrixV();

//     // Compute R_svd
//     Eigen::Matrix2d VU_T = V * U.transpose();
//     double d = VU_T.determinant();
//     Eigen::Matrix2d diagonalMatrix = Eigen::Matrix2d::Identity();
//     diagonalMatrix(1, 1) = d; // Ensure proper orientation
//     Eigen::Matrix2d R_svd = V * diagonalMatrix * U.transpose();

//     // Compute t_svd
//     Eigen::Vector2d t_svd = goalPositions.rowwise().mean() - R_svd * currentPositions.rowwise().mean();

//     // Transform currentPositions using the optimal R and t
//     Eigen::MatrixXd currentTransformed = R_svd * currentPositions;
//     currentTransformed.colwise() += t_svd;

//     // Compute RMSE
//     double rmse = computeRMSE(currentTransformed, goalPositions);
//     //std::cout << "RMSE between transformed currentPositions and goalPositions: " << rmse << std::endl;
//     return rmse;

// }

Eigen::MatrixXd PIBT::translateMatrixToOrigin(const Eigen::MatrixXd& A) {
    Eigen::VectorXd offset = A.rowwise().mean();
    return A.colwise() - offset;
}

Eigen::VectorXd PIBT::computeResiduals(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                                       const Eigen::Matrix2d& R, const Eigen::Vector2d& t) {
    Eigen::MatrixXd P_transformed = R * P;
    P_transformed.colwise() += t;
    Eigen::MatrixXd diff = P_transformed - Q;
    return diff.colwise().lpNorm<1>();  // L1 norm for each point
}

double PIBT::computeMADThreshold(const Eigen::VectorXd& residuals, double k) {
    Eigen::VectorXd sorted_residuals = residuals;
    std::sort(sorted_residuals.data(), sorted_residuals.data() + sorted_residuals.size());
    double median = sorted_residuals(sorted_residuals.size() / 2);
    
    Eigen::VectorXd abs_deviations = (residuals.array() - median).abs().matrix();
    std::sort(abs_deviations.data(), abs_deviations.data() + abs_deviations.size());
    
    double mad = abs_deviations(abs_deviations.size() / 2);
    return k * mad / 0.6745;
}

double PIBT::getAllFormationScore(){
    int swarm_size = global_costplan->swarm_size_;
    Eigen::MatrixXd currentPositions(2, swarm_size);
    Eigen::MatrixXd goalPositions(2, swarm_size);

    int i = 0;
    for (auto agent : occupied_now) {
        currentPositions(0, i) = agent.second->cpx;
        currentPositions(1, i) = agent.second->cpy;
        goalPositions(0, i) = agent.second->gpx;
        goalPositions(1, i) = agent.second->gpy;
        i++;
    }

    // Translate both matrices to the origin
    Eigen::MatrixXd current_bar = translateMatrixToOrigin(currentPositions);
    Eigen::MatrixXd goal_bar = translateMatrixToOrigin(goalPositions);

    // Compute the cross-covariance matrix S
    Eigen::MatrixXd S = current_bar * goal_bar.transpose();

    // Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Compute R_svd
    Eigen::Matrix2d VU_T = V * U.transpose();
    double d = VU_T.determinant();
    Eigen::Matrix2d diagonalMatrix = Eigen::Matrix2d::Identity();
    diagonalMatrix(1, 1) = d;
    Eigen::Matrix2d R_svd = V * diagonalMatrix * U.transpose();

    // Compute t_svd
    Eigen::Vector2d t_svd = goalPositions.rowwise().mean() - R_svd * currentPositions.rowwise().mean();

    // Compute residuals using L1 norm
    Eigen::VectorXd residuals = computeResiduals(currentPositions, goalPositions, R_svd, t_svd);

    // Compute MAD threshold
    double threshold = computeMADThreshold(residuals);

    i = 0;
    // update the agent afs value (to feed into priority later)
    for (auto agent : occupied_now) {
        agent.second->afs = residuals(i);
        if (residuals(i) > threshold){
            // we want to get back into formation!
            agent.second->aw3 = global_costplan->swarm_size_;
        }
        else{ 
            agent.second->aw3 = 1.0;
        }
        i++;
    }
}

double PIBT::getFormationScore(std::shared_ptr<Agent> p, std::shared_ptr<Vertex> vertex) {
    std::pair<int,int> curDir = {vertex->n->x - p->cpx, vertex->n->y - p->cpy};
    int swarm_size = global_costplan->swarm_size_;
    Eigen::MatrixXd currentPositions(2, swarm_size);
    Eigen::MatrixXd goalPositions(2, swarm_size);

    int i = 0;
    int agent_id = 0;
    for (auto agent : occupied_next) {
        currentPositions(0, i) = agent.second->cpx;
        currentPositions(1, i) = agent.second->cpy;
        goalPositions(0, i) = agent.second->gpx;
        goalPositions(1, i) = agent.second->gpy;
        i++;
    }
    for (auto agent : occupied_now) {
        if (agent.second->id == p->id) {
            currentPositions(0, i) = vertex->n->x;
            currentPositions(1, i) = vertex->n->y;
            agent_id = i;
        } else {
            currentPositions(0, i) = agent.second->cpx + curDir.first;
            currentPositions(1, i) = agent.second->cpy + curDir.second;
        }
        goalPositions(0, i) = agent.second->gpx;
        goalPositions(1, i) = agent.second->gpy;
        i++;
    }

    // Translate both matrices to the origin
    Eigen::MatrixXd current_bar = translateMatrixToOrigin(currentPositions);
    Eigen::MatrixXd goal_bar = translateMatrixToOrigin(goalPositions);

    // Compute the cross-covariance matrix S
    Eigen::MatrixXd S = current_bar * goal_bar.transpose();

    // Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Compute R_svd
    Eigen::Matrix2d VU_T = V * U.transpose();
    double d = VU_T.determinant();
    Eigen::Matrix2d diagonalMatrix = Eigen::Matrix2d::Identity();
    diagonalMatrix(1, 1) = d;
    Eigen::Matrix2d R_svd = V * diagonalMatrix * U.transpose();

    // Compute t_svd
    Eigen::Vector2d t_svd = goalPositions.rowwise().mean() - R_svd * currentPositions.rowwise().mean();

    // Compute residuals using L1 norm
    Eigen::VectorXd residuals = computeResiduals(currentPositions, goalPositions, R_svd, t_svd);

    // // Compute MAD threshold
    // double threshold = computeMADThreshold(residuals);

    // // Count outliers
    // int outlier_count = (residuals.array() > threshold).count();

    return residuals(agent_id);  // just the residual value of that agent
}



std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, VertexComparator> PIBT::getSuccessors(std::shared_ptr<Agent> p)
 {
    std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, VertexComparator> successors;

    for (const auto& dir : directions) {
        int newX = p->cpx + dir.first;
        int newY = p->cpy + dir.second;

        if (newX >= 0 && newX < x_size_ && newY >= 0 && newY < y_size_) {
            auto node = global_costplan->nodemap_[newY * x_size_ + newX];
            if (node->mapvalue < COLLISION_THRESH) {
                auto vertex = std::make_shared<Vertex>();
                vertex->idx = newY * x_size_ + newX;
                vertex->n = node;


                float manhattan = abs(p->cpx - p->gpx) + abs(p->cpy - p->gpy);
          
                float w2 = 0;
                float w4 = (manhattan <= 2*distance_thresh) ? abs(newX - p->gpx) + abs(newY - p->gpy) : -1.0;
                float w1 = (w4 != -1.0) ? 0.0 : 1.0;
                if (w4 == -1.0){w4 = 0;}

                double fs = 0.0;

                if (w3 != 0){
                    fs = getFormationScore(p, vertex);
                }

                vertex->f = w1 * node->h[0] + 
                           w2 * node->h[1] + 
                           w3 * fs + 
                           w4;
                vertex->manh = abs(newX - p->gpx) + abs(newY - p->gpy);
                successors.push(vertex);
            }
        }
    }
    return successors;
}

void PIBT::update_agent_priority(std::shared_ptr<Agent> ai){
    // ai->priority = abs(ai->cpx - ai->gpx) + abs(ai->cpy - ai->gpy)  ;  
    ai->priority = global_costplan->nodemap_[ai->cpy*x_size_ + ai->cpx]->h[0] - 10*ai->afs;
}

void PIBT::update_weights(bool fform){
    if (fform){
        w3 = 0.0;
    }
    else{
        w3 = 12.0;
    }
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
    auto it = std::find_if(undecided.begin(), undecided.end(),
    [ai](const std::shared_ptr<Agent>& a) {
        return ai->priority == a->priority && 
               ai->random_priority == a->random_priority;
    });
    if (it != undecided.end()) {
        undecided.erase(it);
    }
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
                ai->random_priority = dis(gen) * ai->priority;
                return true;
            } else {
                // Put back in occupied_now since move failed
                occupied_now[ai->cpy * x_size_ + ai->cpx] = ai;
                // Remove from occupied_next and continue to next vertex
                occupied_next.erase(vi_star->idx);
                // undecided.insert(ak);
                continue;
            }
        } else {
            // No conflict, update position and return valid
            ai->cpx = vi_star->idx % x_size_;
            ai->cpy = vi_star->idx / x_size_;
            ai->path.push_back({ai->cpx, ai->cpy});  // Record path
            return true;
        }
    }
    // If no valid move found, stay in place
    occupied_next[ai->cpy * x_size_ + ai->cpx] = ai;
    ai->path.push_back({ai->cpx, ai->cpy});  // Record staying in place
    // ai->priority += 15; //increase priority so this doesn't get stuck always
    ai->random_priority = dis(gen) * ai->priority;
    return false;
}

bool PIBT::isComplete() {
    bool all_agents = true;
    for (const auto& agent : agents) {
        all_agents = all_agents && (agent->cpx == agent->gpx && agent->cpy == agent->gpy);
        if (!all_agents){
            return false;
        }
    }
    return all_agents;
}

bool PIBT::runPIBT(){
    // give it a timeout
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::seconds(10);  // 60 second timeout
    print_agent_positions("node_map_costs.txt");
    while (!isComplete()){

        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time > timeout_duration) {
             return false;  // Timeout reached
        }
        plan_one_step(); 
        print_agent_positions("node_map_costs.txt");
    }
    return true;  // Successfully completed

}