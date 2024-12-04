#ifndef PIBT_H
#define PIBT_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <random>
#include "../include/eigen/Eigen/Dense"
#include "../include/eigen/Eigen/SVD"
#include "../include/GlobalPlanner.h"
#include "../include/MapMakerFine.h"

static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_real_distribution<float> dis(0.0f, 2.0f);

struct Agent {
    int id;
    int cpx;   //current pos of that agent
    int cpy;
    int gpx;
    int gpy;      //this is the goal node for that agent
    float afs;    //agent specific formation score, to be used to find deviants
    float aw3;    //how much that agent cares about coming back into formation
    int priority;
    float random_priority;
    std::vector<std::vector<int>> path;
};

struct AgentPriorityComparator {
    bool operator()(const std::shared_ptr<Agent>& a1, const std::shared_ptr<Agent>& a2) const {
        // Compare priorities
        if (a1->priority != a2->priority) {
            return a1->priority < a2->priority;
        }
        // Use random_priority as tie-breaker
        return a1->random_priority < a2->random_priority;
    }
};

struct Vertex {
    int idx = 0;
    std::shared_ptr<Node> n = nullptr;
    int f = 0;
    int manh = 0;
};

struct VertexComparator {
    bool operator()(const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2) const {
        if (v1->f != v2->f) {
            return v1->f > v2->f;  // Lower f values have higher priority
        }
        return v1->manh > v2->manh;  // If f values are equal, lower manhattan values have higher priority
    }
};


class PIBT {
    public:
    GlobalPlanner* global_costplan;
    int x_size_;
    int y_size_;
    float distance_thresh = global_costplan->swarm_size_;
    float w3;
    std::vector<int> start_positions;
    std::vector<int> goal_positions;
    std::vector<std::shared_ptr<Agent>> agents;
    std::unordered_map<int, std::shared_ptr<Agent>> occupied_now;
    std::unordered_map<int, std::shared_ptr<Agent>> occupied_next;
    std::set<std::shared_ptr<Agent>, AgentPriorityComparator> undecided;

    PIBT(GlobalPlanner* global_planner, std::vector<int>& start_positions, std::vector<int>& goal_positions);
    ~PIBT();
    void refresh_lists();
    void initialize_pibt();
    void print_agent_positions(const std::string& filename);

    double getFormationScore(std::shared_ptr<Agent> p,std::shared_ptr<Vertex> vertex);
    double getAllFormationScore();
    Eigen::MatrixXd translateMatrixToOrigin(const Eigen::MatrixXd& A);
    Eigen::VectorXd computeResiduals(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                                     const Eigen::Matrix2d& R, const Eigen::Vector2d& t);
    double computeMADThreshold(const Eigen::VectorXd& residuals, double k = 1.1);

    std::priority_queue<std::shared_ptr<Vertex>, 
                   std::vector<std::shared_ptr<Vertex>>, 
                   VertexComparator> getSuccessors(std::shared_ptr<Agent> p);
    void update_agent_priority(std::shared_ptr<Agent> ai);
    void update_weights(bool fform);
    void plan_one_step();
    bool funcPIBT(std::shared_ptr<Agent> ai, std::shared_ptr<Agent> aj = nullptr);
    bool isComplete();
    bool runPIBT();
};

#endif