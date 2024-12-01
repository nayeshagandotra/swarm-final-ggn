// globalplanner.cpp
#include "../include/GlobalPlanner.h"


GlobalPlanner::GlobalPlanner(int num_agents, NodeMap nodemap, 
                           int x_size, int y_size)
    : swarm_size_(num_agents), nodemap_(nodemap), x_size_(x_size), 
    y_size_(y_size) {}

GlobalPlanner::~GlobalPlanner() {}

bool GlobalPlanner::isFree(std::shared_ptr<Node> p) const {
    return p->mapvalue < COLLISION_THRESH;
}

int euclidean(std::shared_ptr<Node> p1, std::shared_ptr<Node> p2) {
    return (p1->x - p2->x)*(p1->x - p2->x) + 
                (p1->y - p2->y)*(p1->y - p2->y);
}

int manhattan(std::shared_ptr<Node> p1, std::shared_ptr<Node> p2) {
    return abs((p1->x - p2->x)) + abs((p1->y - p2->y));
}

struct NodeComparator {
    NodeComparator(int n) : n_(n) {}
    
    bool operator()(const std::shared_ptr<Node> s1, 
                   const std::shared_ptr<Node> s2) const {
        return (s1->h[n_]) > (s2->h[n_]);
    }
    int n_;
};

std::vector<std::shared_ptr<Node>> GlobalPlanner::getSuccessors(std::shared_ptr<Node> p) {
    std::vector<std::shared_ptr<Node>> successors;
    std::vector<std::pair<int, int>> directions = {
    {0, 1},   // Right
    {1, 1},   // Down-Right
    {1, 0},   // Down
    {1, -1},  // Down-Left
    {0, -1},  // Left
    {-1, -1}, // Up-Left
    {-1, 0},  // Up
    {-1, 1}   // Up-Right
    };

    for (const auto& dir : directions) {
        int newX = p->x + dir.first;
        int newY = p->y + dir.second;

        if (newX >= 0 && newX < x_size_ && newY >= 0 && newY < y_size_) {
            successors.push_back(nodemap_[newY * x_size_ + newX]);
        }
    }
    return successors;
}

void GlobalPlanner::distBWDijkstra(std::shared_ptr<Node> goal) {
    // pq for open, based on desired h value
    std::priority_queue<std::shared_ptr<Node>,
                   std::vector<std::shared_ptr<Node>>,
                   NodeComparator> open(NodeComparator(0));  // Use h[0] for comparison
    
    goal->h[0] = 0;
    open.push(goal);

    while (!open.empty()) {
        auto current = open.top();
        current->closed_dj[0] = true;
        open.pop();

        for (const auto& successor : getSuccessors(current)) {
            if (isFree(successor) && !successor->closed_dj[0]) {
                float new_g = current->h[0] + 1;    //assume cost = 1 for each movement
                if (successor->h[0] > new_g) {
                    successor->h[0] = new_g;
                    open.push(successor);
                }
            }
        }
    }
    return;
}

void GlobalPlanner::calculateRectSum() {
    float out_of_bounds_value = x_size_ * y_size_;
    int rect_width = swarm_size_;
    int rect_height = swarm_size_/2 + 1;
    
    for (int y = 0; y < y_size_; y++) {
        for (int x = 0; x < x_size_; x++) {
            auto current = nodemap_[y*x_size_ + x];
            float sum = 0;
            
            // Calculate bounds for the rectangle
            int x_start = x - rect_width/2;
            int x_end = x + rect_width/2;
            int y_start = y - rect_height/2;
            int y_end = y + rect_height/2;
            
            // Sum all mapvalues within the rectangle
            for (int sy = y_start; sy <= y_end; sy++) {
                for (int sx = x_start; sx <= x_end; sx++) {
                    if (sx >= 0 && sx < x_size_ && sy >= 0 && sy < y_size_) {
                        sum += nodemap_[sy*x_size_ + sx]->mapvalue;
                    } else {
                        sum += out_of_bounds_value;
                    }
                }
            }
            
            current->h[1] = sum/(rect_width*rect_height);
        }
    }
}