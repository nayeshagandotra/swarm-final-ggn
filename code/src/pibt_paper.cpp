#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <unordered_map>

struct Agent {
    int id;
    std::pair<int, int> position;
    std::pair<int, int> goal;
    std::vector<std::pair<int, int>> path;
};

class PIBT {
public:
    PIBT(const std::vector<std::vector<int>>& grid, std::vector<Agent>& agents)
        : grid_(grid), agents_(agents) {}

    void run() {
        undecided_ = agents_;
        while (!undecided_.empty()) {
            auto agent = get_highest_priority_agent();
            if (!pibt(agent, 0)) {
                std::cerr << "Failed to find a path for agent " << agent.id << std::endl;
            }
        }
        display_paths();
    }

private:
    std::vector<std::vector<int>> grid_;
    std::vector<Agent> agents_;
    std::vector<Agent> undecided_;
    std::vector<std::pair<int, int>> occupied_;

    bool is_valid_position(const std::pair<int, int>& position) {
        int x = position.first, y = position.second;
        return x >= 0 && y >= 0 && x < grid_.size() && y < grid_[0].size() && grid_[x][y] == 0;
    }

    double f(const Agent& agent, const std::pair<int, int>& position) {
        // Priority function (negative squared distance to the goal)
        return -std::pow(position.first - agent.goal.first, 2) - std::pow(position.second - agent.goal.second, 2);
    }

    Agent get_highest_priority_agent() {
        return *std::max_element(undecided_.begin(), undecided_.end(),
            [this](const Agent& a, const Agent& b) {
                return f(a, a.position) < f(b, b.position);
            });
    }

    std::vector<std::pair<int, int>> get_neighbors(const std::pair<int, int>& position) {
        std::vector<std::pair<int, int>> neighbors;
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}
        };
        for (const auto& dir : directions) {
            std::pair<int, int> neighbor = {position.first + dir.first, position.second + dir.second};
            if (is_valid_position(neighbor)) {
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    bool pibt(Agent& agent, int time) {
        std::vector<std::pair<int, int>> ci = get_neighbors(agent.position);
        ci.push_back(agent.position); // Allow waiting in place

        while (!ci.empty()) {
            auto vi_star = *std::max_element(ci.begin(), ci.end(),
                [&agent, this](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                    return f(agent, a) < f(agent, b);
                });

            if (is_occupied(vi_star, time + 1)) {
                ci.erase(std::remove(ci.begin(), ci.end(), vi_star), ci.end());
            } else {
                agent.path.push_back(vi_star);
                agent.position = vi_star;
                occupied_.push_back(vi_star);
                undecided_.erase(std::remove_if(undecided_.begin(), undecided_.end(),
                                                [&agent](const Agent& a) { return a.id == agent.id; }),
                                 undecided_.end());
                return true;
            }
        }

        return false;
    }

    bool is_occupied(const std::pair<int, int>& position, int time) {
        return std::find(occupied_.begin(), occupied_.end(), position) != occupied_.end();
    }

    void display_paths() {
        for (const auto& agent : agents_) {
            std::cout << "Agent " << agent.id << " path: ";
            for (const auto& step : agent.path) {
                std::cout << "(" << step.first << ", " << step.second << ") ";
            }
            std::cout << std::endl;
        }
    }
};

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
    };

    std::vector<Agent> agents = {
        {1, {0, 0}, {3, 3}, {}},
        {2, {0, 3}, {3, 0}, {}}
    };

    PIBT pibt(grid, agents);
    pibt.run();

    return 0;
}
