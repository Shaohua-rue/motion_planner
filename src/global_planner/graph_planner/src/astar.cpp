#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>
#include "matplotlibcpp.h"
#include <fmt/core.h>
namespace plt = matplotlibcpp;

#include "GraphSearchPlanner.hpp"

using std::shared_ptr;
using std::unordered_map;
using std::vector;
constexpr bool show_animation = false;

class AStarPlanner : public GraphSearchPlanner {
private:
    shared_ptr<Node> get_mincost_node(const unordered_map<double, shared_ptr<Node>>& node_set,
                                      shared_ptr<Node> goal);
    double calc_heuristic(shared_ptr<Node> node1, shared_ptr<Node> node2);

public:
    AStarPlanner() {}
    AStarPlanner(vector<double> ox, vector<double> oy, double reso, double radius)
        : GraphSearchPlanner(ox, oy, reso, radius) {}
    ~AStarPlanner() override {}

    vector<vector<double>> planning(double sx, double sy, double gx, double gy) override;
};

vector<vector<double>> AStarPlanner::planning(double sx, double sy, double gx, double gy) {
    shared_ptr<Node> nstart = std::make_shared<Node>(
        calc_xyindex(sx, get_minx()), calc_xyindex(sy, get_miny()), 0.0, -1, nullptr);
    shared_ptr<Node> ngoal = std::make_shared<Node>(calc_xyindex(gx, get_minx()),
                                                    calc_xyindex(gy, get_miny()), 0.0, -1, nullptr);
    unordered_map<double, shared_ptr<Node>> open_set;
    unordered_map<double, shared_ptr<Node>> closed_set;
    open_set[calc_grid_index(nstart)] = nstart;

    while (true) {
        if (open_set.size() == 0) {
            fmt::print("Open set is empty..\n");
            break;
        }

        shared_ptr<Node> current = get_mincost_node(open_set, ngoal);
        double c_id = calc_grid_index(current);
        open_set.erase(c_id);

        if (show_animation) {
            plt::plot({calc_grid_position(current->x, get_minx())},
                      {calc_grid_position(current->y, get_miny())}, "xc");
            if (closed_set.size() % 10 == 0) {
                plt::pause(0.001);
            }
        }
        if (current->x == ngoal->x && current->y == ngoal->y) {
            fmt::print("Find goal\n");
            ngoal->parent_index = current->parent_index;
            ngoal->cost = current->cost;
            break;
        }

        closed_set[c_id] = current;
        for (const vector<double>& m : get_motion()) {
            shared_ptr<Node> node = std::make_shared<Node>(current->x + m[0], current->y + m[1],
                                                           current->cost + m[2], c_id, current);
            double n_id = calc_grid_index(node);

            if (!verify_node(node) || closed_set.find(n_id) != closed_set.end()) {
                continue;
            }

            if (open_set.find(n_id) == open_set.end() || open_set[n_id]->cost >= node->cost) {
                open_set[n_id] = node;
            }
        }
    }
    vector<vector<double>> path = calc_final_path(ngoal, closed_set);

    return path;
}

shared_ptr<Node> AStarPlanner::get_mincost_node(
    const unordered_map<double, shared_ptr<Node>>& node_set, shared_ptr<Node> goal) {
    shared_ptr<Node> min_node = nullptr;
    for (const auto& n : node_set) {
        if (min_node == nullptr || ((min_node->cost + calc_heuristic(min_node, goal)) >
                                    (n.second->cost + calc_heuristic(n.second, goal)))) {
            min_node = n.second;
        }
    }
    return min_node;
}

double AStarPlanner::calc_heuristic(shared_ptr<Node> node1, shared_ptr<Node> node2) {
    double weight = 1.0;
    double distance = weight * hypot(node1->x - node2->x, node1->y - node2->y);

    return distance;
}

int main(int argc, char** argv) {
    double start_x = 10;
    double start_y = 10;
    double goal_x = 50.0;
    double goal_y = 50.0;
    double grid_size = 2.0;
    double robot_radius = 1.0;

    std::vector<double> obstacle_x;
    std::vector<double> obstacle_y;
    for (int i = -10; i < 60; ++i) {
        obstacle_x.emplace_back(i);
        obstacle_y.emplace_back(-10.0);
    }
    for (int i = -10; i < 60; ++i) {
        obstacle_x.emplace_back(60.0);
        obstacle_y.emplace_back(i);
    }
    for (int i = -10; i < 61; ++i) {
        obstacle_x.emplace_back(i);
        obstacle_y.emplace_back(60.0);
    }
    for (int i = -10; i < 61; ++i) {
        obstacle_x.emplace_back(-10.0);
        obstacle_y.emplace_back(i);
    }
    for (int i = -10; i < 40; ++i) {
        obstacle_x.emplace_back(20.0);
        obstacle_y.emplace_back(i);
    }
    for (int i = 0; i < 40; ++i) {
        obstacle_x.emplace_back(40.0);
        obstacle_y.emplace_back(60.0 - i);
    }
    if (show_animation) {
        plt::plot(obstacle_x, obstacle_y, "sk");
        plt::plot({start_x}, {start_y}, "og");
        plt::plot({goal_x}, {goal_x}, "xb");
        plt::grid(true);
        plt::title("A*");
        plt::axis("equal");
    }
    utils::TicToc t_m;
    AStarPlanner astar(obstacle_x, obstacle_y, grid_size, robot_radius);
    vector<vector<double>> path = astar.planning(start_x, start_y, goal_x, goal_y);
    fmt::print("rrt planning costtime: {:.3f} s\n", t_m.toc() / 1000);
    if (show_animation) {
        plt::plot(path[0], path[1], "-r");
        plt::pause(0.01);
        plt::show();
    }

    return 0;
}
