#pragma once
#include <fmt/core.h>

#include <cmath>
#include <random>
#include <string>
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cmath>
#include "utils.hpp"
#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;
constexpr bool show_animation = false;
class Node{
public:
    double x;
    double y;
    double cost;

    shared_ptr<Node> parent;
    Node()
    : x(0), y(0),cost(0),parent(nullptr)
    {}
    Node(double _x, double _y, double cost_ = 0,shared_ptr<Node> _parent = nullptr)
    : x(_x), y(_y),cost(cost_), parent(_parent)
    {}

    ~Node(){}

};
class Informed_RRT_Star{
public:
    Informed_RRT_Star(Vector2d _start, Vector2d _goal, vector<vector<double>>& obs, vector<vector<bool>>& obstacle_map_,Vector2d rand_area,
        double expand = 1, double goal_sample = 0.2, int _max_iter = 20000,
        double _robot_radius = 1.0);
    ~Informed_RRT_Star(){};

    vector<vector<double>> planning(void);
private:
    shared_ptr<Node> get_random_node(void);
    shared_ptr<Node> get_nearest_node(Node rnd_node);
    shared_ptr<Node> steer(shared_ptr<Node> from_node, shared_ptr<Node> to_node);
    Vector2d calc_distance_and_angle(shared_ptr<Node> from_node, shared_ptr<Node> to_node);
    bool check_collision(shared_ptr<Node> new_node);
    void plot_circle(double x, double y, double size, bool is_fill = false, string style = "-b");
    vector<int> get_neighbor_nodes(shared_ptr<Node> node);
    shared_ptr<Node> get_new_parent(shared_ptr<Node> new_node,vector<int> neighbor_ids);
    void draw_graph(Node rnd);

    void rewire(shared_ptr<Node> new_node,vector<int> neighbor_ids);

    vector<vector<double>> generate_final_course(void);
    void propagateCostToLeaves(shared_ptr<Node>parent_node);

    shared_ptr<Node> transform(double x, double y);
private:
    shared_ptr<Node> start;
    shared_ptr<Node> goal;

    double min_rand;
    double max_rand;
    double expand_dis;

    double c_best_;  
    double c_min_;  

    double path_cost;

    double goal_sample_rate;

    int max_iter;

    int robot_radius;

    vector<shared_ptr<Node>>node_list;

    vector<vector<double>>obstacle_list;
    vector<vector<bool>> obstacle_map;

    vector<vector<double>> boundary;
    vector<vector<double>> dir;
    std::mt19937 engine;
};

