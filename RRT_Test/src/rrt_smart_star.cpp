#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <chrono>

const double WHEEL_RADIUS = 33;
const double ROBOT_RADIUS = 220;
const double WHEEL_DISTANCE = 287;
const int clearance = 5 + static_cast<int>(ROBOT_RADIUS);
const int width = 6000;
const int height = 2000;
const int scale = 5;
const cv::Scalar clearance_color = cv::Scalar(0, 255, 255);
const cv::Scalar obstacle_color = cv::Scalar(0, 0, 0);
const cv::Scalar robot_radius_color = cv::Scalar(254, 105, 180);

const int step_size = 100;
const int distance_threshold = 10;
const int search_radius = 200;
const int N = 100;
const int b = 4;
const int r_beacon = 100;

struct Point {
    int x;
    int y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
     // 重载 != 运算符
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// 自定义调整函数
int adjust(int x, int threshold) {
    return static_cast<int>(std::round(x * 2) / 2) / threshold;
}

// 创建障碍物地图
cv::Mat obstacles() {
    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC3);
    cv::rectangle(canvas, cv::Point(clearance, clearance), cv::Point(width - clearance, height - clearance), cv::Scalar(255, 255, 255), -1);

    // OBSTACLE 1
    int x1 = 1500, x2 = 1750;
    int y1 = 0, y2 = 1000;
    for (int i = x1 - clearance; i < x2 + clearance; ++i) {
        for (int j = y1 + clearance; j < y2 + clearance; ++j) {
            canvas.at<cv::Vec3b>(j, i) = cv::Vec3b(clearance_color[0], clearance_color[1], clearance_color[2]);
        }
    }
    for (int i = x1; i < x2; ++i) {
        for (int j = y1; j < y2; ++j) {
            canvas.at<cv::Vec3b>(j, i) = cv::Vec3b(obstacle_color[0], obstacle_color[1], obstacle_color[2]);
        }
    }

    // OBSTACLE 2
    x1 = 2500, x2 = 2750;
    y1 = height - 1000, y2 = height;
    for (int i = x1 - clearance; i < x2 + clearance; ++i) {
        for (int j = y1 - clearance; j < y2 - clearance + 1; ++j) {
            canvas.at<cv::Vec3b>(j, i) = cv::Vec3b(clearance_color[0], clearance_color[1], clearance_color[2]);
        }
    }
    for (int i = x1; i < x2; ++i) {
        for (int j = y1; j < y2; ++j) {
            canvas.at<cv::Vec3b>(j, i) = cv::Vec3b(obstacle_color[0], obstacle_color[1], obstacle_color[2]);
        }
    }

    // OBSTACLE 3
    cv::Point center(4200, 800);
    int radius = 600;
    cv::circle(canvas, center, radius + clearance, clearance_color, -1);
    cv::circle(canvas, center, radius, obstacle_color, -1);

    return canvas;
}

// 检查节点是否有效
bool valid_node(const cv::Mat& canvas, const std::unordered_map<Point, int, PointHash>& visited, int x, int y) {
    int x_vis = adjust(x, distance_threshold);
    int y_vis = adjust(y, distance_threshold);
    int x_cvs = static_cast<int>(std::round(x * 2) / 2);
    int y_cvs = static_cast<int>(std::round(y * 2) / 2);

    return canvas.at<cv::Vec3b>(y_cvs, x_cvs)[0] != 0 && visited.find({x_vis, y_vis}) == visited.end();
}

// 检查边是否有效
bool valid_edge(const cv::Mat& canvas, int x_parent, int y_parent, int x_child, int y_child) {
    int n_sample = 10;
    std::vector<int> x_intermediate(n_sample - 2);
    std::vector<int> y_intermediate(n_sample - 2);
    for (int i = 0; i < n_sample - 2; ++i) {
        x_intermediate[i] = static_cast<int>(std::round((x_parent + (i + 1) * (x_child - x_parent) / n_sample) * 2) / 2);
        y_intermediate[i] = static_cast<int>(std::round((y_parent + (i + 1) * (y_child - y_parent) / n_sample) * 2) / 2);
    }

    for (size_t i = 0; i < x_intermediate.size(); ++i) {
        if (canvas.at<cv::Vec3b>(y_intermediate[i], x_intermediate[i])[0] == 0) {
            return false;
        }
    }
    return true;
}

// 计算路径
std::vector<Point> compute_path(const std::unordered_map<Point, Point, PointHash>& parent, Point x_start, Point x_final) {
    std::vector<Point> path;
    Point current = x_final;
    while (current != x_start) {
        path.push_back(current);
        current = parent.at(current);
    }
    path.push_back(x_start);
    std::reverse(path.begin(), path.end());
    return path;
}

// 采样函数
Point sample(bool reached, int iterations, const std::vector<Point>& beacons) {
    std::random_device rd;
    std::mt19937 gen(rd());
    Point node;
    if (reached && iterations % 2 == 0 &&!beacons.empty()) {
        std::uniform_int_distribution<> dis(0, beacons.size() - 1);
        Point beacon = beacons[dis(gen)];
        std::uniform_int_distribution<> dis_x(beacon.x - r_beacon, beacon.x + r_beacon);
        std::uniform_int_distribution<> dis_y(beacon.y - r_beacon, beacon.y + r_beacon);
        node = {dis_x(gen), dis_y(gen)};
    } else {
        std::uniform_int_distribution<> dis_x(clearance, width - clearance);
        std::uniform_int_distribution<> dis_y(clearance, height - clearance);
        node = {dis_x(gen), dis_y(gen)};
    }
    return node;
}

int main() {
    cv::Mat canvas = obstacles();

    Point x_start = {clearance + 1, clearance + 1};
    Point x_goal = {width - clearance - 1, clearance + 1};

    std::unordered_map<Point, int, PointHash> visited;
    std::unordered_map<Point, Point, PointHash> parent;
    std::unordered_map<Point, std::vector<Point>, PointHash> children;
    std::unordered_map<Point, double, PointHash> cost;

    visited[x_start] = 1;
    parent[x_start] = x_start;
    children[x_start] = {};
    cost[x_start] = 0;

    bool reached = false;
    int iterations = 0;
    double c_best = std::numeric_limits<double>::infinity();
    int n = -1;
    auto start = std::chrono::high_resolution_clock::now();

    while (iterations < 500) {
        Point x_node = sample(reached, iterations, {});

        if (valid_node(canvas, visited, x_node.x, x_node.y)) {
            double min_dist = std::numeric_limits<double>::infinity();
            Point x_parent;
            for (const auto& p : parent) {
                double dist = std::sqrt(std::pow(p.first.x - x_node.x, 2) + std::pow(p.first.y - x_node.y, 2));
                if (dist < min_dist) {
                    min_dist = dist;
                    x_parent = p.first;
                }
            }

            double theta = std::atan2(x_node.y - x_parent.y, x_node.x - x_parent.x);
            int x_new = static_cast<int>(x_parent.x + step_size * std::cos(theta));
            int y_new = static_cast<int>(x_parent.y + step_size * std::sin(theta));

            if (reached) {
                iterations++;
            }

            if (valid_node(canvas, visited, x_new, y_new)) {
                std::vector<Point> neighbours;
                for (const auto& p : parent) {
                    if (std::sqrt(std::pow(p.first.x - x_new, 2) + std::pow(p.first.y - y_new, 2)) < search_radius) {
                        neighbours.push_back(p.first);
                    }
                }

                double min_cost = cost[x_parent] + step_size;
                for (const auto& p : neighbours) {
                    double new_cost = cost[p] + std::sqrt(std::pow(p.x - x_new, 2) + std::pow(p.y - y_new, 2));
                    if (new_cost < min_cost) {
                        min_cost = new_cost;
                        x_parent = p;
                    }
                }

                if (!valid_edge(canvas, x_parent.x, x_parent.y, x_new, y_new)) {
                    continue;
                }

                cost[{x_new, y_new}] = min_cost;

                for (const auto& p : neighbours) {
                    double new_cost = cost[{x_new, y_new}] + std::sqrt(std::pow(p.x - x_new, 2) + std::pow(p.y - y_new, 2));
                    if (new_cost < cost[p]) {
                        if (!valid_edge(canvas, p.x, p.y, x_new, y_new)) {
                            continue;
                        }
                        double previous_cost = cost[p];
                        cost[p] = new_cost;

                        Point x_parent_neighbour = parent[p];
                        auto& children_list = children[x_parent_neighbour];
                        children_list.erase(std::remove(children_list.begin(), children_list.end(), p), children_list.end());

                        parent[p] = {x_new, y_new};
                        children[{x_new, y_new}].push_back(p);

                        double cost_reduction = previous_cost - new_cost;
                        std::queue<Point> queue;
                        queue.push(p);
                        while (!queue.empty()) {
                            Point current = queue.front();
                            queue.pop();
                            if (children.find(current) != children.end()) {
                                for (const auto& child : children[current]) {
                                    cost[child] -= cost_reduction;
                                    queue.push(child);
                                }
                            }
                        }
                    }
                }

                Point x_achieved = {x_new, y_new};
                visited[{adjust(x_new, distance_threshold), adjust(y_new, distance_threshold)}] = 1;
                parent[{x_new, y_new}] = x_parent;
                children[x_parent].push_back({x_new, y_new});

                if (reached) {
                    std::vector<Point> path = compute_path(parent, x_start, {x_new, y_new});
                    Point node = path.back();
                    Point parent_node = parent[node];
                    Point grandparent_node = parent[parent_node];

                    while (node != x_start) {
                        if (valid_edge(canvas, grandparent_node.x, grandparent_node.y, node.x, node.y)) {
                            auto& children_list = children[parent_node];
                            children_list.erase(std::remove(children_list.begin(), children_list.end(), node), children_list.end());

                            parent[node] = grandparent_node;
                            children[grandparent_node].push_back(node);

                            double previous_cost = cost[node];
                            cost[node] = cost[grandparent_node] + std::sqrt(std::pow(node.x - grandparent_node.x, 2) + std::pow(node.y - grandparent_node.y, 2));

                            double cost_reduction = previous_cost - cost[node];
                            std::queue<Point> queue;
                            queue.push(node);
                            while (!queue.empty()) {
                                Point current = queue.front();
                                queue.pop();
                                if (children.find(current) != children.end()) {
                                    for (const auto& child : children[current]) {
                                        cost[child] -= cost_reduction;
                                        queue.push(child);
                                    }
                                }
                            }

                            node = grandparent_node;
                            parent_node = parent[node];
                            grandparent_node = parent[parent_node];
                        } else {
                            node = parent_node;
                            parent_node = parent[node];
                            grandparent_node = parent[parent_node];
                        }
                    }
                }

                double goal_distance = std::sqrt(std::pow(x_new - x_goal.x, 2) + std::pow(y_new - x_goal.y, 2));
                if (goal_distance < 100 && cost[{x_new, y_new}] + goal_distance < c_best) {
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = end - start;
                    std::cout << "Goal reached: " << elapsed.count() << " seconds" << std::endl;

                    c_best = cost[{x_new, y_new}] + goal_distance;
                    std::vector<Point> beacons;

                    children[{x_new, y_new}].push_back(x_goal);
                    parent[x_goal] = {x_new, y_new};
                    Point x_final = x_goal;
                    cost[x_goal] = c_best;
                    n = iterations;

                    std::cout << "Current Cost: " << cost[x_final] << std::endl;
                    reached = true;
                }
            }
        }
    }

    std::cout << "Final cost: " << cost[x_goal] << std::endl;

    std::vector<Point> path = compute_path(parent, x_start, x_goal);

    double sum = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        sum += std::sqrt(std::pow(path[i].x - path[i + 1].x, 2) + std::pow(path[i].y - path[i + 1].y, 2));
    }

    std::cout << "Total distance: " << sum << std::endl;
    std::cout << "Difference in cost: " << std::round(cost[x_goal] - sum) << std::endl;

    canvas = obstacles();

    cv::circle(canvas, cv::Point(x_start.x, x_start.y), 5, cv::Scalar(0, 0, 254), 10);
    cv::circle(canvas, cv::Point(x_goal.x, x_goal.y), 5, cv::Scalar(0, 0, 254), 10);

    for (const auto& p : parent) {
        cv::line(canvas, cv::Point(p.first.x, p.first.y), cv::Point(p.second.x, p.second.y), cv::Scalar(0, 255, 0), 5);
        cv::circle(canvas, cv::Point(p.second.x, p.second.y), 5, cv::Scalar(255, 0, 255), 3);
    }

    for (const auto& p : children) {
        for (const auto& child : p.second) {
            cv::circle(canvas, cv::Point(child.x, child.y), 5, cv::Scalar(255, 0, 255), 3);
        }
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(canvas, cv::Point(path[i].x, path[i].y), cv::Point(path[i + 1].x, path[i + 1].y), cv::Scalar(0, 0, 255), 5);
    }

    cv::Mat canvas_resized;
    cv::resize(canvas, canvas_resized, cv::Size(width / scale, height / scale));

    cv::imshow("Canvas", canvas_resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

