#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <random>
#include <limits>

// 为 std::pair<int, int> 提供自定义哈希函数
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

// 定义机器人和环境相关参数
const int WHEEL_RADIUS = 33;
const int ROBOT_RADIUS = 220;
const int WHEEL_DISTANCE = 287;
const int clearance = 5 + ROBOT_RADIUS;
const int width = 6000;
const int height = 2000;
const int scale = 5;

const cv::Scalar clearance_color(0, 255, 255);
const cv::Scalar obstacle_color(0, 0, 0);
const cv::Scalar robot_radius_color(254, 105, 180);

// 创建包含障碍物的画布
cv::Mat obstacles() {
    cv::Mat canvas = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    cv::rectangle(canvas, cv::Point(clearance, clearance), cv::Point(width - clearance, height - clearance), cv::Scalar(255, 255, 255), -1);

    // 障碍物 1
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

    // 障碍物 2
    x1 = 2500; x2 = 2750;
    y1 = height - 1000; y2 = height;
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

    // 障碍物 3
    cv::Point center(4200, 800);
    int radius = 600;
    cv::circle(canvas, center, 600 + clearance, clearance_color, -1);
    cv::circle(canvas, center, 600, obstacle_color, -1);

    return canvas;
}

// 调整坐标以用于访问已访问节点
int adjust(int x, int threshold) {
    return static_cast<int>(std::round(x * 2) / 2) / threshold;
}

// 检查节点是否有效
bool valid_node(const cv::Mat& canvas, const std::unordered_map<std::pair<int, int>, int, pair_hash>& visited, int x, int y, int distance_threshold) {
    int x_vis = adjust(x, distance_threshold);
    int y_vis = adjust(y, distance_threshold);
    int x_cvs = static_cast<int>(std::round(x * 2) / 2);
    int y_cvs = static_cast<int>(std::round(y * 2) / 2);

    return canvas.at<cv::Vec3b>(y_cvs, x_cvs)[0] != 0 && visited.find({x_vis, y_vis}) == visited.end();
}

// 检查边是否有效
bool valid_edge(const cv::Mat& canvas, int x_parent, int y_parent, int x_child, int y_child) {
    // 采样 3 个中间点
    std::vector<int> x_intermediate;
    std::vector<int> y_intermediate;
    for (int i = 1; i < 4; ++i) {
        double t = static_cast<double>(i) / 4;
        int x = static_cast<int>(std::round((1 - t) * x_parent + t * x_child));
        int y = static_cast<int>(std::round((1 - t) * y_parent + t * y_child));
        x_intermediate.push_back(static_cast<int>(std::round(x * 2) / 2));
        y_intermediate.push_back(static_cast<int>(std::round(y * 2) / 2));
    }

    for (size_t i = 0; i < x_intermediate.size(); ++i) {
        int x = x_intermediate[i];
        int y = y_intermediate[i];
        if (canvas.at<cv::Vec3b>(y, x)[0] == 0) {
            return false;
        }
    }
    return true;
}

// 计算两点之间的距离
double distance(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

int main() {
    cv::Mat canvas = obstacles();

    int x_start = clearance + 1, y_start = clearance + 1, theta_start = 0;
    int x_goal = width - clearance - 1, y_goal = clearance + 1;

    int step_size = 100;
    int distance_threshold = 100;
    int search_radius = 300;

    // 使用自定义哈希函数
    std::unordered_map<std::pair<int, int>, int, pair_hash> visited;
    visited[{adjust(x_start, distance_threshold), adjust(y_start, distance_threshold)}] = 1;

    // 使用自定义哈希函数
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> parent;
    parent[{x_start, y_start}] = {x_start, y_start};

    // 使用自定义哈希函数
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, pair_hash> children;
    children[{x_start, y_start}] = {};

    // 使用自定义哈希函数
    std::unordered_map<std::pair<int, int>, double, pair_hash> cost;
    cost[{x_start, y_start}] = 0;

    bool reached = false;
    int iterations = 0;
    double c_best = std::numeric_limits<double>::max();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(clearance, width - clearance);
    std::uniform_int_distribution<> dis_y(clearance, height - clearance);

    auto start = std::chrono::high_resolution_clock::now();
    while (iterations < 500) {
        int x_node = dis_x(gen);
        int y_node = dis_y(gen);

        if (valid_node(canvas, visited, x_node, y_node, distance_threshold)) {
            double min_dist = std::numeric_limits<double>::max();
            std::pair<int, int> nearest;
            for (const auto& p : parent) {
                double dist = distance(p.first.first, p.first.second, x_node, y_node);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = p.first;
                }
            }
            int x_parent = nearest.first;
            int y_parent = nearest.second;

            double theta = std::atan2(y_node - y_parent, x_node - x_parent);
            int x_new = static_cast<int>(x_parent + step_size * std::cos(theta));
            int y_new = static_cast<int>(y_parent + step_size * std::sin(theta));

            if (reached) {
                iterations++;
            }

            if (valid_node(canvas, visited, x_new, y_new, distance_threshold)) {
                std::vector<std::pair<int, int>> neighbours;
                for (const auto& p : parent) {
                    if (distance(p.first.first, p.first.second, x_new, y_new) < search_radius) {
                        neighbours.push_back(p.first);
                    }
                }

                double min_cost = cost[{x_parent, y_parent}] + step_size;
                for (const auto& n : neighbours) {
                    double new_cost = cost[n] + distance(n.first, n.second, x_new, y_new);
                    if (new_cost < min_cost) {
                        min_cost = new_cost;
                        x_parent = n.first;
                        y_parent = n.second;
                    }
                }

                if (!valid_edge(canvas, x_parent, y_parent, x_new, y_new)) {
                    continue;
                }

                cost[{x_new, y_new}] = min_cost;

                for (const auto& n : neighbours) {
                    double new_cost = cost[{x_new, y_new}] + distance(n.first, n.second, x_new, y_new);
                    if (new_cost < cost[n]) {
                        if (!valid_edge(canvas, n.first, n.second, x_new, y_new)) {
                            continue;
                        }

                        double previous_cost = cost[n];
                        cost[n] = new_cost;

                        std::pair<int, int> x_parent_neighbour = parent[n];
                        auto& children_list = children[x_parent_neighbour];
                        children_list.erase(std::remove(children_list.begin(), children_list.end(), n), children_list.end());

                        parent[n] = {x_new, y_new};
                        children[{x_new, y_new}].push_back(n);

                        double cost_reduction = previous_cost - new_cost;
                        std::queue<std::pair<int, int>> queue;
                        queue.push(n);
                        while (!queue.empty()) {
                            std::pair<int, int> current = queue.front();
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

                int x_achieved = x_new;
                int y_achieved = y_new;

                int x_adjusted = adjust(x_new, distance_threshold);
                int y_adjusted = adjust(y_new, distance_threshold);

                visited[{x_adjusted, y_adjusted}] = 1;
                parent[{x_new, y_new}] = {x_parent, y_parent};
                children[{x_parent, y_parent}].push_back({x_new, y_new});

                if (!reached) {
                    if (distance(x_new, y_new, x_goal, y_goal) < distance_threshold) {
                        auto end = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> elapsed = end - start;
                        std::cout << "Goal reached: " << elapsed.count() << " seconds" << std::endl;

                        children[{x_new, y_new}].push_back({x_goal, y_goal});
                        parent[{x_goal, y_goal}] = {x_new, y_new};

                        int x_final = x_goal;
                        int y_final = y_goal;

                        cost[{x_goal, y_goal}] = cost[{x_new, y_new}] + distance(x_goal, y_goal, x_new, y_new);
                        std::cout << "Current Cost: " << cost[{x_goal, y_goal}] << std::endl;

                        reached = true;
                    }
                }
            }
        }
    }

    int x_final = x_goal;
    int y_final = y_goal;
    std::cout << "Final cost: " << cost[{x_final, y_final}] << std::endl;

    std::vector<std::pair<int, int>> path;
    std::pair<int, int> current = {x_final, y_final};
    while (current != std::make_pair(x_start, y_start)) {
        path.push_back(current);
        current = parent[current];
    }
    path.push_back({x_start, y_start});
    std::reverse(path.begin(), path.end());

    double sum = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        sum += distance(path[i].first, path[i].second, path[i + 1].first, path[i + 1].second);
    }

    std::cout << "Total distance: " << sum << std::endl;
    std::cout << "Difference in cost: " << std::round(cost[{x_final, y_final}] - sum) << std::endl;

    canvas = obstacles();

    cv::circle(canvas, cv::Point(x_start, y_start), 5, cv::Scalar(0, 0, 254), 10);
    cv::circle(canvas, cv::Point(x_goal, y_goal), 5, cv::Scalar(0, 0, 254), 10);

    for (const auto& p : parent) {
        cv::line(canvas, cv::Point(p.first.first, p.first.second), cv::Point(p.second.first, p.second.second), cv::Scalar(0, 255, 0), 5);
        cv::circle(canvas, cv::Point(p.second.first, p.second.second), 5, cv::Scalar(255, 0, 255), 3);
    }

    for (const auto& c : children) {
        for (const auto& child : c.second) {
            cv::circle(canvas, cv::Point(child.first, child.second), 5, cv::Scalar(255, 0, 255), 3);
        }
    }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(canvas, cv::Point(path[i].first, path[i].second), cv::Point(path[i + 1].first, path[i + 1].second), cv::Scalar(0, 0, 255), 5);
    }

    int width_resized = width / scale;
    int height_resized = height / scale;
    cv::Mat canvas_resized;
    cv::resize(canvas, canvas_resized, cv::Size(width_resized, height_resized));

    cv::imshow("Canvas", canvas_resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

