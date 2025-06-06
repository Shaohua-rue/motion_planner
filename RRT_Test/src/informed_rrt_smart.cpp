#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <chrono>

// 定义常量
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
const int distance_threshold = 1;
const int search_radius = 200;
const int N = 1000;
const int b = 4;
const int r_beacon = 100;

// 自定义哈希函数用于 std::pair<int, int>
struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// 调整值到访问空间
int adjust(int x, int threshold) {
    return static_cast<int>(std::round(x * 2) / 2 / threshold);
}

// 检查节点是否有效（不在障碍物或边界内）
bool valid_node(int x, int y, const cv::Mat& canvas) {
    if (x < 0 || x >= canvas.cols || y < 0 || y >= canvas.rows)
        return false;
        
    cv::Vec3b color = canvas.at<cv::Vec3b>(y, x);
    // 将 cv::Scalar 转换为 cv::Vec3b 进行比较
    cv::Vec3b obstacle_color_b(obstacle_color[0], obstacle_color[1], obstacle_color[2]);
    cv::Vec3b clearance_color_b(clearance_color[0], clearance_color[1], clearance_color[2]);
    
    return color != obstacle_color_b && color != clearance_color_b;
}


// 检查边是否有效（两点之间的连线不穿过障碍物）
bool valid_edge(int x_parent, int y_parent, int x_child, int y_child, const cv::Mat& canvas) {
    int steps = std::max(std::abs(x_child - x_parent), std::abs(y_child - y_parent));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        int x = static_cast<int>(x_parent + t * (x_child - x_parent));
        int y = static_cast<int>(y_parent + t * (y_child - y_parent));
        if (!valid_node(x, y, canvas))
            return false;
    }
    return true;
}

// 在单位球内采样
std::pair<int, int> sample_unit_ball() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-1.0, 1.0);
    
    double x, y;
    do {
        x = dis(gen);
        y = dis(gen);
    } while (x*x + y*y > 1.0);
    
    return {static_cast<int>(x * 1000), static_cast<int>(y * 1000)};
}

// 采样节点（在有效空间内随机采样）
std::pair<int, int> sample(int x_start, int y_start, int x_goal, int y_goal, double c_max, const cv::Mat& canvas) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(clearance, width - clearance);
    std::uniform_int_distribution<> dis_y(clearance, height - clearance);
    
    int x, y;
    do {
        x = dis_x(gen);
        y = dis_y(gen);
    } while (!valid_node(x, y, canvas));
    
    return {x, y};
}

// 创建障碍物地图
cv::Mat create_obstacles() {
    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC3);
    cv::rectangle(canvas, cv::Point(clearance, clearance), cv::Point(width - clearance, height - clearance), 
                  cv::Scalar(255, 255, 255), -1);

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

int main() {
    cv::Mat canvas = create_obstacles();

    int x_start = clearance + 1, y_start = clearance + 1;
    int x_goal = width - clearance - 1, y_goal = clearance + 1;

    // 数据结构初始化，使用自定义哈希函数
    std::unordered_map<std::pair<int, int>, int, PairHash> visited;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> parent;
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, int>>, PairHash> children;
    std::unordered_map<std::pair<int, int>, double, PairHash> cost;

    // 初始化起始节点
    visited[{adjust(x_start, distance_threshold), adjust(y_start, distance_threshold)}] = 1;
    parent[{x_start, y_start}] = {x_start, y_start};
    children[{x_start, y_start}] = {};
    cost[{x_start, y_start}] = 0;

    bool reached = false;
    int iterations = 0;
    double c_best = std::numeric_limits<double>::infinity();
    auto start_time = std::chrono::high_resolution_clock::now();

    while (iterations < 500) {
        auto [x_node, y_node] = sample(x_start, y_start, x_goal, y_goal, c_best, canvas);

        if (valid_node(x_node, y_node, canvas)) {
            // 找到最近的节点
            double min_dist = std::numeric_limits<double>::infinity();
            std::pair<int, int> nearest;
            for (const auto& [node, _] : parent) {
                double dist = std::hypot(node.first - x_node, node.second - y_node);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = node;
                }
            }
            int x_parent = nearest.first, y_parent = nearest.second;

            // 计算新节点位置
            double theta = std::atan2(y_node - y_parent, x_node - x_parent);
            int x_new = static_cast<int>(x_parent + step_size * std::cos(theta));
            int y_new = static_cast<int>(y_parent + step_size * std::sin(theta));

            if (reached) {
                if (cost[{x_goal, y_goal}] < c_best) {
                    c_best = cost[{x_goal, y_goal}];
                    std::cout << "New best cost: " << c_best << std::endl;
                }
                ++iterations;
            }

            if (valid_node(x_new, y_new, canvas)) {
                // 找到邻居节点
                std::vector<std::pair<int, int>> neighbours;
                for (const auto& [node, _] : parent) {
                    if (std::hypot(node.first - x_new, node.second - y_new) < search_radius) {
                        neighbours.push_back(node);
                    }
                }

                // 选择最佳父节点
                double min_cost = cost[{x_parent, y_parent}] + step_size;
                for (const auto& [x, y] : neighbours) {
                    double new_cost = cost[{x, y}] + std::hypot(x - x_new, y - y_new);
                    if (new_cost < min_cost) {
                        min_cost = new_cost;
                        x_parent = x;
                        y_parent = y;
                    }
                }

                if (!valid_edge(x_parent, y_parent, x_new, y_new, canvas)) {
                    continue;
                }

                cost[{x_new, y_new}] = min_cost;

                // 优化现有节点
                for (const auto& [x, y] : neighbours) {
                    double new_cost = cost[{x_new, y_new}] + std::hypot(x - x_new, y - y_new);
                    if (new_cost < cost[{x, y}]) {
                        if (!valid_edge(x, y, x_new, y_new, canvas)) {
                            continue;
                        }
                        double previous_cost = cost[{x, y}];
                        cost[{x, y}] = new_cost;

                        // 更新父节点关系
                        auto [x_parent_neighbour, y_parent_neighbour] = parent[{x, y}];
                        auto& children_list = children[{x_parent_neighbour, y_parent_neighbour}];
                        children_list.erase(std::remove(children_list.begin(), children_list.end(), std::make_pair(x, y)), children_list.end());

                        parent[{x, y}] = {x_new, y_new};
                        children[{x_new, y_new}].push_back({x, y});

                        // 传播成本减少
                        double cost_reduction = previous_cost - new_cost;
                        std::queue<std::pair<int, int>> queue;
                        queue.push({x, y});
                        while (!queue.empty()) {
                            auto [x_, y_] = queue.front();
                            queue.pop();
                            if (children.find({x_, y_}) != children.end()) {
                                for (const auto& child : children[{x_, y_}]) {
                                    cost[child] -= cost_reduction;
                                    queue.push(child);
                                }
                            }
                        }
                    }
                }

                // 添加新节点
                int x_adjusted = adjust(x_new, distance_threshold);
                int y_adjusted = adjust(y_new, distance_threshold);
                visited[{x_adjusted, y_adjusted}] = 1;
                parent[{x_new, y_new}] = {x_parent, y_parent};
                children[{x_parent, y_parent}].push_back({x_new, y_new});

                // 检查是否到达目标
                double goal_distance = std::hypot(x_new - x_goal, y_new - y_goal);
                if (goal_distance < 100 && cost[{x_new, y_new}] + goal_distance < c_best) {
                    auto end_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = end_time - start_time;
                    std::cout << "Goal reached in " << elapsed.count() << " seconds" << std::endl;

                    c_best = cost[{x_new, y_new}] + goal_distance;
                    children[{x_new, y_new}].push_back({x_goal, y_goal});
                    parent[{x_goal, y_goal}] = {x_new, y_new};
                    cost[{x_goal, y_goal}] = c_best;

                    std::cout << "Current Cost: " << cost[{x_goal, y_goal}] << std::endl;
                    reached = true;
                }
            }
        }
    }

    std::cout << "Final cost: " << cost[{x_goal, y_goal}] << std::endl;

    // 生成路径
    std::vector<std::pair<int, int>> path;
    std::pair<int, int> current = {x_goal, y_goal};
    while (current != std::make_pair(x_start, y_start)) {
        path.push_back(current);
        current = parent[current];
    }
    path.push_back({x_start, y_start});
    std::reverse(path.begin(), path.end());

    // 计算路径总长度
    double sum = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        sum += std::hypot(path[i].first - path[i + 1].first, path[i].second - path[i + 1].second);
    }
    std::cout << "Total path distance: " << sum << std::endl;
    std::cout << "Difference in cost: " << std::round(cost[{x_goal, y_goal}] - sum) << std::endl;

    // 可视化
    canvas = create_obstacles();
    cv::circle(canvas, cv::Point(x_start, y_start), 5, cv::Scalar(254, 0, 0), 10);
    cv::circle(canvas, cv::Point(x_goal, y_goal), 5, cv::Scalar(254, 0, 0), 10);

    // 绘制所有节点和边
    for (const auto& [node, parent_node] : parent) {
        cv::line(canvas, cv::Point(node.first, node.second), cv::Point(parent_node.first, parent_node.second), 
                 cv::Scalar(0, 255, 0), 2);
        cv::circle(canvas, cv::Point(parent_node.first, parent_node.second), 2, cv::Scalar(255, 0, 255), 1);
    }

    // 绘制最终路径
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(canvas, cv::Point(path[i].first, path[i].second), cv::Point(path[i + 1].first, path[i + 1].second), 
                 cv::Scalar(0, 0, 255), 5);
    }

    cv::circle(canvas, cv::Point(x_goal, y_goal), 5, cv::Scalar(0, 0, 255), 10);
    cv::circle(canvas, cv::Point(x_start, y_start), 5, cv::Scalar(254, 0, 0), 10);

    // 调整图像大小以便显示
    cv::Mat canvas_resized;
    cv::resize(canvas, canvas_resized, cv::Size(width / scale, height / scale));

    cv::imshow("RRT* Smart Path Planning", canvas_resized);
    std::cout << "Press any key to exit..." << std::endl;
    cv::waitKey(0);

    return 0;
}
