#include "rrt.h"

// 坐标到地图索引的映射函数
int coordToIndex(double coord, double min_coord) {
    return static_cast<int>(coord - min_coord);
}

int main(int argc, char** argv) {

    vector<vector<double>> obstacle_list;
    obstacle_list.resize(2);
    utils::TicToc t_m;
    Vector2d start(10, 10);
    Vector2d goal(50, 50);
    Vector2d area(-10, 60);

    int n = area[1] - area[0] + 1;
    vector<vector<bool>> obstacle_map(n,vector<bool>(n, false));
    for (int i = -10; i < 60; ++i) {
        int x = coordToIndex(i,-10);
        int y = coordToIndex(-10,-10);
        obstacle_list[0].push_back(i);
        obstacle_list[1].push_back(-10);
        obstacle_map[x][y] = true;
    }
    for (int i = -10; i < 60; ++i) {
        int x = coordToIndex(60,-10);
        int y = coordToIndex(i,-10);
        obstacle_list[0].push_back(60);
        obstacle_list[1].push_back(i);
        obstacle_map[x][y] = true;
    }
    for (int i = -10; i < 61; ++i) {
        int x = coordToIndex(i,-10);
        int y = coordToIndex(60,-10);
        obstacle_list[0].push_back(i);
        obstacle_list[1].push_back(60);
        obstacle_map[x][y] = true;
    }
    for (int i = -10; i < 61; ++i) {
        int x = coordToIndex(-10,-10);
        int y = coordToIndex(i,-10);
        obstacle_list[0].push_back(-10);
        obstacle_list[1].push_back(i);
        obstacle_map[x][y] = true;
    }
    for (int i = -10; i < 40; ++i) {
        
        int x = coordToIndex(20,-10);
        int y = coordToIndex(i,-10);
        obstacle_list[0].push_back(20);
        obstacle_list[1].push_back(i);
        obstacle_map[x][y] = true;
    }
    for (int i = 0; i < 40; ++i) {
        int x = coordToIndex(i,-10);
        int y = coordToIndex(-10,-10);
        obstacle_list[0].push_back(i);
        obstacle_list[1].push_back(-10);
        obstacle_map[x][y] = true;
    }
    for (int i = 0; i < 40; ++i) {
        int x = coordToIndex(40.0,-10);
        int y = coordToIndex(60.0 - i,-10);
        obstacle_list[0].push_back(40);
        obstacle_list[1].push_back(60-i);
        obstacle_map[x][y] = true;
    }

    RRT rrt(start, goal, obstacle_list,obstacle_map,area);
    vector<vector<double>> path = rrt.planning();
    fmt::print("rrt planning costtime: {:.3f} s\n", t_m.toc() / 1000);

    if (path[0].size() < 2) {
        fmt::print("planning failed!\n");
        return 0;
    }


    plt::plot(path[0], path[1], "-r");
    plt::grid(true);
    plt::show();


    return 0;
}
