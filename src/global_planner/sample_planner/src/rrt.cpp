#include "rrt.h"

RRT::RRT(Vector2d _start, Vector2d _goal, vector<vector<double>>& obs, vector<vector<bool>>& obstacle_map_,Vector2d rand_area,
        double expand, double goal_sample, int _max_iter,
        double _robot_radius){
    start = make_shared<Node>(_start[0], _start[1]);
    goal = make_shared<Node>(_goal[0], _goal[1]);
    min_rand = rand_area[0];
    max_rand = rand_area[1];
    expand_dis = expand;
    goal_sample_rate = goal_sample;
    max_iter = _max_iter;
    robot_radius = _robot_radius;
    obstacle_list = obs;
    obstacle_map = obstacle_map_;
    std::random_device seed;
    engine.seed(seed());

    boundary.resize(2);
    for (double i = min_rand - 1; i < (max_rand + 1); i += 0.2) {
        boundary[0].push_back(i);
        boundary[0].push_back(max_rand + 1);
        boundary[0].push_back(i);
        boundary[0].push_back(min_rand - 1);
        boundary[1].push_back(min_rand - 1);
        boundary[1].push_back(i);
        boundary[1].push_back(max_rand + 1);
        boundary[1].push_back(i);
    }
    vector<vector<double>> tmp = {{robot_radius, 0}, {-robot_radius, 0}, {0, robot_radius}, {0, -robot_radius}};
    dir = tmp;

}

vector<vector<double>> RRT::planning(void){
    node_list.push_back(start);
    for(int iterator  = 0; iterator < max_iter; iterator++){
        shared_ptr<Node> rnd_node = get_random_node();

        shared_ptr<Node> nearest_node = get_nearest_node(*rnd_node);
        shared_ptr<Node> new_node = steer(nearest_node, rnd_node);

        if(check_collision(new_node)){
            node_list.emplace_back(new_node);
        }else
        {
            continue;
        }
        if(show_animation && (iterator % 5 == 0)) {
            draw_graph(*rnd_node);
        }
        Vector2d d_angle = calc_distance_and_angle(node_list.back(), goal);
        if (d_angle[0] <= 1.0) {
            goal->parent = node_list.back();
            break;
        }
    }
    return generate_final_course();

}


/***
*@brief get random node
***/
shared_ptr<Node> RRT::get_random_node(void) {
    shared_ptr<Node> rnd = nullptr;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(engine) > goal_sample_rate) {
        std::uniform_real_distribution<double> dist_xy(min_rand, max_rand);
        rnd = make_shared<Node>(dist_xy(engine), dist_xy(engine));
    } else {
        rnd = make_shared<Node>(goal->x, goal->y);
    }

    return rnd;
}
/**
*@brief get nearest node
*
 */
shared_ptr<Node> RRT::get_nearest_node(Node rnd_node) {
    shared_ptr<Node> min_node = make_shared<Node>(node_list[0]->x, node_list[0]->y);
    double distance = hypot(min_node->x - rnd_node.x, min_node->y - rnd_node.y);
    for (int idx = 1; idx < node_list.size(); ++idx) {
        double tmp = hypot(node_list[idx]->x - rnd_node.x, node_list[idx]->y - rnd_node.y);
        if (tmp < distance) {
            min_node = node_list[idx];
            distance = tmp;
        }
    }

    return min_node;
}

/**
*@brief calculate new node 控制树的扩展
*/
shared_ptr<Node>  RRT::steer(shared_ptr<Node>  from_node, shared_ptr<Node>  to_node) {
    Vector2d d_angle = calc_distance_and_angle(from_node, to_node);
    double dist = std::min(expand_dis, d_angle[0]);
    shared_ptr<Node>  node_new =
        make_shared<Node>(from_node->x + dist * cos(d_angle[1]), from_node->y + dist * sin(d_angle[1]));
    node_new->parent = from_node;

    return node_new;
}

/**
*@brief calculate distance and angle
*/
Vector2d RRT::calc_distance_and_angle(shared_ptr<Node> from_node, shared_ptr<Node> to_node) {
    Vector2d d_angle;
    double dx = to_node->x - from_node->x;
    double dy = to_node->y - from_node->y;
    double d = hypot(dx, dy);
    double angle = atan2(dy, dx);
    d_angle << d, angle;

    return d_angle;
}

/***
*@brief check_collision
*/
bool RRT::check_collision(shared_ptr<Node> new_node) {
    if (new_node == nullptr) {
        return true;
    }
    auto parent = new_node->parent;
    vector<vector<double>> point_list;
    point_list.clear();

    int distance = hypot(new_node->x - parent->x, new_node->y - parent->y);
    if(distance >= 2 && parent != nullptr){
        float d_x = (new_node->x - parent->x);
        float d_y = (new_node->y - parent->y);
    
        // 在父节点和新节点之间插入中间点
        for (int i = 0; i < distance; i++) {
            double t = static_cast<double>(i) / (distance - 1);  // t从0到1
            double x = parent->x + t * d_x;
            double y = parent->y + t * d_y;
            point_list.emplace_back(vector<double>{x, y});
        }
        
 
        for (int index = 0; index < point_list.size(); index++){   
            double world_x = point_list[index][0] - min_rand;
            double world_y = point_list[index][1] - min_rand;
            for(auto d: dir){
                int index_x = ceil(world_x + d[0]);
                int index_y = ceil(world_y + d[1]);
                if(index_x >= 0 && index_x < obstacle_map.size() && index_y >= 0 && index_y <  obstacle_map.size())
                {
                    if(obstacle_map[index_x][index_y] == 1){
                        return false;
                    }
                }
            }
        }

    } 
    else
    {
        // 短距离情况下，只检查新节点是否和障碍物发生碰撞
        for (vector<double> obs : obstacle_list) {
             for(auto d: dir){
                int index_x = ceil(new_node->x + d[0])- min_rand;
                int index_y = ceil(new_node->y + d[1])- min_rand;
                if(index_x >= 0 && index_x < obstacle_map.size() && index_y >= 0 && index_y <  obstacle_map.size())
                {
                    if(obstacle_map[index_x][index_y] == 1){
                        return false;
                    }
                }
            }
        }
    }
    point_list.clear();

    return true; // 没有检测到碰撞
}


/**
*@brief final path
*/
vector<vector<double>> RRT::generate_final_course(void) {
    vector<vector<double>> final_path(2);
    shared_ptr<Node> n = goal;

    while (n != nullptr) {
        final_path[0].emplace_back(n->x);
        final_path[1].emplace_back(n->y);
        n = n->parent;
    }

    return final_path;
}

// /**
// *@brief plot circle
// */

// void RRT::plot_circle(double x, double y, double size, bool is_fill, string style) {
//     vector<double> xl;
//     vector<double> yl;
//     for (double deg = 0; deg <= M_PI * 2; deg += (M_PI / 36.0)) {
//         xl.push_back(x + size * cos(deg));
//         yl.push_back(y + size * sin(deg));
//     }
//     if (is_fill) {
//         plt::fill(xl, yl, {{"color", "gray"}});
//     } else {
//         plt::plot(xl, yl, style);
//     }
// }

/**
*@brief draw_graph
*/
void RRT::draw_graph(Node rnd) {
    plt::clf();

    plt::plot(boundary[0], boundary[1], "sk");
    plt::plot({rnd.x}, {rnd.y}, "^k");


    plt::plot(obstacle_list[0],obstacle_list[1], "sk");

    for (shared_ptr<Node> n : node_list) {
        if (n->parent != nullptr) {
            plt::plot({n->x, n->parent->x}, {n->y, n->parent->y}, "-g");
        }
    }

    // for (vector<double> obs : obstacle_list) {
    //     plot_circle(obs[0], obs[1], obs[2], true);
    // }

    plt::plot({start->x}, {start->y}, "xr");
    plt::plot({goal->x}, {goal->y}, "xr");
    plt::axis("equal");
    plt::grid(true);
    plt::title("Rapid-exploration Random Tree");
    plt::pause(0.01);
}