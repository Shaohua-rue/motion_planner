#include "informed_rrt_star.h"

Informed_RRT_Star::Informed_RRT_Star(Vector2d _start, Vector2d _goal, vector<vector<double>>& obs, vector<vector<bool>>& obstacle_map_,Vector2d rand_area,
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

    c_best_ = std::numeric_limits<double>::max();
    path_cost = std::numeric_limits<double>::max();
    c_min_ = std::hypot(_start[0] - _goal[0], _start[1] - _goal[1]);

}

vector<vector<double>> Informed_RRT_Star::planning(void){
    node_list.push_back(start);
    int consecutive_improvement = 0;
    for(int iterator  = 0; iterator < max_iter; iterator++){
        shared_ptr<Node> rnd_node = get_random_node();

        shared_ptr<Node> nearest_node = get_nearest_node(*rnd_node);
        shared_ptr<Node> new_node = steer(nearest_node, rnd_node);

        if(check_collision(new_node)){
            //RRT_Star:寻找x的临近节点
            vector<int> near_node_index = get_neighbor_nodes(new_node);
            //重新选择父节点
            shared_ptr<Node> new_parent = get_new_parent(new_node, near_node_index);
            //假设父节点更新了
            if(new_parent != nullptr && check_collision(new_parent)){
                rewire(new_parent, near_node_index);
                node_list.emplace_back(new_node);
            }
            else{
                node_list.emplace_back(new_node);
            }
           
        }else
        {
            continue;
        }
        std::cout<<"当前节点"<< new_node->x << " " << new_node->y << std::endl;
        if(show_animation && (iterator % 5 == 0)) {
            draw_graph(*rnd_node);
        }
        Vector2d d_angle = calc_distance_and_angle(new_node, goal);
        if (d_angle[0] <= 1.0) {
            double current_cost = new_node->cost + d_angle[0];
            std::cout<<"当前路径成本: " << current_cost << std::endl;
            std::cout<<"最佳路径成本: " << c_best_ << std::endl;
            // 5. 更新最佳路径
            if (current_cost < c_best_) {
                c_best_ = current_cost;
                consecutive_improvement++;
                
                goal->parent = new_node;
                std::cout << "发现更优路径，成本: " << path_cost << std::endl;
            }
            
            // 6. 终止条件（连续5次优化或达到精度）
            if (consecutive_improvement >= 5 || path_cost < c_min_ * 1.1) {
                std::cout << "规划完成，最终成本: " << path_cost << std::endl;
                break;
            }
        }
    }
    return generate_final_course();

}


/***
*@brief get random node
***/
shared_ptr<Node> Informed_RRT_Star::get_random_node(void) {
    // ellipse sample
    if (c_best_ < std::numeric_limits<double>::max())
    {
        while (true)
        {
            // unit ball sample
            double x, y;
            std::random_device rd;
            std::mt19937 eng(rd());
            std::uniform_real_distribution<float> p(-1, 1);
            while (true)
            {
                x = p(eng);
                y = p(eng);
                if (x * x + y * y < 1)
                break;
            }
            // transform to ellipse
            shared_ptr<Node> temp = transform(x, y);
            if (temp->x < max_rand  && temp->x > min_rand  && temp->y < max_rand  && temp->y > min_rand)
            {
                return temp;
            }
        }
    }

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
shared_ptr<Node> Informed_RRT_Star::get_nearest_node(Node rnd_node) {
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
shared_ptr<Node>  Informed_RRT_Star::steer(shared_ptr<Node>  from_node, shared_ptr<Node>  to_node) {
    Vector2d d_angle = calc_distance_and_angle(from_node, to_node);
    double dist = std::min(expand_dis, d_angle[0]);
    shared_ptr<Node>  node_new =
        make_shared<Node>(from_node->x + dist * cos(d_angle[1]), from_node->y + dist * sin(d_angle[1]),dist + from_node->cost, from_node);

    return node_new;
}

/**
*@brief calculate distance and angle
*/
Vector2d Informed_RRT_Star::calc_distance_and_angle(shared_ptr<Node> from_node, shared_ptr<Node> to_node) {
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
bool Informed_RRT_Star::check_collision(shared_ptr<Node> new_node) {
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
vector<vector<double>> Informed_RRT_Star::generate_final_course(void) {
    vector<vector<double>> final_path(2);
    shared_ptr<Node> n = goal;

    while (n != nullptr) {
        final_path[0].emplace_back(n->x);
        final_path[1].emplace_back(n->y);
        n = n->parent;
    }

    return final_path;
}

/**
*@brief draw_graph
*/
void Informed_RRT_Star::draw_graph(Node rnd) {
    plt::clf();

    plt::plot(boundary[0], boundary[1], "sk");
    plt::plot({rnd.x}, {rnd.y}, "^k");


    plt::plot(obstacle_list[0],obstacle_list[1], "sk");

    for (shared_ptr<Node> n : node_list) {
        if (n->parent != nullptr) {
            plt::plot({n->x, n->parent->x}, {n->y, n->parent->y}, "-g");
        }
    }

    plt::plot({start->x}, {start->y}, "xr");
    plt::plot({goal->x}, {goal->y}, "xr");
    plt::axis("equal");
    plt::grid(true);
    plt::title("Rapid-exploration Random Tree");
    plt::pause(0.01);
}

/**
*@brief 获得node周围半径为r的所有节点索引
*
*/
vector<int> Informed_RRT_Star::get_neighbor_nodes(shared_ptr<Node> node)
{
    vector<int> neighbor_nodes;
    int nodeSize = node_list.size();

    double r = expand_dis * sqrt((log(nodeSize) / nodeSize));
    
    for(int i = 0; i < nodeSize; i++){
        shared_ptr<Node> n = node_list[i];
        if(hypot(n->x - node->x, n->y - node->y) < r){
            neighbor_nodes.emplace_back(i);
        }
    }
    return neighbor_nodes;
}

/**
*@brief get new parent
**/
shared_ptr<Node> Informed_RRT_Star::get_new_parent(shared_ptr<Node> new_node,vector<int> neighbor_ids)
{
    if(new_node == nullptr && neighbor_ids.size() == 0 ) return nullptr;

    vector<double> cost_list;
    for(auto index : neighbor_ids){
        shared_ptr<Node> neighbor = node_list[index];
        shared_ptr<Node> tmp_node = steer(neighbor, new_node);
        if(tmp_node && check_collision(tmp_node)){
            cost_list.emplace_back(tmp_node->cost);
        }
        else{
            cost_list.emplace_back(INFINITY);
        }
    }
    if (cost_list.empty()) {
         return nullptr;
    } 
    double min_cost = *min_element(cost_list.begin(), cost_list.end());
    int min_index = neighbor_ids[min_element(cost_list.begin(), cost_list.end())- cost_list.begin()];

    if(min_cost == INFINITY){
        std::cout<<"There is no good path.(min cost is INFINITY)";
        return nullptr;
    }

    shared_ptr<Node> new_parent = steer(node_list[min_index], new_node);
    return new_parent;
}

void Informed_RRT_Star::rewire(shared_ptr<Node> new_node,vector<int> neighbor_ids)
{
    for (int i: neighbor_ids) {
        shared_ptr<Node> near_node = node_list[i];
        shared_ptr<Node> edge_node = steer(new_node, near_node);

        if (edge_node && check_collision(edge_node) && near_node->cost > edge_node->cost) {
            near_node->x = edge_node->x;
            near_node->y = edge_node->y;
            near_node->cost = edge_node->cost;
            near_node->parent = edge_node->parent;
            propagateCostToLeaves(new_node);
        }

    }
}
void Informed_RRT_Star::propagateCostToLeaves(shared_ptr<Node>parent_node) {
    for (shared_ptr<Node> node: node_list) {
        if (node->parent == parent_node) {
            Vector2d da = calc_distance_and_angle(parent_node, node);
            node->cost = da[0] + parent_node->cost;
            propagateCostToLeaves(node);
        }
    }
}

shared_ptr<Node> Informed_RRT_Star::transform(double x, double y)
{
    // center
  double center_x = (start -> x + goal -> x) / 2;
  double center_y = (start -> y + goal -> y) / 2;

  // rotation
  double theta = -std::atan2(goal->y - start->y, goal->x - start->x);

  // ellipse
  double a = c_best_ / 2.0;
  double c = c_min_ / 2.0;
  double b = std::sqrt(std::max(0.0, a * a - c * c));

  // transform
  int tx = static_cast<int>(a * cos(theta) * x + b * sin(theta) * y + center_x);
  int ty = static_cast<int>(-a * sin(theta) * x + b * cos(theta) * y + center_y);
  return make_shared<Node>(tx, ty, 0, nullptr);
}