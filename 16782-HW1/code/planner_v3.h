#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#define DUMMY_GOAL_COORD -25

#include <algorithm>
#include <chrono>
#include <iostream>
#include <list>
#include <math.h>
#include <memory>
#include <queue>
#include <vector>

struct Node;

typedef std::shared_ptr<Node> NodePtr;
typedef std::vector<std::vector<std::list<NodePtr>>> NodeGrid;
typedef std::pair<int, int> Point;
typedef Point Diff;

struct Node : public std::enable_shared_from_this<Node> {
  double g = __DBL_MAX__;
  double f = __DBL_MAX__;
  double h = __DBL_MAX__;

  NodePtr parent = nullptr;
  std::vector<NodePtr> children;

  int x;
  int y;
  int t;

  int goal_x;
  int goal_y;

  bool is_on_open_list = false;

  int *target_trajectory;
  int total_time;

  int max_possible_cost;
  double max_distance_possible;

  // data = <x, y, t>, goal = <goal_x, goal_y>
  Node(Point coordinates, int current_time, int target_steps,
       int *target_traj, int collision_thresh, double diagonal_length) {
    x = coordinates.first;
    y = coordinates.second;
    t = current_time;

    total_time = target_steps;
    target_trajectory = target_traj;
    max_possible_cost = collision_thresh;
    max_distance_possible = diagonal_length;
    // goal_x = goal.first;
    // goal_y = goal.second;

    h = heuristicFunction();
  }

  Node() {
    x = DUMMY_GOAL_COORD;
    y = DUMMY_GOAL_COORD;
    t = -1;
    h = 0;
  }

  void setParent(NodePtr parent_to_set) {
    NodePtr this_node_ptr = shared_from_this();

    if (parent != nullptr) {
      auto this_node_it = std::find(parent->children.begin(),
                                    parent->children.end(), this_node_ptr);
      parent->children.erase(this_node_it);
    }

    parent = parent_to_set;

    parent->children.push_back(this_node_ptr);
  }

  void printNodeInfo() {
    std::cout << "x, y, t: " << x << ", " << y << ", " << t << "\n";
    if (parent != nullptr) {
      std::cout << "Parent is (x, y, t): " << parent->x << ", " << parent->y
                << ", " << parent->t << "\n\n";

    } else {
      std::cout << "This node doesn't have a parent!\n\n";
    }
  }

  double heuristicFunction() {
    // double result =
    //     sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y));

    // int max_time_to_reach = goal_time - t;
    // if ((result / max_time_to_reach) > 1.0)
    //   result = __DBL_MAX__;

    // return result;

    // double result = __DBL_MAX__;
    // for (int i = 0; i < total_time; i++) {
    //   int manhattan_distance = abs(x - target_trajectory[i]) +
    //                            abs(y - target_trajectory[total_time + i]);

    //   int min_time_required = manhattan_distance;
    //   int time_remaining = i - t;

    //   if (min_time_required < time_remaining) {
    //     continue;
    //   }

    //   if (min_time_required < result) {
    //     result = min_time_required;
    //   }
    // }

    // return 10*result;
    // ---------------------------------------------------------------------------------
    // double result = __DBL_MAX__;

    // int num_points_reachable = 0;
    // int avg_distance = 0;
    // for (int i = 0; i < total_time; i++) {
    //   int manhattan_distance = abs(x - target_trajectory[i]) +
    //                            abs(y - target_trajectory[total_time + i]);

    //   int min_time_required = manhattan_distance;
    //   int time_remaining = i - t;
    //   double min_velocity_to_reach =
    //       (1.0 * manhattan_distance) / time_remaining;

    //   if (min_time_required < time_remaining) {
    //     // std::cout << "Does this ever run?\n";
    //     continue;
    //   }

    //   num_points_reachable++;
    //   avg_distance += manhattan_distance;
    // }

    // // if (num_points_reachable < 0.05 * (total_time - t))
    // //   return __DBL_MAX__;
    // // std::cout << "number of reachable points" << num_points_reachable <<
    // "\n"; if (num_points_reachable == 0) {
    //   return __DBL_MAX__;
    // }
    // avg_distance /= num_points_reachable;

    // return 100*(avg_distance * max_possible_cost) / max_distance_possible;
    // -------------------------------------------------------------------------------

    // double result = __DBL_MAX__;

    int num_points_reachable = 0;
    // int avg_distance = 0;
    for (int i = t; i < total_time; i++) {
      int manhattan_distance = abs(x - target_trajectory[i]) +
                               abs(y - target_trajectory[total_time + i]);

      int dx = abs(x - target_trajectory[i]);
      int dy = abs(y - target_trajectory[total_time + i]);

      int chebyshev_distance = (dx + dy) - (dx < dy ? dx : dy);

      int min_time_required = chebyshev_distance;
      int time_remaining = i - t;
      // double min_velocity_to_reach =
      // (1.0 * manhattan_distance) / time_remaining;

      if (min_time_required < time_remaining) {
        // std::cout << "Does this ever run?\n";
        // std::cout << "mtr " << min_time_required << ", time_r " << time_remaining << "\n";
        return 0;
      }

      // num_points_reachable++;
      // avg_distance += manhattan_distance;
    }

    // if (num_points_reachable < 0.05 * (total_time - t))
    //   return __DBL_MAX__;
    // std::cout << "number of reachable points" << num_points_reachable <<
    // "\n"; if (num_points_reachable == 0) { return __DBL_MAX__;
    // }
    // avg_distance /= num_points_reachable;

    // return 100*(avg_distance * max_possible_cost) / max_distance_possible;
    return __DBL_MAX__;
  }
};

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    // Return true if A > B

    return A->f > B->f;
  }
};

class RealTimePlanner {
public:
  RealTimePlanner(int *map, int collision_thresh, int x_size, int y_size,
                  int target_steps, int *target_traj, int robotposeX,
                  int robotposeY) {
    map_ = map;
    collision_thresh_ = collision_thresh;
    x_size_ = x_size;
    y_size_ = y_size;
    diagonal_length = sqrt(x_size_ * x_size_ + y_size_ * y_size_);
    target_steps_ = target_steps;
    target_traj_ = target_traj;

    robot_pose_ = {robotposeX - 1, robotposeY - 1};
    goal_ = {(int)target_traj[target_steps - 1] - 1,
             (int)target_traj[target_steps - 1 + target_steps] - 1};

    std::cout << "Goal is to reach " << goal_.first << ", " << goal_.second
              << ", " << target_steps << "\n";

    std::cout << "Initializing 3d vector. ";

    std::list<NodePtr> time;
    std::vector<std::list<NodePtr>> time_and_y(y_size_, time);
    node_grid_ = NodeGrid(x_size_, time_and_y);
    std::cout << "Done initializing\n";
    // std::cout << "Node grid size: " << node_grid_.size() << ", "
    //           << node_grid_[0].size() << ", " << node_grid_[0][0].size()
    //           << "\n";

    NodePtr start = std::make_shared<Node>(
        Point(robotposeX - 1, robotposeY - 1), 0, target_steps_, target_traj_,
        collision_thresh_, diagonal_length);
    start->g = 0;
    start->f = start->h;

    addNode(start);

    open_list_.push(start);

    std::cout << "The size of the open list is" << open_list_.size() << "\n";
    std::cout << "The top most element is: \n";
    open_list_.top()->printNodeInfo();

    // expandStates(std::chrono::milliseconds(500));
    // constructPathFromPlan();
  }

  bool expandStates(std::chrono::milliseconds time_allowed,
                    int curr_time_step) {
    if (open_list_.empty()) {
      return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed = start_time - start_time;

    std::cout << "Expanding states\n";
    std::vector<NodePtr> online_closed_list;
    static bool goal_found = false;

    std::cout << "Is goal found?" << goal_found << "\n";
    while (time_elapsed < time_allowed && !open_list_.empty() && !goal_found) {
      NodePtr curr_node = open_list_.top();
      open_list_.pop();
      online_closed_list.push_back(curr_node);
      curr_node->is_on_open_list = false;
      if (curr_node->t < curr_time_step) {
        // std::cout <<"Node is useless lol\n";
        continue;
      }

      // std::cout << "Node expanded:\n";
      // curr_node->printNodeInfo();

      std::vector<Diff> directions;

      if (curr_node->t < target_steps_) {
        directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1},
                      {1, -1},  {1, 0},  {1, 1},  {0, 0}};
      }

      // std::cout << "Checking this node's neighbor\n";

      if (isGoalNode(curr_node)) {
        // std::cout << "Yes, this is a goal node\n";
        directions = {{-2, -2}};
      }

      for (int dir = 0; dir < directions.size(); dir++) {
        int neighbor_x;
        int neighbor_y;

        if (directions[dir].first != -2) {
          neighbor_x = curr_node->x + directions[dir].first;
          neighbor_y = curr_node->y + directions[dir].second;
        } else {
          neighbor_x = DUMMY_GOAL_COORD;
          neighbor_y = DUMMY_GOAL_COORD;
        }

        if (getMapData(neighbor_x, neighbor_y) >= collision_thresh_) {
          // std::cout << "result - " << getMapData(neighbor_x, neighbor_y)
          // << "\n";
          continue;
        }

        NodePtr neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
        if ((neighbor == nullptr)) {
          bool within_bounds = addNode(std::make_shared<Node>(
              Point(neighbor_x, neighbor_y), curr_node->t + 1, target_steps_,
              target_traj_, collision_thresh_, diagonal_length));

          if (!within_bounds) {
            std::cout << "Invalid neighbor, goes out of bound\n";
            continue;
          }
          // std::cout << "Node added\n";
          neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
        }

        // std::cout << "Currently checking neighbor:\n";
        // neighbor->printNodeInfo();

        double tentative_g =
            curr_node->g + getMapData(neighbor->x, neighbor->y);

        // std::cout << "tentative g cost: " << tentative_g
        // << "; neighbor's g cost" << neighbor->g << "\n";

        if (tentative_g < neighbor->g) {
          // std::cout << "Setting the parent of this neigbor to the current
          // node\n";

          neighbor->setParent(curr_node);
          // std::cout << "Set the parent!\n";
          neighbor->g = tentative_g;
          neighbor->f = neighbor->g + neighbor->h;

          // std::cout << "Now the neigbor is\n";
          // neighbor->printNodeInfo();

          if (neighbor->x == DUMMY_GOAL_COORD ||
              neighbor->y == DUMMY_GOAL_COORD) {
            // std::cout << "Found the goal!\n";
            goal_found = true;
            break;
          }

          if (!neighbor->is_on_open_list) {
            // std::cout
            // << "This neighbor is going to be added to the open list\n";
            // neighbor->printNodeInfo();
            open_list_.push(neighbor);
            neighbor->is_on_open_list = true;
          }
        }
      }

      // if (open_list_.size() > 40) {
      //   std::cout << "Open set is too large! breaking\n";
      //   break;
      // }

      time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
    }

    time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
    // std::cout << "TOTAL TIME: "
    // << std::chrono::duration<double>(time_elapsed).count() << "\n";
    std::cout << "Current size of the openset: " << open_list_.size() << "\n";
    std::cout << "-----------------------------------------------------------"
                 "----------------\n";
    updateHValues(online_closed_list);

    return true;
  }

  Point decideNextMove(int robot_pose_x, int robot_pose_y, int curr_time) {
    // printValues(curr_time + 1);
    std::cout << "Starting next move calc\n";
    NodePtr curr_node =
        getNodeAt(robot_pose_x - 1, robot_pose_y - 1, curr_time);
    std::cout << "Curr node found\n";
    // std::cout << "Checking for node (x,y,t) " << robot_pose_x - 1 << ", "
    // << robot_pose_y - 1<< ", " << curr_time << "\n";
    if (curr_node == nullptr) {
      std::cout << "Current node is NULL! How can this be?\n";
      addNode(std::make_shared<Node>(Point(robot_pose_x - 1, robot_pose_y - 1),
                                     curr_time, target_steps_, target_traj_,
                                     collision_thresh_, diagonal_length));
      curr_node = getNodeAt(robot_pose_x - 1, robot_pose_y - 1, curr_time);
      // return {robot_pose_x, robot_pose_y};
    }

    std::vector<Diff> directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1},
                                    {1, -1},  {1, 0},  {1, 1},  {0, 0}};

    Point next_move(robot_pose_x, robot_pose_y);
    double next_move_cost = __DBL_MAX__;

    for (int i = 0; i < directions.size(); i++) {
      int neighbor_x = curr_node->x + directions[i].first;
      int neighbor_y = curr_node->y + directions[i].second;
      NodePtr neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
      if (neighbor == nullptr) {
        bool within_bounds = addNode(std::make_shared<Node>(
            Point(neighbor_x, neighbor_y), curr_node->t + 1, target_steps_,
            target_traj_, collision_thresh_, diagonal_length));

        if (!within_bounds) {
          std::cout << "Invalid neighbor, goes out of bound\n";
          continue;
        }

        neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
      }

      std::cout << "Checking neighbor- " << neighbor_x << ", " << neighbor_y
                << "\n";
      double neighbor_move_cost = (neighbor->h == __DBL_MAX__)
                                   ? __DBL_MAX__
                                   : (neighbor->g + neighbor->h);
      std::cout << "Cost from that neighbor is- (g, h, f)" << neighbor->g << ","
                << neighbor->h << ", " << neighbor_move_cost << "\n";

      if (next_move_cost > neighbor_move_cost) {
        next_move_cost = neighbor_move_cost;
        next_move = Point(neighbor->x + 1, neighbor->y + 1);
      }
    }

    std::cout << "Next move is: " << next_move.first << ", " << next_move.second
              << "\n";
    std::cout << "Heuristic to reach destination for this is " << next_move_cost
              << "\n";

    return next_move;

    // NodePtr lowest_cost_child = nullptr;
    // double min_cost = __DBL_MAX__;

    // for (auto child : curr_node->children) {
    //   // std::cout << "Checking child " << child->x << ", " << child->y << ",
    //   "
    //   // << child->t << "\n"; std::cout << "This child has f cost= " << if
    //   // (lowest_cost_child == nullptr) {
    //   //   lowest_cost_child = child;
    //   //   min_cost = lowest_cost_child->g + lowest_cost_child->h;
    //   //   continue;
    //   // }

    //   if (min_cost > (child->g + child->h)) {
    //     min_cost = child->g + child->h;
    //     lowest_cost_child = child;
    //   }
    // }

    // if (lowest_cost_child == nullptr) {
    //   std::cout << "No children lol. Robot stuck\n";
    //   return {robot_pose_x, robot_pose_y};
    // }
    // std::cout << "Next move is: " << lowest_cost_child->x + 1 << ", "
    //           << lowest_cost_child->y + 1 << "\n";
    // std::cout << "Heuristic to reach destination for this is " <<
    // lowest_cost_child->h << "\n"; return {lowest_cost_child->x + 1,
    // lowest_cost_child->y + 1};
  }

  void constructPathFromPlan() {
    // NodePtr curr_node = getNodeAt(goal_.first, goal_.second, target_steps_ -
    // 5);
    NodePtr curr_node =
        getNodeAt(DUMMY_GOAL_COORD, DUMMY_GOAL_COORD, target_steps_ - 5)
            ->parent;

    if (curr_node == nullptr) {
      // std::cout << "No parent lol\n";
      return;
    }

    // std::cout << "Reconstructing path\n";
    // curr_node->printNodeInfo();

    while (curr_node->parent != nullptr) {
      commands.push_back({curr_node->x + 1, curr_node->y + 1});
      // std::cout << "Added pose - " << commands.back().first << ", "
      // << commands.back().second << "\n";

      // std::cout << "Added node to plan\n";
      // curr_node->printNodeInfo();
      curr_node = curr_node->parent;
    }
  }

  std::vector<Point> commands;

private:
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_list_;
  std::list<NodePtr> closed_list_;
  NodeGrid node_grid_;

  int *map_;
  int x_size_;
  int y_size_;
  int collision_thresh_;
  double diagonal_length;

  int target_steps_;
  int *target_traj_;

  Point robot_pose_;

  Point goal_;

  // void updateHValues(std::vector<NodePtr> &closed_list) {
  //   for (auto it_ = closed_list.rbegin(); it_ != closed_list.rend(); it_++) {
  //     // std::cout << "Is it updating?";
  //     (*it_)->h = __DBL_MAX__;
  //     for (auto child : (*it_)->children) {
  //       if ((*it_)->h > (child->h + getMapData(child->x, child->y))) {
  //         (*it_)->h = (child->h + getMapData(child->x, child->y));
  //       }
  //     }
  //   }
  // }

  void updateHValues(std::vector<NodePtr> &closed_list) {
    for (auto it_ = closed_list.rbegin(); it_ != closed_list.rend(); it_++) {
      // std::cout << "Is it updating?";
      (*it_)->h = open_list_.top()->f - (*it_)->h;
      // for (auto child : (*it_)->children) {
      //   if ((*it_)->h > (child->h + getMapData(child->x, child->y))) {
      //     (*it_)->h = (child->h + getMapData(child->x, child->y));
      //   }
      // }
    }
  }

  void printValues(int current_time) {
    for (int i = 0; i < x_size_; i++) {
      for (int j = 0; j < y_size_; j++) {
        NodePtr curr_node = getNodeAt(i, j, current_time);
        if (curr_node == nullptr) {
          std::cout << "null  ";
          continue;
        }

        std::cout << curr_node->g + curr_node->h << " ";
      }

      std::cout << "\n";
    }
  }

  double getMapData(int x, int y) {
    if (x == DUMMY_GOAL_COORD || y == DUMMY_GOAL_COORD) {
      return 0;
    }

    if (x >= x_size_ || x < 0 || y >= y_size_ || y < 0)
      return __DBL_MAX__;

    // std::cout << "getting map data of " << x << ", " << y
    //           << "; : Flattened index is - " << ((y - 1) * x_size_ + (x - 1))
    //           << ", The value is - " << map_[((y - 1) * x_size_ + (x - 1))]
    //           << "\n";
    return map_[(y * x_size_ + x)] + 0.01;
  }

  inline Diff diffFromAToB(NodePtr A, NodePtr B) {
    return {B->x - A->x, B->y - A->y};
  }

  NodePtr getNodeAt(int x, int y, int t) {
    if (x == DUMMY_GOAL_COORD || y == DUMMY_GOAL_COORD) {
      static NodePtr dummy_goal = std::make_shared<Node>();
      return dummy_goal;
    }

    if (x >= x_size_ || x < 0 || y >= y_size_ || y < 0) {
      return nullptr;
    }

    auto it_ = node_grid_[x][y].begin();
    for (; it_ != node_grid_[x][y].end(); it_++) {
      // std::cout << "Checking time: " << (*it_)->t << "\n";
      if ((*it_)->t == t) {
        return *it_;
      }
    }

    return nullptr;
  }

  bool addNode(NodePtr node) {
    if (node->x >= x_size_ || node->x < 0 || node->y >= y_size_ ||
        node->y < 0) {
      return false;
    }

    node_grid_[node->x][node->y].push_back(node);
    return true;
  }

  bool isGoalNode(NodePtr node) {
    if (node->t >= target_steps_) {
      return false;
    }

    // (int)target_traj[target_steps - 1] - 1
    int max_time = (target_steps_ - 10 > 5) ? 5 : target_steps_ - 10;
    int curr_t_target = (node->t + max_time) > 0 ? (node->t + max_time) : 0;
    int target_x = ((int)target_traj_[curr_t_target]) - 1;
    int target_y = ((int)target_traj_[curr_t_target + target_steps_]) - 1;

    // std::cout << "Goal at current time t=" << node->t << " is " << target_x/
    // << ", " << target_y << "\n";
    bool x_pos_correct = (target_x == node->x);
    bool y_pos_correct = (target_y == node->y);
    bool no_collision = getMapData(node->x, node->y) < collision_thresh_;

    return x_pos_correct && y_pos_correct && no_collision;
    // return target_traj_
  }
};

#endif // REAL_TIME_PLANNER_H