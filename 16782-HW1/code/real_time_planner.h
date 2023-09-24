#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

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
typedef std::vector<std::vector<std::vector<NodePtr>>> NodeGrid;
typedef std::pair<int, int> Point;
typedef Point Diff;

struct Node : public std::enable_shared_from_this<Node> {
  double cost_to_come_g = __DBL_MAX__;
  double cost_to_go_f = __DBL_MAX__;
  double heuristic_cost_to_go_h = __DBL_MAX__;

  NodePtr parent = nullptr;
  std::vector<NodePtr> children;

  int x;
  int y;
  int t;

  int goal_x;
  int goal_y;

  bool is_on_open_list = false;

  // data = <x, y, t>, goal = <goal_x, goal_y>
  Node(Point coordinates, Point goal, int time) {
    x = coordinates.first;
    y = coordinates.second;
    t = time;

    goal_x = goal.first;
    goal_y = goal.second;

    heuristic_cost_to_go_h =
        sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y));
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
};

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    // Return true if A > B

    return A->cost_to_go_f > B->cost_to_go_f;
  }
};

class RealTimePlanner {
public:
  RealTimePlanner(double *map, int collision_thresh, int x_size, int y_size,
                  int target_steps, double *target_traj, int robotposeX,
                  int robotposeY) {
    map_ = map;
    collision_thresh_ = collision_thresh;
    x_size_ = x_size;
    y_size_ = y_size;
    target_steps_ = target_steps;
    target_traj_ = target_traj;

    robot_pose_ = {robotposeX - 1, robotposeY - 1};
    goal_ = {(int)target_traj[target_steps - 1] - 1,
             (int)target_traj[target_steps - 1 + target_steps] - 1};

    std::cout << "Goal is to reach " << goal_.first << ", " << goal_.second
              << "\n";

    std::vector<NodePtr> time(target_steps_, nullptr);
    std::vector<std::vector<NodePtr>> time_and_y(y_size_, time);
    node_grid_ = NodeGrid(x_size_, time_and_y);

    std::cout << "Node grid size: " << node_grid_.size() << ", "
              << node_grid_[0].size() << ", " << node_grid_[0][0].size()
              << "\n";

    NodePtr start =
        std::make_shared<Node>(Point(robotposeX - 1, robotposeY - 1), goal_, 0);
    start->cost_to_come_g = 0;
    start->cost_to_go_f = start->heuristic_cost_to_go_h;

    node_grid_[start->x][start->y][0] = start;

    open_list_.push(start);

    std::cout << "The size of the open list is" << open_list_.size() << "\n";
    std::cout << "The top most element is: \n";
    open_list_.top()->printNodeInfo();

    expandStates(std::chrono::milliseconds(1000000));
    constructPathFromPlan();
  }

  bool expandStates(std::chrono::milliseconds time_allowed) {
    if (open_list_.empty()) {
      return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed = start_time - start_time;

    std::cout << "Expanding states\n";
    while (time_elapsed < time_allowed && !open_list_.empty()) {
      NodePtr curr_node = open_list_.top();
      open_list_.pop();
      curr_node->is_on_open_list = false;

      std::cout << "Node expanded:\n";
      curr_node->printNodeInfo();

      Diff directions[8] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};

      // std::cout << "Checking this node's neighbor\n";
      bool goal_found = false;

      for (int dir = 0; dir < 8; dir++) {
        int neighbor_x = curr_node->x + directions[dir].first;
        int neighbor_y = curr_node->y + directions[dir].second;

        if (getMapData(neighbor_x, neighbor_y) >= collision_thresh_) {
          std::cout << "result - " << getMapData(neighbor_x, neighbor_y)
                    << "\n";
          continue;
        }

        NodePtr neighbor;
        if ((node_grid_[neighbor_x][neighbor_y][0] == nullptr)) {
          node_grid_[neighbor_x][neighbor_y][0] =
              std::make_shared<Node>(Point(neighbor_x, neighbor_y), goal_, 0);
        }

        neighbor = node_grid_[neighbor_x][neighbor_y][0];
        std::cout << "Currently checking neighbor:\n";
        neighbor->printNodeInfo();

        double tentative_cost_to_come_g =
            curr_node->cost_to_come_g + getMapData(neighbor->x, neighbor->y);

        std::cout << "tentative g cost: " << tentative_cost_to_come_g
                  << "; neighbor's g cost" << neighbor->cost_to_come_g << "\n";

        if (tentative_cost_to_come_g < neighbor->cost_to_come_g) {
          // std::cout << "Setting the parent of this neigbor to the current
          // node\n";

          neighbor->setParent(curr_node);
          // std::cout << "Set the parent!\n";
          neighbor->cost_to_come_g = tentative_cost_to_come_g;
          neighbor->cost_to_go_f =
              neighbor->cost_to_come_g + neighbor->heuristic_cost_to_go_h;

          std::cout << "Now the neigbor is\n";
          neighbor->printNodeInfo();

          if (neighbor->x == goal_.first && neighbor->y == goal_.second) {
            std::cout << "Found the goal!\n";
            goal_found = true;
            break;
          }

          if (!neighbor->is_on_open_list) {
            std::cout
                << "This neighbor is going to be added to the open list\n";
            open_list_.push(neighbor);
            neighbor->is_on_open_list = true;
          }
        }
      }

      if (goal_found) {
        break;
      }

      std::cout << "Current size of the openset: " << open_list_.size() << "\n";

      // if (open_list_.size() > 20) {
      //   std::cout << "Open set is too large! breaking\n";
      //   break;
      // }

      std::cout << "-----------------------------------------------------------"
                   "----------------\n";

      time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
    }

    return true;
  }

  // void updateHValues() {}

  // void decideNextMove() {}

  void constructPathFromPlan() {
    NodePtr curr_node = node_grid_[goal_.first][goal_.second][0];

    if (curr_node == nullptr)
      return;

    std::cout << "Reconstructing path backwards from:\n";
    curr_node->printNodeInfo();

    while (curr_node->parent != nullptr) {
      commands.push_back({curr_node->x + 1, curr_node->y + 1});
      curr_node = curr_node->parent;
      std::cout << "Added pose - " << commands.back().first << ", " << commands.back().second << "\n";
    }
  }

  std::vector<Point> commands;

private:
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_list_;
  std::list<NodePtr> closed_list_;
  NodeGrid node_grid_;

  double *map_;
  int x_size_;
  int y_size_;
  int collision_thresh_;

  int target_steps_;
  double *target_traj_;

  Point robot_pose_;

  Point goal_;

  double getMapData(int x, int y) {
    if (x >= x_size_ || x < 0 || y >= y_size_ || y < 0)
      return __DBL_MAX__;

    // std::cout << "getting map data of " << x << ", " << y
    //           << "; : Flattened index is - " << ((y - 1) * x_size_ + (x - 1))
    //           << ", The value is - " << map_[((y - 1) * x_size_ + (x - 1))]
    //           << "\n";
    return map_[(y * x_size_ + x)];
  }

  inline Diff diffFromAToB(NodePtr A, NodePtr B) {
    return {B->x - A->x, B->y - A->y};
  }
};

#endif // REAL_TIME_PLANNER_H