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

  double *target_trajectory;
  int total_time;

  Node(Point coordinates, int current_time, int target_steps,
       double *target_traj) {
    x = coordinates.first;
    y = coordinates.second;
    t = current_time;

    total_time = target_steps;
    target_trajectory = target_traj;

    heuristic_cost_to_go_h = heuristicFunction();
  }

  Node() {
    x = DUMMY_GOAL_COORD;
    y = DUMMY_GOAL_COORD;
    t = -1;
    heuristic_cost_to_go_h = 0;
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

  double heuristicFunction() {
    double result =  __DBL_MAX__;
    for (int i = 0; i < total_time; i++) {
      int manhattan_distance = abs(x - target_trajectory[i]) +
                               abs(y - target_trajectory[total_time + i]);

      int min_time_required = manhattan_distance;
      int time_remaining = i - t;

      if (min_time_required < time_remaining) {
        continue;
      }

      if (min_time_required < result) {
        result = min_time_required;
      }
    }

    return 300*result;
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

    std::list<NodePtr> time;
    std::vector<std::list<NodePtr>> time_and_y(y_size_, time);
    node_grid_ = NodeGrid(x_size_, time_and_y);

    NodePtr start = std::make_shared<Node>(
        Point(robotposeX - 1, robotposeY - 1), 0, target_steps_, target_traj_);
    start->cost_to_come_g = 0;
    start->cost_to_go_f = start->heuristic_cost_to_go_h;

    addNode(start);

    open_list_.push(start);

    expandStates(std::chrono::milliseconds(1000000));
    constructPathFromPlan();
  }

  bool expandStates(std::chrono::milliseconds time_allowed) {
    if (open_list_.empty()) {
      return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed = start_time - start_time;

    while (time_elapsed < time_allowed && !open_list_.empty()) {
      NodePtr curr_node = open_list_.top();
      open_list_.pop();
      curr_node->is_on_open_list = false;

      std::vector<Diff> directions;

      if (curr_node->t < target_steps_) {
        directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1},
                      {1, -1},  {1, 0},  {1, 1},  {0, 0}};
      }

      bool goal_found = false;

      if (isGoalNode(curr_node)) {
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
          continue;
        }

        NodePtr neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
        if ((neighbor == nullptr)) {
          addNode(std::make_shared<Node>(Point(neighbor_x, neighbor_y),
                                         curr_node->t + 1, target_steps_,
                                         target_traj_));
          neighbor = getNodeAt(neighbor_x, neighbor_y, curr_node->t + 1);
        }

        double tentative_cost_to_come_g =
            curr_node->cost_to_come_g + getMapData(neighbor->x, neighbor->y);

        if (tentative_cost_to_come_g < neighbor->cost_to_come_g) {
          neighbor->setParent(curr_node);
          neighbor->cost_to_come_g = tentative_cost_to_come_g;
          neighbor->cost_to_go_f =
              neighbor->cost_to_come_g + neighbor->heuristic_cost_to_go_h;

          if (neighbor->x == DUMMY_GOAL_COORD ||
              neighbor->y == DUMMY_GOAL_COORD) {
            goal_found = true;
            break;
          }

          if (!neighbor->is_on_open_list) {
            open_list_.push(neighbor);
            neighbor->is_on_open_list = true;
          }
        }
      }

      if (goal_found) {
        break;
      }

      time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
    }

    time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
    return true;
  }

  void constructPathFromPlan() {
    NodePtr curr_node =
        getNodeAt(DUMMY_GOAL_COORD, DUMMY_GOAL_COORD, target_steps_ - 5)
            ->parent;

    if (curr_node == nullptr) {
      return;
    }

    while (curr_node->parent != nullptr) {
      commands.push_back({curr_node->x + 1, curr_node->y + 1});
      curr_node = curr_node->parent;
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
    if (x == DUMMY_GOAL_COORD || y == DUMMY_GOAL_COORD) {
      return 0;
    }

    if (x >= x_size_ || x < 0 || y >= y_size_ || y < 0)
      return __DBL_MAX__;

    return map_[(y * x_size_ + x)];
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
      if ((*it_)->t == t) {
        return *it_;
      }
    }

    return nullptr;
  }

  void addNode(NodePtr node) { node_grid_[node->x][node->y].push_back(node); }

  bool isGoalNode(NodePtr node) {
    if (node->t >= target_steps_) {
      return false;
    }

    // (int)target_traj[target_steps - 1] - 1
    int max_time = (target_steps_-10 > 5) ? 5 : target_steps_-10;
    int curr_t_target = (node->t + max_time) > 0 ? (node->t + max_time) : 0;
    int target_x = ((int)target_traj_[curr_t_target]) - 1;
    int target_y = ((int)target_traj_[curr_t_target + target_steps_]) - 1;

    bool x_pos_correct = (target_x == node->x);
    bool y_pos_correct = (target_y == node->y);
    bool no_collision = getMapData(node->x, node->y) < collision_thresh_;

    return x_pos_correct && y_pos_correct && no_collision;
  }
};

#endif // REAL_TIME_PLANNER_H