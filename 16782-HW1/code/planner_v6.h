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
class NodeComparator;

typedef std::shared_ptr<Node> NodePtr;
typedef std::vector<std::vector<std::vector<NodePtr>>> NodeGrid;
typedef std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator>
    OpenList;
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

  Node(Point coordinates, Point goal, int time) {
    x = coordinates.first;
    y = coordinates.second;
    t = time;

    goal_x = goal.first;
    goal_y = goal.second;

    // Chebyshev distance
    int dx = abs(x - goal_x);
    int dy = abs(y - goal_y);
    heuristic_cost_to_go_h = (dx + dy) - (dx < dy ? dx : dy);
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
};

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    return A->cost_to_go_f > B->cost_to_go_f;
  }
};

class RealTimePlanner {
public:
  RealTimePlanner(double *map, int collision_thresh, int x_size, int y_size,
                  int target_steps, double *target_traj, int robotposeX,
                  int robotposeY) {
    auto start_time = std::chrono::high_resolution_clock::now();
    map_ = map;
    collision_thresh_ = collision_thresh;
    x_size_ = x_size;
    y_size_ = y_size;
    target_steps_ = target_steps;
    target_traj_ = target_traj;

    robot_pose_ = {robotposeX - 1, robotposeY - 1};

    for (int goal_idx = target_steps - 1; goal_idx >= 0; goal_idx--) {
      int goal_x = (int)target_traj[goal_idx] - 1;
      int goal_y = (int)target_traj[goal_idx + target_steps] - 1;

      // Chebyshev distance
      int dx = abs(robot_pose_.first - goal_x);
      int dy = abs(robot_pose_.second - goal_y);
      int min_dist_to_goal = (dx + dy) - (dx < dy ? dx : dy);

      if (min_dist_to_goal < goal_idx + 1) {
        goal_ = {(int)target_traj[goal_idx] - 1,
                 (int)target_traj[goal_idx + target_steps] - 1};

        break;
      }
    }

    for (double percent = 0; percent < 2.2; percent += 0.25) {
      std::cout << "Trying with transition cost: " << percent * collision_thresh
                << "\n";

      std::vector<NodePtr> time(1, nullptr);
      std::vector<std::vector<NodePtr>> time_and_y(y_size_, time);
      node_grid_ = NodeGrid(x_size_, time_and_y);

      NodePtr start = std::make_shared<Node>(
          Point(robotposeX - 1, robotposeY - 1), goal_, 0);
      start->cost_to_come_g = 0;
      start->cost_to_go_f = start->heuristic_cost_to_go_h;

      node_grid_[start->x][start->y][0] = start;

      OpenList open_list;
      open_list.push(start);

      expandStates(open_list, percent * collision_thresh);
      commands.clear();
      constructPathFromPlan();
      int steps_to_goal = commands.size() - 1;
      auto time_elapsed =
          (std::chrono::high_resolution_clock::now() - start_time);
      int seconds_time_elapsed =
          std::chrono::duration_cast<std::chrono::seconds>(time_elapsed)
              .count();
      std::cout << "Need to catch the target within "
                << (target_steps - 1 - seconds_time_elapsed) << "s\n";
      if (steps_to_goal < (target_steps - 1 - seconds_time_elapsed)) {
        std::cout << "Phew! can reach the target on time.\n";
        break;
      } else {
        std::cout << "Uh oh! needs " << steps_to_goal << "s to catch the target.\n";
      }
    }
  }

  void expandStates(OpenList &open_list, double transition_cost) {
    while (!open_list.empty()) {
      NodePtr curr_node = open_list.top();
      open_list.pop();
      curr_node->is_on_open_list = false;

      Diff directions[8] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};

      bool goal_found = false;

      for (int dir = 0; dir < 8; dir++) {
        int neighbor_x = curr_node->x + directions[dir].first;
        int neighbor_y = curr_node->y + directions[dir].second;

        if (getMapData(neighbor_x, neighbor_y) >= collision_thresh_) {
          continue;
        }

        NodePtr neighbor;
        if ((node_grid_[neighbor_x][neighbor_y][0] == nullptr)) {
          node_grid_[neighbor_x][neighbor_y][0] =
              std::make_shared<Node>(Point(neighbor_x, neighbor_y), goal_, 0);
        }

        neighbor = node_grid_[neighbor_x][neighbor_y][0];

        double tentative_cost_to_come_g = curr_node->cost_to_come_g +
                                          getMapData(neighbor->x, neighbor->y) +
                                          transition_cost;

        if (tentative_cost_to_come_g < neighbor->cost_to_come_g) {
          neighbor->setParent(curr_node);
          neighbor->cost_to_come_g = tentative_cost_to_come_g;
          neighbor->cost_to_go_f =
              neighbor->cost_to_come_g + neighbor->heuristic_cost_to_go_h;

          if (neighbor->x == goal_.first && neighbor->y == goal_.second) {
            goal_found = true;
            break;
          }

          if (!neighbor->is_on_open_list) {
            open_list.push(neighbor);
            neighbor->is_on_open_list = true;
          }
        }
      }

      if (goal_found) {
        break;
      }
    }
  }

  void constructPathFromPlan() {
    NodePtr curr_node = node_grid_[goal_.first][goal_.second][0];

    if (curr_node == nullptr)
      return;

    while (curr_node->parent != nullptr) {
      commands.push_back({curr_node->x + 1, curr_node->y + 1});
      curr_node = curr_node->parent;
    }
  }

  std::vector<Point> commands;

private:
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

    return map_[(y * x_size_ + x)];
  }

  inline Diff diffFromAToB(NodePtr A, NodePtr B) {
    return {B->x - A->x, B->y - A->y};
  }
};

#endif // REAL_TIME_PLANNER_H