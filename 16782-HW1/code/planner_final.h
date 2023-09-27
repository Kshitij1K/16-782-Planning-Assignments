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

typedef std::chrono::milliseconds milliseconds;

struct Node {
  double cost_to_come_g = __DBL_MAX__;
  double cost_to_go_f = __DBL_MAX__;
  double heuristic_cost_to_go_h = __DBL_MAX__;

  NodePtr parent = nullptr;
  // std::vector<NodePtr> children;

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

  void setParent(NodePtr parent_to_set) { parent = parent_to_set; }
};

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    return A->cost_to_go_f > B->cost_to_go_f;
  }
};

class RealTimePlanner {
public:
  // TODO
  bool plan(int time_for_planning) {
    auto start_time = std::chrono::high_resolution_clock().now();

    static bool iteration_started = false;
    static double percent = 0;

    static int num_iterations = 0;

    for (; percent < 1.2; percent += 0.25) {
      num_iterations++;

      if (!iteration_started) {
        std::vector<NodePtr> time(1, nullptr);
        std::vector<std::vector<NodePtr>> time_and_y(y_size_, time);
        node_grid_ = NodeGrid(x_size_, time_and_y);
        open_list_ = OpenList();

        node_grid_[robot_pose_.first][robot_pose_.second][0] =
            std::make_shared<Node>(robot_pose_, goal_, 0);
        node_grid_[robot_pose_.first][robot_pose_.second][0]->cost_to_come_g =
            0;
        node_grid_[robot_pose_.first][robot_pose_.second][0]->cost_to_go_f =
            node_grid_[robot_pose_.first][robot_pose_.second][0]
                ->heuristic_cost_to_go_h;

        open_list_.push(node_grid_[robot_pose_.first][robot_pose_.second][0]);
        iteration_started = true;
      }

      auto current_time = std::chrono::high_resolution_clock::now();
      auto time_elapsed =
          std::chrono::duration_cast<milliseconds>(current_time - start_time);
      int time_remaining = time_for_planning - time_elapsed.count();
      bool iteration_complete =
          expandStates(percent * collision_thresh_, time_remaining);

      if (iteration_complete) {
        iteration_started = false;
        commands.clear();
        constructPathFromPlan();
        int steps_to_goal = int(commands.size()) - 1;

        if (steps_to_goal >= 0 && steps_to_goal <= max_plan_length_) {
          auto current_time = std::chrono::high_resolution_clock::now();
          auto time_elapsed = std::chrono::duration_cast<milliseconds>(
              current_time - start_time);
          time_taken_for_planning += time_elapsed.count();
          time_taken_for_planning /= num_iterations;
          return true;
        }
      } else {
        return false;
      }
    }

    return true;
  }

  bool isMyPlanBetter(std::vector<Point> &original_plan, int start_index,
                      int target_reach_time) {
    if (commands.size() == 0) {
      return false;
    }

    // Calculate cost of original plan
    double original_cost = 0;
    int num_points_iterated = 0;
    for (int i = start_index; i < target_reach_time; i++) {
      original_cost +=
          getMapData(original_plan[i].first - 1, original_plan[i].second - 1);
      num_points_iterated++;
    }

    // Calculate cost of new plan
    double my_cost = 0;
    for (int i = 0; i < target_reach_time - start_index; i++) {
      int j = (i >= commands.size()) ? int(commands.size()) - 1 : i;
      my_cost += getMapData(commands[j].first - 1, commands[j].second - 1);
    }

    return my_cost < original_cost;
  }

  int time_taken_for_planning;

  RealTimePlanner(int *map, int collision_thresh, int x_size, int y_size,
                  Point start, Point goal, int max_plan_length) {
    auto start_time = std::chrono::high_resolution_clock::now();
    map_ = map;
    collision_thresh_ = collision_thresh;
    x_size_ = x_size;
    y_size_ = y_size;
    max_plan_length_ = max_plan_length;
    time_taken_for_planning = 0;
    robot_pose_ = {start.first - 1, start.second - 1};
    goal_ = {goal.first - 1, goal.second - 1};
  }

  RealTimePlanner() {}

  bool expandStates(double transition_cost, int time_for_planning) {
    auto start_time = std::chrono::high_resolution_clock::now();

    while (!open_list_.empty()) {
      auto current_time = std::chrono::high_resolution_clock::now();
      auto time_elapsed =
          std::chrono::duration_cast<milliseconds>(current_time - start_time);
      if (time_elapsed.count() > time_for_planning) {
        return false;
      }

      NodePtr curr_node = open_list_.top();
      open_list_.pop();
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
            open_list_.push(neighbor);
            neighbor->is_on_open_list = true;
          }
        }
      }
      if (goal_found) {
        break;
      }
    }
    return true;
  }

  void constructPathFromPlan() {
    NodePtr curr_node = node_grid_[goal_.first][goal_.second][0];
    if (curr_node == nullptr)
      return;

    while (curr_node->parent != nullptr) {
      commands.push_back({curr_node->x + 1, curr_node->y + 1});
      curr_node = curr_node->parent;
    }

    std::reverse(commands.begin(), commands.end());
  }

  std::vector<Point> commands;

  ~RealTimePlanner() {
    for (int i = 0; i < node_grid_.size(); i++) {
      for (int j = 0; j < node_grid_[0].size(); j++) {
        if (node_grid_[i][j][0] != nullptr) {
          node_grid_[i][j][0]->parent = nullptr;
        }
        node_grid_[i][j].clear();
      }
      node_grid_[i].clear();
    }
  }

private:
  NodeGrid node_grid_;
  OpenList open_list_;

  int *map_;
  int x_size_;
  int y_size_;
  int collision_thresh_;

  int max_plan_length_;

  int target_steps_;
  int *target_traj_;

  Point robot_pose_;

  Point goal_;

  double getMapData(int x, int y) {
    if (x >= x_size_ || x < 0 || y >= y_size_ || y < 0)
      return __DBL_MAX__;

    return map_[(y * x_size_ + x)];
  }

  // Deprecated
  inline Diff diffFromAToB(NodePtr A, NodePtr B) {
    return {B->x - A->x, B->y - A->y};
  }
};

#endif // REAL_TIME_PLANNER_H