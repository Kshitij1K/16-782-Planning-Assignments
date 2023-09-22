#ifndef REAL_TIME_PLANNER_H
#define REAL_TIME_PLANNER_H

#include <iostream>
#include <math.h>
#include <memory>
#include <queue>
#include <time.h>
#include <vector>

struct Node;

typedef std::shared_ptr<Node> NodePtr;
typedef std::vector<std::vector<std::vector<NodePtr>>> NodeGrid;
typedef std::pair<int, int> Point;

struct Node {
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
};

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    // Return true if A > B
    return A->heuristic_cost_to_go_h > B->heuristic_cost_to_go_h;
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

    robot_pose_ = {robotposeX, robotposeY};
    goal_ = {(int)target_traj[target_steps - 1],
             (int)target_traj[target_steps - 1 + target_steps]};

    std::vector<NodePtr> time(target_steps_, nullptr);
    std::vector<std::vector<NodePtr>> time_and_y(y_size_, time);
    node_grid_ = NodeGrid(x_size_, time_and_y);

    NodePtr start = std::make_shared<Node>(Point(robotposeX, robotposeY), goal_, 0);
    start->cost_to_come_g = 0;
    start->cost_to_go_f = start->heuristic_cost_to_go_h;

      open_list_.push(start);
  }

  void expandStates() {}

  void updateHValues() {}

  void decideNextMove() {}

private:
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_list_;
  std::vector<NodePtr> closed_list_;
  NodeGrid node_grid_;

  double *map_;
  int x_size_;
  int y_size_;
  int collision_thresh_;

  int target_steps_;
  double *target_traj_;

  Point robot_pose_;

  Point goal_;
};

#endif // REAL_TIME_PLANNER_H