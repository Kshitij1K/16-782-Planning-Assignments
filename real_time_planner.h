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
  Node(std::tuple<int, int, int> data, std::pair<int, int> goal) {
    x = std::get<0>(data);
    y = std::get<1>(data);
    t = std::get<2>(data);

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
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_list;

  

};

#endif // REAL_TIME_PLANNER_H