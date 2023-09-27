/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "planner_v4.h"
#include <math.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

void planner_greedy(int *map, int collision_thresh, int x_size, int y_size,
                    int robotposeX, int robotposeY, int target_steps,
                    int *target_traj, int targetposeX, int targetposeY,
                    int curr_time, int *action_ptr) {
  // 8-connected grid
  int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

  // for now greedily move towards the final target position,
  // but this is where you can put your planner

  int goalposeX = target_traj[target_steps - 1];
  int goalposeY = target_traj[target_steps - 1 + target_steps];
  // printf("robot: %d %d;\n", robotposeX, robotposeY);
  // printf("goal: %d %d;\n", goalposeX, goalposeY);

  int bestX = 0,
      bestY = 0; // robot will not move if greedy action leads to collision
  double olddisttotarget =
      (double)sqrt(((robotposeX - goalposeX) * (robotposeX - goalposeX) +
                    (robotposeY - goalposeY) * (robotposeY - goalposeY)));
  double disttotarget;
  for (int dir = 0; dir < NUMOFDIRS; dir++) {
    int newx = robotposeX + dX[dir];
    int newy = robotposeY + dY[dir];

    if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
      if ((map[GETMAPINDEX(newx, newy, x_size, y_size)] >= 0) &&
          (map[GETMAPINDEX(newx, newy, x_size, y_size)] <
           collision_thresh)) // if free
      {
        disttotarget = (double)sqrt(((newx - goalposeX) * (newx - goalposeX) +
                                     (newy - goalposeY) * (newy - goalposeY)));
        if (disttotarget < olddisttotarget) {
          olddisttotarget = disttotarget;
          bestX = dX[dir];
          bestY = dY[dir];
        }
      }
    }
  }
  robotposeX = robotposeX + bestX;
  robotposeY = robotposeY + bestY;
  action_ptr[0] = robotposeX;
  action_ptr[1] = robotposeY;

  return;
}

// This is incorrect
int nextGoalIndex(int target_steps, int *target_traj, Point robot_pose,
                  int current_time) {
  static int current_goal_idx = target_steps;
  
  for (int i = current_goal_idx - 1; i >= 0; i -= 2) {
    // Chebyshev distance
    int dx = abs(robot_pose.first - target_traj[i]);
    int dy = abs(robot_pose.second - target_traj[i]);
    int distance = (dx + dy) - (dx < dy ? dx : dy);

    std::cout << "distance " << distance << ", " << (i - current_time)
              << ", i = " << i << "\n";
    if (distance <= (i - current_time)) {
      current_goal_idx = i;
      break;
    }
  }

  std::cout << "Might try some other goal locations\n";

  if (current_goal_idx == (target_steps + 1)) {
    for (int i = current_goal_idx - 2; i >= 0; i -= 2) {
      // Chebyshev distance

      if (i == target_steps) {
        continue;
      }

      int dx = abs(robot_pose.first - target_traj[i]);
      int dy = abs(robot_pose.second - target_traj[i]);
      int distance = (dx + dy) - (dx < dy ? dx : dy);

      std::cout << "distance " << distance << ", " << (i - current_time)
                << ", i = " << i << "\n";
      if (distance <= (i - current_time)) {
        current_goal_idx = i;
        break;
      }
    }
  }

  return current_goal_idx;
}

bool isAdmissibleMove(Point start, Point goal) {

  // std::cout << "?????????????\n";
  // std::cout << "Checking admissibility, \n";
  // std::cout << start.first << ", " << start.second << "\n";
  // std::cout << goal.first << ", " << goal.second << "\n";

  bool x_move_possible = abs(start.first - goal.first) <= 1;
  bool y_move_possible = abs(start.second - goal.second) <= 1;

  // std::cout << "Result : " << (x_move_possible && y_move_possible) << "\n";
  // std::cout << "?????????????\n";
  return (x_move_possible && y_move_possible);
}

void planner(int *map, int collision_thresh, int x_size, int y_size,
             int robotposeX, int robotposeY, int target_steps, int *target_traj,
             int targetposeX, int targetposeY, int curr_time, int *action_ptr) {

  // if (curr_time >= 10) {
  //   action_ptr[0] = robotposeX;
  //   action_ptr[1] = robotposeY;
  //   std::cout << "Current time: " << curr_time << "\n";
  //   std::cout << "Robot is going to move to - " << action_ptr[0] << ", "
  //             << action_ptr[1] << " in the next step\n";
  //   std::cout
  //       <<
  //       "---------------------------------------------------------------\n";
  // }

  auto start_time = std::chrono::high_resolution_clock().now();
  static bool planning_complete = true;

  static RealTimePlanner planner;

  static std::vector<Point> robot_trajectory(target_steps + 1,
                                             Point(robotposeX, robotposeY));

  static int plan_from_index = 0;
  static int time_estimate_for_planning = 0;
  static int goal_idx;
  static int start_idx;

  static bool does_plan_exist = false;

  static bool planner_prepared = false;

  if (!planner_prepared) {
    // Prepare next planner
    // std::cout << "Time required previously\n";
    plan_from_index = curr_time + time_estimate_for_planning;

    start_idx = (plan_from_index >= target_steps) ? (target_steps - 1)
                                                  : plan_from_index;

    Point start = robot_trajectory[start_idx];
    goal_idx = nextGoalIndex(target_steps, target_traj, start, plan_from_index);
    Point goal(target_traj[goal_idx], target_traj[goal_idx + target_steps]);

    // Initialize planner
    std::cout << "Next Planner prepared with start and goal time to be \n";
    std::cout << start_idx << "," << goal_idx << "\n";
    std::cout << "Time estimate for planning is " << time_estimate_for_planning
              << "\n";
    // << goal.second << "\n";
    planner = RealTimePlanner(map, collision_thresh, x_size, y_size, start,
                              goal, goal_idx - start_idx);
    planner_prepared = true;
    time_estimate_for_planning = 1;
  }

  // Run the planner
  std::cout << "Running the prepared planner\n";
  auto current_time = std::chrono::high_resolution_clock::now();
  auto time_elapsed =
      std::chrono::duration_cast<milliseconds>(current_time - start_time);
  int time_remaining = 750 - time_elapsed.count();
  planning_complete = planner.plan(time_remaining);
  time_estimate_for_planning += 1;

  // Get estimate of time that took to plan
  // time_for_planning = (planner.time_taken_for_planning) / 1000;

  if (planning_complete) {
    // check the original path and the new path
    // whichever is better, replace that in the robot trajectory
    // std::cout << "Planning complete\n";
    bool replace_plan;

    // Point next_move = {-3, -3};
    // if (planner.commands.size() != 0) {
    //   next_move = planner.commands[0];
    // }
    // bool is_next_move_possible =
    //     isAdmissibleMove({robotposeX, robotposeY}, next_move);
    // bool can_robot_reach_target =
    //     ((goal_idx - plan_from_index) >= planner.commands.size());
    // bool is_plan_better =
    //     planner.isMyPlanBetter(robot_trajectory, plan_from_index + 1,
    //     goal_idx);

    if (!does_plan_exist) {
      replace_plan = planner.commands.size();
      start_idx = curr_time;
    } else {
      // replace_plan =
      // bool is_next_move_possible
      // isAdmissibleMove({robotposeX, robotposeY}, planner.commands[0]);

      bool planning_done_in_time = (curr_time <= start_idx);

      std::cout << "Curr time " << curr_time
                << "; replacement is going to be done " << start_idx << "\n";

      // if (!is_next_move_possible) {
      // std::cout << "Why are moves not possible!!!!\n";
      // std::cout << "Current robot time move time is " << curr_time << "\n";
      // }

      bool can_robot_reach_target =
          ((goal_idx - plan_from_index) > planner.commands.size());
      bool is_plan_better =
          planner.isMyPlanBetter(robot_trajectory, curr_time + 1, goal_idx);

      replace_plan =
          planning_done_in_time && can_robot_reach_target && is_plan_better;
      std::cout << "Next move possible?: " << planning_done_in_time << "\n";
      std::cout << "Can robot reach?: " << can_robot_reach_target << "\n";
      std::cout << "is plan better?: " << is_plan_better << "\n";
    }

    // Replace plan if (plan doesn't exist and a new plan is found) or a plan
    // exists and all conditions are met

    // if (!does_plan_exist) {
    //   replace_plan = planner.commands.size();
    // } else {
    //   replace_plan = planner.isMyPlanBetter(robot_trajectory,
    //                                         plan_from_index + 1, goal_idx);
    // }

    // // std::cout << "Current time is " << curr_time;
    // // std::cout << ". Plan replacement begins from " << plan_from_index +
    // 1
    //           // << "\n";
    // if (does_plan_exist && ((plan_from_index + 1) <= curr_time)) {
    //   std::cout << "Plan is too old. Not replacing.\n";
    //   replace_plan = false;
    // } else {
    //   std::cout << "No plan exists, or Plan is not too old. Can replace may
    //   be\n";
    // }
    if (replace_plan) {
      int j = 0;

      for (int i = start_idx + 1; i <= target_steps; i++) {

        if (j >= int(planner.commands.size())) {
          j = int(planner.commands.size()) - 1;
        }

        robot_trajectory[i] = planner.commands[j];
        j++;
      }
      does_plan_exist = true;
      std::cout << "Plan replaced\n";
    }
    planner_prepared = false;
  }

  // Do the action from the robot trajectory

  bool final_check = isAdmissibleMove(Point(robotposeX, robotposeY),
                                      robot_trajectory[curr_time + 1]);

  if (!final_check) {
    std::cout << "Code attempting to make illegal move\n";
    std::cout << "Robot trajectory replaced from " << start_idx << "\n";
    std::cout << "Current robot time  " << curr_time << "\n";
    std::cout << "Robot trajectory right before that "
              << robot_trajectory[curr_time].first << ", "
              << robot_trajectory[curr_time].second << "\n";
  }

  action_ptr[0] = robot_trajectory[curr_time + 1].first;
  action_ptr[1] = robot_trajectory[curr_time + 1].second;
  std::cout << "Current time: " << curr_time << "\n";
  std::cout << "Robot is going to move to - " << action_ptr[0] << ", "
            << action_ptr[1] << " in the next step\n";
  std::cout
      << "---------------------------------------------------------------\n";
}