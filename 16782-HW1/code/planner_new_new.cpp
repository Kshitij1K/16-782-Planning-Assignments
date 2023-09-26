/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "planner_v6.h"
#include <math.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// using std::chrono;
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

// class RealTimePlanner {
// public:
//   bool plan(int time_remaining);
// };

int nextGoalIndex(int target_steps, int *target_traj, Point robot_pose,
                  int time_remaining) {
  static int current_goal_idx = target_steps + 1;

  for (int i = current_goal_idx - 2; i >= 0; i -= 2) {
    // Chebyshev distance
    int dx = abs(robot_pose.first - target_traj[i]);
    int dy = abs(robot_pose.second - target_traj[i]);
    int distance = (dx + dy) - (dx < dy ? dx : dy);
    std::cout << "Checking goal index" << i << ", dis is " << distance << " \n";

    if (distance <= (time_remaining)) {
      current_goal_idx = i;
      break;
    }
  }
  std::cout << "settled on goal idx" << current_goal_idx << "\n";
  return current_goal_idx;
}

void planner(int *map, int collision_thresh, int x_size, int y_size,
             int robotposeX, int robotposeY, int target_steps, int *target_traj,
             int targetposeX, int targetposeY, int curr_time, int *action_ptr) {

  auto start_time = std::chrono::high_resolution_clock().now();
  static bool planning_complete = true;

  static std::shared_ptr<RealTimePlanner> planner;

  static std::vector<Point> robot_trajectory(target_steps + 1,
                                             Point(robotposeX, robotposeY));

  static int plan_from_index = 0;
  static int time_for_planning = 0;
  static int goal_idx;
  //  =
      // nextGoalIndex(target_steps, target_traj, Point(robotposeX, robotposeY),
                    // target_steps - curr_time - 1);

  static bool does_plan_exist = false;

  static bool planner_prepared = false;

  if (!planner_prepared) {
    // Prepare next planner
    // Robot location = robot trajectory[curr time + estimate of time that would
    // be taken] Target location = target trajectory[the subsequent goal point]
    std::cout << "TIME FOR PLANNING " << time_for_planning << "\n";
    plan_from_index = curr_time + time_for_planning;
    // std::cout << "components of plan from index " << curr_time << ", "
    // << time_for_planning << "\n";
    // std::cout << "Value of plan from index" << plan_from_index;
    int start_idx = (plan_from_index >= target_steps) ? (target_steps - 1)
                                                      : plan_from_index;

    Point start = robot_trajectory[start_idx];
    goal_idx = nextGoalIndex(target_steps, target_traj, start,
                             target_steps - plan_from_index - 1);
    Point goal(target_traj[goal_idx], target_traj[goal_idx + target_steps]);

    // std::cout << "Initializing new planner\n";
    // Initialize planner
    planner =
        std::make_shared<RealTimePlanner>(map, collision_thresh, x_size, y_size,
                                          start, goal, goal_idx - start_idx);
    // std::cout << "Initialized new planner\n";
    // planning_complete = false;
    planner_prepared = true;
  }

  // Run the planner
  // Time remaining for this = 800 - time elapsed
  auto current_time = std::chrono::high_resolution_clock::now();
  auto time_elapsed =
      std::chrono::duration_cast<milliseconds>(current_time - start_time);
  int time_remaining = 800 - time_elapsed.count();
  // std::cout << "Starting new plan\n";
  planning_complete = planner->plan(time_remaining);
  // std::cout << "Made new plan\n";

  // Get estimate of time that took to plan
  time_for_planning = (planner->time_taken_for_planning)/1000;

  if (planning_complete) {
    // check the original path and the new path
    // whichever is better, replace that in the robot trajectory
    // if (planner != nullptr) {
    // std::cout << "Checking if plan needs to be replaced\n";
    bool replace_plan;

    if (!does_plan_exist) {
      replace_plan = planner->commands.size();
    } else {
      replace_plan =
          planner->isMyPlanBetter(robot_trajectory, plan_from_index + 1, goal_idx);
    }

    // bool replace_plan = true;
    if (replace_plan) {
      // std::cout << "yes, Plan needs to be Replaced\n";
      int j = 0;

      // std::cout << "Inserting plan from index " << plan_from_index + 1 <<
      // "\n";
      for (int i = plan_from_index + 1; i <= target_steps; i++) {

        if (j >= int(planner->commands.size())) {
          j = int(planner->commands.size()) - 1;
        }

        // std::cout << "Robot trajectory size and i" << robot_trajectory.size()
        // << ", " << i << "\n";
        // std::cout << "Command size and j" << planner->commands.size() << ", "
        // << j << "\n";

        robot_trajectory[i] = planner->commands[j];
        j++;
      }
      does_plan_exist = true;
    }

    // for (int i = 0; i < planner->commands.size(); i++) {
      // std::cout << planner->commands[i].first << ", @"
                // << planner->commands[i].second << "\n";
    // }
    // for (int i = 0; i < robot_trajectory.size(); i++) {
      // std::cout << robot_trajectory[i].first << "@ " <<
      // robot_trajectory[i].second
                // << "\n";
    // }
    // Next planner needs to be prepared
    planner_prepared = false;
  }

  // Do the action from the robot trajectory
  action_ptr[0] = robot_trajectory[curr_time + 1].first;
  action_ptr[1] = robot_trajectory[curr_time + 1].second;
  std::cout << "ACTION TAKEN: " << action_ptr[0] << ", " << action_ptr[1]
            << "\n";
  std::cout << "CURR ROBOT: " << robotposeX << ", " << robotposeY << "\n";
}