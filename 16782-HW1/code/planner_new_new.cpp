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

typedef std::chrono::milliseconds milliseconds;

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

int nextGoalIndex(int target_steps, int *target_traj, Point robot_pose) {}

void planner(int *map, int collision_thresh, int x_size, int y_size,
             int robotposeX, int robotposeY, int target_steps, int *target_traj,
             int targetposeX, int targetposeY, int curr_time, int *action_ptr) {

  auto start_time = std::chrono::high_resolution_clock().now();
  static bool planning_complete = true;

  static std::shared_ptr<RealTimePlanner> planner;

  static std::vector<Point> robot_trajectory(target_steps,
                                             Point(robotposeX, robotposeY));

  static int plan_from_index = 0;
  static int time_for_planning = 0;

  if (planning_complete) {
    // check the original path and the new path
    // whichever is better, replace that in the robot trajectory
    if (planner != nullptr) {
      bool replace_plan =
          planner->isMyPlanBetter(robot_trajectory, plan_from_index);

      if (replace_plan) {
        int j = 0;
        for (int i = plan_from_index; i < target_steps; i++) {

          if (j >= int(planner->commands.size())) {
            j = int(planner->commands.size()) - 1;
          }

          robot_trajectory[i] = planner->commands[j];
          j++;
        }
      }
    }

    // Plan next path
    // Robot location = robot trajectory[curr time + estimate of time that would
    // be taken] Target location = target trajectory[the subsequent goal point]
    int plan_from_index = curr_time + time_for_planning + 1;
    int start_idx = (plan_from_index >= target_steps) ? (target_steps - 1)
                                                      : plan_from_index;

    Point start = robot_trajectory[start_idx];
    int goal_idx = nextGoalIndex(target_steps, target_traj, start);
    Point goal(target_traj[goal_idx], target_traj[goal_idx + target_steps]);

    // Initialize planner
    planner =
        std::make_shared<RealTimePlanner>(map, collision_thresh, x_size, y_size,
                                          start, goal, goal_idx - start_idx);
    planning_complete = false;
  }

  // Keep planning
  // Time remaining for this = 800 - time elapsed
  auto current_time = std::chrono::high_resolution_clock::now();
  auto time_elapsed =
      std::chrono::duration_cast<milliseconds>(current_time - start_time);
  int time_remaining = 800 - time_elapsed.count();
  planning_complete = planner->plan(time_remaining);
  // Get estimate of time that took to plan
  time_for_planning = planner->time_taken_for_planning;

  // Do the action from the robot trajectory
  action_ptr[0] = robot_trajectory[curr_time].first;
  action_ptr[1] = robot_trajectory[curr_time].second;
}