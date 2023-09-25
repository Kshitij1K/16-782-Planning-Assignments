/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "planner_v5.h"
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

// void planner(int *map, int collision_thresh, int x_size, int y_size,
//              int robotposeX, int robotposeY, int target_steps, int
//              *target_traj, int targetposeX, int targetposeY, int curr_time,
//              int *action_ptr) {

//   static RealTimePlanner rtplanner(map, collision_thresh, x_size, y_size,
//                                    target_steps, target_traj, robotposeX,
//                                    robotposeY);

//   static int curr_index = rtplanner.commands.size() - 1;

//   if (curr_index >= 0) {
//     action_ptr[0] = rtplanner.commands[curr_index].first;
//     action_ptr[1] = rtplanner.commands[curr_index].second;
//   } else {
//     action_ptr[0] = robotposeX;
//     action_ptr[1] = robotposeY;
//   }

//   curr_index--;
//   return;
// }

// void planner(int *map, int collision_thresh, int x_size, int y_size,
//              int robotposeX, int robotposeY, int target_steps, int
//              *target_traj, int targetposeX, int targetposeY, int curr_time,
//              int *action_ptr) {

//   auto start_time = std::chrono::high_resolution_clock::now();
//   static double percent = 0;
//   int max_time_allowed = 800;

//   // TODO: Only allocate when sure of robot pose and goal location
//   static std::shared_ptr<RealTimePlanner> rtplanner;

//   static bool first_time = true;
//   static int time_required_for_plan;

//   static double current_plan_cost = 0;
//   static double cost_incurred_till_now = 0;

//   // static int milliseconds_required = -1;
//   // static int max_milliseconds_required = -1;

//   static std::vector<Point> commands_to_give(target_steps,
//                                              {robotposeX, robotposeY});

//   // if (first_time) {
//   //   rtplanner.plan(robotposeX, robotposeY, 0);
//   // }

//   // TODO
//   static Point goal;
//   static Point robot_start_pose;

//   for (; percent < 1.2; percent += 0.25) {
//     std::cout << "Trying with transition cost: " << percent *
//     collision_thresh
//               << "\n";

//     // TODO: decide on correct robot_pose, and goal pose
//     rtplanner = std::make_shared<RealTimePlanner>(
//         map, collision_thresh, x_size, y_size, goal, robot_start_pose.first,
//         robot_start_pose.second);
//     auto current_time = std::chrono::high_resolution_clock::now();

//     int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
//                            current_time - start_time)
//                            .count();
//     bool completed = rtplanner->expandStates(percent * collision_thresh,
//                                              max_time_allowed -
//                                              time_elapsed);

//     if (!completed) {
//       break;
//     }

//     time_required_for_plan = rtplanner->time_required_per_plan;

//     int steps_to_goal = rtplanner->commands.size() - 1;

//     std::cout << "Need to catch the target within "
//               << (target_steps - 1 - curr_time) << "s\n";
//     if (steps_to_goal < (target_steps - 1 - curr_time)) {
//       std::cout << "Phew! can reach the target on time.\n";

//       // TODO: Transfer this to the plan from the start location, if the cost
//       is
//       // better than the current plan cost
//       // If transferred
//       // Compute new robot start location and goal location
//       // reset the planner
//       // reset bool
//       break;
//     } else {
//       std::cout << "Uh oh! needs " << steps_to_goal
//                 << "s to catch the target.\n";
//       // reset the planner
//       // same start and goal location
//     }

//     action_ptr[0] = commands_to_give[curr_time].first;
//     action_ptr[1] = commands_to_give[curr_time].second;
//     cost_incurred_till_now +=
//         map[GETMAPINDEX(action_ptr[0], action_ptr[1], x_size, y_size)];
//   }
// }

void planner(int *map, int collision_thresh, int x_size, int y_size,
             int robotposeX, int robotposeY, int target_steps, int *target_traj,
             int targetposeX, int targetposeY, int curr_time, int *action_ptr) {
  static std::vector<Point> commands_to_give(target_steps,
                                             {robotposeX, robotposeY});

  action_ptr[0] = commands_to_give[curr_time].first;
  action_ptr[1] = commands_to_give[curr_time].second;
}