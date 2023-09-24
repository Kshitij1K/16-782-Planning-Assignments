/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include "real_time_planner.h"
#include <math.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

void planner_greedy(double *map, int collision_thresh, int x_size, int y_size,
                    int robotposeX, int robotposeY, int target_steps,
                    double *target_traj, int targetposeX, int targetposeY,
                    int curr_time, double *action_ptr) {
  // 8-connected grid
  int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};

  // for now greedily move towards the final target position,
  // but this is where you can put your planner

  int goalposeX = (int)target_traj[target_steps - 1];
  int goalposeY = (int)target_traj[target_steps - 1 + target_steps];
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
      if (((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] >= 0) &&
          ((int)map[GETMAPINDEX(newx, newy, x_size, y_size)] <
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

void planner(double *map, int collision_thresh, int x_size, int y_size,
             int robotposeX, int robotposeY, int target_steps,
             double *target_traj, int targetposeX, int targetposeY,
             int curr_time, double *action_ptr) {

  // Point goal((int)target_traj[target_steps - 1],
  //            (int)target_traj[target_steps - 1 + target_steps]);

  // std::cout << "-----------------MAP INFO--------------------"
  //           << "\n";
  // std::cout << "Map size: " << x_size << ", " << y_size << "\n";
  // std::cout << "Total time" << target_steps << "\n";

  static RealTimePlanner rtplanner(map, collision_thresh, x_size, y_size,
                                   target_steps, target_traj, robotposeX,
                                   robotposeY);

  // rtplanner.expandStates(std::chrono::milliseconds(1));
  // rtplanner.constructPathFromPlan();

  // std::cout << "Current Robot Pose: " << robotposeX << ", " << robotposeY << "\n";
  // std::cout << "Current Obj Pose: " << targetposeX << ", " << targetposeY << "\n";

  static int curr_index = rtplanner.commands.size() - 1;
  
  std::cout << "Number of commands left: " << curr_index << "\n";

  if (curr_index >= 0) {
    action_ptr[0] = rtplanner.commands[curr_index].first;
    action_ptr[1] = rtplanner.commands[curr_index].second;
  } else {
    action_ptr[0] = rtplanner.commands[0].first;
    action_ptr[1] = rtplanner.commands[0].second;
  }

  // std::cout << ""

  curr_index--;
  return;
}