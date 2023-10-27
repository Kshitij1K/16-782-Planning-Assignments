// clang-format off
/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <list>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Custom Includes */
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <queue>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
        }
    }    
    *planlength = numofsamples;
    
    return;
}

// clang-format on
typedef std::vector<double> ContinuousArmCfg;
typedef std::vector<int> DiscreteArmCfg;

template <typename T>
void makeVector(std::vector<T> &result, T *data, int size) {
  result.reserve(size);

  for (int i = 0; i < size; i++) {
    result.push_back(data[i]);
  }
}

// Parameters
double resolution = 0.02; // radians per index
int max_iters = 10000;
int epsilon = 5;
long int seed = 34234;
long int max_neighbor_distance = 10;
long int max_prm_edge_distance = 50;
long int num_prm_points = 25000;

DiscreteArmCfg discretizeArmConfig(const ContinuousArmCfg &input) {
  DiscreteArmCfg result;
  result.reserve(input.size());
  for (int i = 0; i < input.size(); i++) {
    result.push_back(std::floor(input[i] / resolution));
  }

  return result;
}

ContinuousArmCfg revertDiscretizationArmConfig(const DiscreteArmCfg &input) {
  ContinuousArmCfg result;
  result.reserve(input.size());
  for (int i = 0; i < input.size(); i++) {
    result.push_back(input[i] * resolution);
  }
  return result;
}

bool IsValidArmConfiguration(const DiscreteArmCfg &q, double *map, int x_size,
                             int y_size) {
  ContinuousArmCfg q_cont = revertDiscretizationArmConfig(q);
  // Seriously guys, its time to move on from malloc
  double *q_dangerous = (double *)malloc(q_cont.size() * sizeof(double));
  for (int i = 0; i < q_cont.size(); i++) {
    q_dangerous[i] = q_cont[i];
  }

  bool result =
      IsValidArmConfiguration(q_dangerous, q_cont.size(), map, x_size, y_size);
  free(q_dangerous);
  return result;
}

static bool validLine(double *map, int x_size, int y_size, DiscreteArmCfg start,
                      DiscreteArmCfg end, int numofDOFs) {
  double distance = 0;
  int i, j;
  for (j = 0; j < numofDOFs; j++) {
    if (distance < fabs(start[j] - end[j]))
      distance = fabs(start[j] - end[j]);
  }
  int numofsamples = (int)(distance / (PI / 20));
  if (numofsamples < 2) {
    printf("the arm is already at the goal\n");
    return true;
  }
  for (i = 0; i < numofsamples; i++) {
    DiscreteArmCfg intermediate;

    for (j = 0; j < numofDOFs; j++) {
      intermediate.push_back(start[j] + ((double)(i) / (numofsamples - 1)) *
                                            (end[j] - start[j]));
    }

    if (!IsValidArmConfiguration(intermediate, map, x_size, y_size)) {
      return false;
    }
  }
  return true;
}
//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION //
//                                                                                                                   //
//*******************************************************************************************************************//

struct NodeData;

typedef std::shared_ptr<NodeData> NodeDataPtr;

struct NodeData {
  DiscreteArmCfg parent_;
  long int f_val;
  long int cost;
};

class NodeHash {
public:
  std::size_t operator()(const DiscreteArmCfg &arm_cfg) const noexcept {
    std::string string_of_angles;

    for (int i = 0; i < arm_cfg.size(); i++) {
      string_of_angles.append(std::to_string(arm_cfg[i]));
    }

    return std::hash<std::string>{}(string_of_angles);
  }
};

void printCfg(const DiscreteArmCfg &q) {
  if (q.size() == 0) {
    std::cout << "q is empty!\n";
    return;
  }

  for (int i = 0; i < int(q.size()) - 1; i++) {
    std::cout << q[i] << ", ";
  }

  std::cout << q.back();
}

void printCfg(const ContinuousArmCfg &q) {
  if (q.size() == 0) {
    std::cout << "q is empty!\n";
    return;
  }

  for (int i = 0; i < int(q.size()) - 1; i++) {
    std::cout << q[i] << ", ";
  }

  std::cout << q.back();
}

static DiscreteArmCfg randomCfgGenerator(int num_DOFs) {
  static std::default_random_engine generator;
  static bool is_seeded = false;
  if (!is_seeded) {
    generator.seed(seed);
    is_seeded = true;
  }
  long int max = std::floor(2 * PI / resolution);
  static std::uniform_int_distribution<> random_int_generator(0, max);

  DiscreteArmCfg q;

  for (int i = 0; i < num_DOFs; i++) {
    q.push_back(random_int_generator(generator));
  }

  return q;
}

enum class ExtendResult { ADVANCED, REACHED, TRAPPED };

ExtendResult extendNewCfg(const DiscreteArmCfg &q_near,
                          const DiscreteArmCfg &q_rand, DiscreteArmCfg &q_new,
                          double *map, int x_size, int y_size, int numDOFs) {

  q_new = DiscreteArmCfg();

  int max_angle_change = abs(q_near[0] - q_rand[0]);

  for (int i = 0; i < q_near.size(); i++) {
    if (max_angle_change < abs(q_near[i] - q_rand[i])) {
      max_angle_change = abs(q_near[i] - q_rand[i]);
    }
  }

  double percent_interpolate = (epsilon * 1.0) / max_angle_change;

  if (max_angle_change == 0 || percent_interpolate > 1) {
    q_new = q_rand;
    return ExtendResult::REACHED;
  }

  for (int i = 0; i < q_near.size(); i++) {
    int diff = q_rand[i] - q_near[i];
    int new_angle = std::floor(q_near[i] + percent_interpolate * diff);
    q_new.push_back(new_angle);
  }

  if (validLine(map, x_size, y_size, q_near, q_new, numDOFs)) {
    return ExtendResult::ADVANCED;
  } else {
    return ExtendResult::TRAPPED;
  }
}

long int distanceBetweenCfgs(const DiscreteArmCfg &q1,
                             const DiscreteArmCfg &q2) {
  long int distance = abs(q1[0] - q2[0]);

  for (int i = 1; i < q1.size(); i++) {
    if (abs(q1[i] - q2[i]) > distance) {
      distance = abs(q1[i] - q2[i]);
    }
  }

  return distance;
}
void generateFinalPlan(const std::list<ContinuousArmCfg> &reversed_plan,
                       double ***plan_to_fill) {
  if (reversed_plan.size() == 0) {
    *plan_to_fill = NULL;
    return;
  }

  int numDOFs = reversed_plan.front().size();
  *plan_to_fill = (double **)malloc((reversed_plan.size()) * sizeof(double *));

  int i = 0;
  std::cout << "Generating and printing final plan:\n";
  for (auto it_ = reversed_plan.rbegin(); (it_ != reversed_plan.rend());
       it_++) {
    (*plan_to_fill)[i] = (double *)malloc(numDOFs * sizeof(double));
    auto q_cont = *it_;
    printCfg(q_cont);
    std::cout << "\n";
    for (int j = 0; j < numDOFs; j++) {
      (*plan_to_fill)[i][j] = q_cont[j];
    }
    i++;
  }
}

class RRTree {
public:
  RRTree(ContinuousArmCfg start) {
    start_ = start;
    auto start_discrete = discretizeArmConfig(start_);
    addNode(start_discrete, 0);
  }

  RRTree(DiscreteArmCfg start) {
    addNode(start, 0);
  }

  DiscreteArmCfg nearestNeighbor(const DiscreteArmCfg &q) {
    long int distance = __INT64_MAX__;
    DiscreteArmCfg nearest_neighbor;
    for (auto it_ : node_map_) {
      long int calculated_distance = distanceBetweenCfgs(it_.first, q);
      if (calculated_distance < distance) {
        nearest_neighbor = it_.first;
        distance = calculated_distance;
      }
    }

    return nearest_neighbor;
  }

  std::list<DiscreteArmCfg> nearestNeighbors(const DiscreteArmCfg &q) {
    std::list<DiscreteArmCfg> nearest_neighbors;
    for (auto it_ : node_map_) {
      long int calculated_distance = distanceBetweenCfgs(it_.first, q);
      if (calculated_distance < max_neighbor_distance) {
        nearest_neighbors.push_back(it_.first);
      }
    }

    return nearest_neighbors;
  }

  void addNode(DiscreteArmCfg &q, long int cost) {
    auto data = std::make_shared<NodeData>();
    data->cost = cost;
    node_map_.insert({q, data});
  }

  void setParent(DiscreteArmCfg &q, DiscreteArmCfg &parent) {
    if (node_map_.find(q) == node_map_.end()) {
      std::cout << "Node q: ";
      printCfg(q);
      std::cout << " not found! Setting Parent failed\n";
      return;
    }

    auto node = node_map_.at(q);
    node->parent_ = parent;
  }

  std::list<ContinuousArmCfg> generateReversedPlan(DiscreteArmCfg &goal) {
    std::list<ContinuousArmCfg> reversed_plan;
    reversed_plan.push_back(revertDiscretizationArmConfig(goal));
    auto current = node_map_.at(goal)->parent_;

    while (current.size() != 0) {
      reversed_plan.push_back(revertDiscretizationArmConfig(current));
      current = node_map_.at(current)->parent_;
    }

    if (!start_.empty()) {
      reversed_plan.push_back(start_);
    }

    return reversed_plan;
  }

  std::list<ContinuousArmCfg> generateReversedPlan(ContinuousArmCfg &goal) {
    std::list<ContinuousArmCfg> reversed_plan;
    reversed_plan.push_back(goal);
    auto current = node_map_.at(discretizeArmConfig(goal))->parent_;

    while (node_map_.at(current)->parent_.size() != 0) {
      reversed_plan.push_back(revertDiscretizationArmConfig(current));
      current = node_map_.at(current)->parent_;
    }

    if (!start_.empty()) {
      reversed_plan.push_back(start_);
    }

    return reversed_plan;
  }

  bool doesNodeExist(const DiscreteArmCfg &q) {
    return (node_map_.find(q) != node_map_.end());
  }

  inline long int getNodeCost(const DiscreteArmCfg &q) {
    return node_map_.at(q)->cost;
  }

private:
  std::unordered_map<DiscreteArmCfg, NodeDataPtr, NodeHash> node_map_;
  ContinuousArmCfg start_;
};

ExtendResult extensionResult(double *map, int x_size, int y_size, int numofDOFs,
                             DiscreteArmCfg q_near, DiscreteArmCfg q_new,
                             DiscreteArmCfg q_fin) {
  if (validLine(map, x_size, y_size, q_near, q_new, numofDOFs)) {
    if (q_near == q_new) {
      return ExtendResult::REACHED;
    } else {
      return ExtendResult::ADVANCED;
    }
  } else {
    return ExtendResult::TRAPPED;
  }
}

static void plannerRRT(double *map, int x_size, int y_size,
                       double *armstart_anglesV_rad,
                       double *armgoal_anglesV_rad, int numofDOFs,
                       double ***plan, int *planlength) {
  /* TODO: Replace with your implementation */
  ContinuousArmCfg arm_start_cont;
  makeVector(arm_start_cont, armstart_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_start = discretizeArmConfig(arm_start_cont);

  ContinuousArmCfg arm_goal_cont;
  makeVector(arm_goal_cont, armgoal_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_goal = discretizeArmConfig(arm_goal_cont);

  RRTree rrtree(arm_start_cont);

  for (int iters = 0; iters < max_iters; iters++) {

    if ((iters % 100) == 0) {
      std::cout << "Iteration: " << iters << "\n";
      std::cout
          << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
             "~~~~~~~~~~~~~~~~~~\n";
    }
    bool is_q_valid = false;
    DiscreteArmCfg q_rand;
    while (!is_q_valid) {
      q_rand = randomCfgGenerator(numofDOFs);
      is_q_valid = IsValidArmConfiguration(q_rand, map, x_size, y_size);
    }

    DiscreteArmCfg q_near = rrtree.nearestNeighbor(q_rand);
    DiscreteArmCfg q_new;
    auto result =
        extendNewCfg(q_near, q_rand, q_new, map, x_size, y_size, numofDOFs);

    if ((rrtree.doesNodeExist(q_new)) || result == ExtendResult::TRAPPED) {
      continue;
    }

    rrtree.addNode(q_new, 0);
    rrtree.setParent(q_new, q_near);

    if (validLine(map, x_size, y_size, q_new, arm_goal, numofDOFs)) {
      std::cout << "Reached goal\n";
      rrtree.addNode(arm_goal, 0);
      rrtree.setParent(arm_goal, q_new);
      auto reversed_plan = rrtree.generateReversedPlan(arm_goal_cont);
      generateFinalPlan(reversed_plan, plan);
      *planlength = reversed_plan.size();
      return;
    }
  }

  std::list<ContinuousArmCfg> default_plan;
  default_plan.push_back(arm_goal_cont);
  default_plan.push_back(arm_start_cont);
  generateFinalPlan(default_plan, plan);
  *planlength = 2;
  std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
               "~~~~~~~~\n";
  std::cout << "PLAN NOT FOUND! RETURNING DEFAULT\n";
  std::cout << "\n\n\n\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
               "~~~~~~~~~~~~~~~~~~\n";
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(double *map, int x_size, int y_size,
                              double *armstart_anglesV_rad,
                              double *armgoal_anglesV_rad, int numofDOFs,
                              double ***plan, int *planlength) {
  /* TODO: Replace with your implementation */
  ContinuousArmCfg arm_start_cont;
  makeVector(arm_start_cont, armstart_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_start = discretizeArmConfig(arm_start_cont);

  ContinuousArmCfg arm_goal_cont;
  makeVector(arm_goal_cont, armgoal_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_goal = discretizeArmConfig(arm_goal_cont);

  auto rrtree_A = std::make_shared<RRTree>(arm_start_cont);
  auto rrtree_B = std::make_shared<RRTree>(arm_goal_cont);

  long int num_swaps = 0;

  for (int iters = 0; iters < max_iters; iters++) {

    if ((iters % 100) == 0) {
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                   "~~~~~~~~~~~~~~~~~~~~\n";
      std::cout << "Iteration: " << iters << "\n";
    }
    bool is_q_valid = false;
    DiscreteArmCfg q_rand;
    while (!is_q_valid) {
      q_rand = randomCfgGenerator(numofDOFs);
      is_q_valid = IsValidArmConfiguration(q_rand, map, x_size, y_size);
    }

    DiscreteArmCfg q_near_a = rrtree_A->nearestNeighbor(q_rand);
    DiscreteArmCfg q_new_a;
    auto result_a =
        extendNewCfg(q_near_a, q_rand, q_new_a, map, x_size, y_size, numofDOFs);

    if ((rrtree_A->doesNodeExist(q_new_a)) ||
        result_a == ExtendResult::TRAPPED) {
      continue;
    }

    rrtree_A->addNode(q_new_a, 0);
    rrtree_A->setParent(q_new_a, q_near_a);

    DiscreteArmCfg q_near_b = rrtree_B->nearestNeighbor(q_new_a);
    DiscreteArmCfg q_new_final_b = q_near_b;

    DiscreteArmCfg q_new_b;
    auto result_b = extendNewCfg(q_new_final_b, q_new_a, q_new_b, map, x_size,
                                 y_size, numofDOFs);

    long long int d = 0;
    while (result_b == ExtendResult::ADVANCED) {
      q_new_final_b = q_new_b;
      result_b = extendNewCfg(q_new_final_b, q_new_a, q_new_b, map, x_size,
                              y_size, numofDOFs);
    }

    if (q_new_final_b != q_near_b) {
      rrtree_B->addNode(q_new_final_b, 0);
      rrtree_B->setParent(q_new_final_b, q_near_b);
    }

    if (result_b == ExtendResult::REACHED) {
      std::cout << "Plan found!\n";
      auto reversed_plan_a = rrtree_A->generateReversedPlan(q_new_a);

      auto reversed_plan_b = rrtree_B->generateReversedPlan(q_new_final_b);

      if (num_swaps % 2) {
        reversed_plan_a.reverse();
        reversed_plan_a.insert(reversed_plan_a.end(), reversed_plan_b.begin(),
                               reversed_plan_b.end());
        generateFinalPlan(reversed_plan_a, plan);
        *planlength = reversed_plan_a.size();
        return;
      } else {
        reversed_plan_b.reverse();
        reversed_plan_b.insert(reversed_plan_b.end(), reversed_plan_a.begin(),
                               reversed_plan_a.end());
        generateFinalPlan(reversed_plan_b, plan);
        *planlength = reversed_plan_b.size();
        return;
      }
    }

    rrtree_A.swap(rrtree_B);
    num_swaps++;
  }

  std::list<ContinuousArmCfg> default_plan;
  default_plan.push_back(arm_goal_cont);
  default_plan.push_back(arm_start_cont);
  generateFinalPlan(default_plan, plan);
  *planlength = 2;
  std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
               "~~~~~~~~\n";
  std::cout << "PLAN NOT FOUND! RETURNING DEFAULT\n";
  std::cout << "\n\n\n\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
               "~~~~~~~~~~~~~~~~~~\n";
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION //
//                                                                                                                   //
//*******************************************************************************************************************//

long int getLineCost(DiscreteArmCfg q1, DiscreteArmCfg q2) {
  long int cost = 0;

  for (int i = 0; i < q1.size(); i++) {
    cost += abs(q1[i] - q2[i]);
  }

  return cost;
}

static void plannerRRTStar(double *map, int x_size, int y_size,
                           double *armstart_anglesV_rad,
                           double *armgoal_anglesV_rad, int numofDOFs,
                           double ***plan, int *planlength) {
  /* TODO: Replace with your implementation */
  ContinuousArmCfg arm_start_cont;
  makeVector(arm_start_cont, armstart_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_start = discretizeArmConfig(arm_start_cont);

  ContinuousArmCfg arm_goal_cont;
  makeVector(arm_goal_cont, armgoal_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_goal = discretizeArmConfig(arm_goal_cont);

  std::cout << "All vectors made\n";
  std::cout << "Goal is to reach: ";
  printCfg(arm_goal);

  RRTree rrtree(arm_start_cont);

  long int goal_cost = __INT64_MAX__;
  DiscreteArmCfg goal_parent;

  for (int iters = 0; iters < max_iters; iters++) {

    if ((iters % 100) == 0) {
      std::cout
          << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
             "~~~~~~~~~~~~~~~~~~\n";
      std::cout << "Iteration: " << iters << "\n";
    }

    bool is_q_valid = false;
    DiscreteArmCfg q_rand;
    while (!is_q_valid) {
      q_rand = randomCfgGenerator(numofDOFs);
      is_q_valid = IsValidArmConfiguration(q_rand, map, x_size, y_size);
    }

    DiscreteArmCfg q_near = rrtree.nearestNeighbor(q_rand);
    DiscreteArmCfg q_new;
    auto result =
        extendNewCfg(q_near, q_rand, q_new, map, x_size, y_size, numofDOFs);

    if ((rrtree.doesNodeExist(q_new)) || result == ExtendResult::TRAPPED) {
      continue;
    }

    std::list<DiscreteArmCfg> neighbors = rrtree.nearestNeighbors(q_new);

    int lowest_cost = __INT_MAX__;
    DiscreteArmCfg lowest_cost_neighbor;
    for (auto neighbor : neighbors) {
      long int cost =
          getLineCost(neighbor, q_new) + rrtree.getNodeCost(neighbor);
      if (cost < lowest_cost) {
        lowest_cost = cost;
        lowest_cost_neighbor = neighbor;
      }
    }

    rrtree.addNode(q_new, lowest_cost);
    rrtree.setParent(q_new, lowest_cost_neighbor);

    for (auto neighbor : neighbors) {
      if (neighbor == lowest_cost_neighbor) {
        continue;
      }

      long int previous_cost = rrtree.getNodeCost(neighbor);
      long int new_cost =
          getLineCost(q_new, neighbor) + rrtree.getNodeCost(q_new);
      if (new_cost < previous_cost) {
        rrtree.setParent(neighbor, q_new);
      }
    }

    if (validLine(map, x_size, y_size, q_new, arm_goal, numofDOFs)) {
      long int cost = rrtree.getNodeCost(q_new) + getLineCost(q_new, arm_goal);

      if (cost < goal_cost) {
        goal_cost = cost;
        goal_parent = q_new;
      }
    }
  }

  if (goal_cost < __INT64_MAX__) {
    rrtree.addNode(arm_goal, goal_cost);
    rrtree.setParent(arm_goal, goal_parent);
    auto reversed_plan = rrtree.generateReversedPlan(arm_goal_cont);
    generateFinalPlan(reversed_plan, plan);
    *planlength = reversed_plan.size();
  } else {
    std::list<ContinuousArmCfg> default_plan;
    default_plan.push_back(arm_goal_cont);
    default_plan.push_back(arm_start_cont);
    generateFinalPlan(default_plan, plan);
    *planlength = 2;
    std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                 "~~~~~~~~~~\n";
    std::cout << "PLAN NOT FOUND! RETURNING DEFAULT\n";
    std::cout << "\n\n\n\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                 "~~~~~~~~~~~~~~~~~~~~\n";
  }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION //
//                                                                                                                   //
//*******************************************************************************************************************//
typedef std::shared_ptr<std::list<DiscreteArmCfg>> NeighborList;

class PRMGraph {
public:
  void addNode(DiscreteArmCfg &q) {
    NeighborList neighbors = std::make_shared<std::list<DiscreteArmCfg>>();
    graph.insert({q, neighbors});
  }

  void addEdge(DiscreteArmCfg &q1, DiscreteArmCfg &q2) {
    graph.at(q1)->push_back(q2);
    graph.at(q2)->push_back(q1);
  }

  NeighborList getConnectedNeighbors(DiscreteArmCfg &q) {
    auto list_it = graph.find(q);
    if (list_it != graph.end()) {
      return (*list_it).second;
    } else {
      return nullptr;
    }
  }

  std::list<DiscreteArmCfg> getNearestNeighbors(const DiscreteArmCfg &q) {
    std::list<DiscreteArmCfg> nearest_neighbors;
    for (auto it_ : graph) {

      if (it_.first == q) {
        continue;
      }

      long int calculated_distance = distanceBetweenCfgs(it_.first, q);
      if (calculated_distance < max_prm_edge_distance) {
        nearest_neighbors.push_back(it_.first);
      }
    }

    return nearest_neighbors;
  }

  DiscreteArmCfg getNearestConnectedNeighbor(double *map, int x_size,
                                             int y_size, DiscreteArmCfg q,
                                             int numofDOFs) {
    DiscreteArmCfg nearest_neighbor;
    long int distance = __INT64_MAX__;
    for (auto neighbor : graph) {
      if (neighbor.second->size() == 0) {
        continue;
      }

      if (validLine(map, x_size, y_size, q, neighbor.first, numofDOFs) &&
          (distanceBetweenCfgs(neighbor.first, q) < distance)) {
        nearest_neighbor = neighbor.first;
        distance = distanceBetweenCfgs(neighbor.first, q);
      }
    }
    return nearest_neighbor;
  }

  bool doesNodeExist(const DiscreteArmCfg &q) {
    return (graph.find(q) != graph.end());
  }

private:
  std::unordered_map<DiscreteArmCfg, NeighborList, NodeHash> graph;
};

typedef std::pair<DiscreteArmCfg, NodeData> Node;
typedef std::shared_ptr<Node> NodePtr;

class NodeComparator {
public:
  bool operator()(NodePtr A, NodePtr B) {
    return (A->second.f_val > B->second.f_val);
  }
};

typedef std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator>
    OpenList;

std::list<ContinuousArmCfg> aStarSearch(PRMGraph &graph, DiscreteArmCfg &start,
                                        DiscreteArmCfg &goal) {
  std::list<ContinuousArmCfg> empty_result;

  if (graph.getConnectedNeighbors(start)->size() == 0 ||
      graph.getConnectedNeighbors(goal)->size() == 0) {
    std::cout << "Not a feasible start and goal location!\n";
    return empty_result;
  }

  RRTree search_tree(start);
  OpenList open_list;
  std::unordered_set<DiscreteArmCfg, NodeHash> explored;

  NodeData start_node_data;
  start_node_data.cost = 0;
  start_node_data.f_val = getLineCost(start, goal);
  NodePtr start_node = std::make_shared<Node>(start, start_node_data);
  open_list.push(start_node);
  explored.insert(start);

  while (!open_list.empty()) {
    auto expanded_node = open_list.top();
    open_list.pop();
    long int expanded_node_g = expanded_node->second.cost;

    if (expanded_node->first == goal) {
      std::cout << "PRM Goal found!\n";
      std::list<ContinuousArmCfg> plan;

      while (!open_list.empty()) {
        open_list.pop();
      }

      return search_tree.generateReversedPlan(goal);
    }

    for (auto connected_neighbor :
         (*graph.getConnectedNeighbors(expanded_node->first))) {

      if (explored.count(connected_neighbor) != 0) {
        continue;
      }

      NodeData connected_neighbor_data;

      connected_neighbor_data.cost =
          expanded_node_g +
          getLineCost(expanded_node->first, connected_neighbor);

      connected_neighbor_data.f_val =
          connected_neighbor_data.cost + getLineCost(connected_neighbor, goal);

      NodePtr connected_neighbor_node =
          std::make_shared<Node>(connected_neighbor, connected_neighbor_data);

      search_tree.addNode(connected_neighbor, connected_neighbor_data.cost);
      search_tree.setParent(connected_neighbor, expanded_node->first);
      connected_neighbor_data.parent_ = expanded_node->first;

      open_list.push(connected_neighbor_node);
      explored.insert(connected_neighbor);
    }
  }

  return empty_result;
}

static void plannerPRM(double *map, int x_size, int y_size,
                       double *armstart_anglesV_rad,
                       double *armgoal_anglesV_rad, int numofDOFs,
                       double ***plan, int *planlength) {
  /* TODO: Replace with your implementation */
  ContinuousArmCfg arm_start_cont;
  makeVector(arm_start_cont, armstart_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_start = discretizeArmConfig(arm_start_cont);

  ContinuousArmCfg arm_goal_cont;
  makeVector(arm_goal_cont, armgoal_anglesV_rad, numofDOFs);
  DiscreteArmCfg arm_goal = discretizeArmConfig(arm_goal_cont);

  PRMGraph graph;

  for (long int iters = 0; iters < num_prm_points; iters++) {

    if ((iters % 100) == 0) {
      std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                   "~~~~~~~~~~~~~~~~~~~~\n";
      std::cout << "Iteration: " << iters << "\n";
    }

    DiscreteArmCfg q_rand = randomCfgGenerator(numofDOFs);
    if (graph.doesNodeExist(q_rand) ||
        !IsValidArmConfiguration(q_rand, map, x_size, y_size)) {
      continue;
    }

    graph.addNode(q_rand);
    auto neighbors = graph.getNearestNeighbors(q_rand);
    for (auto neighbor : neighbors) {
      if (validLine(map, x_size, y_size, neighbor, q_rand, numofDOFs)) {
        graph.addEdge(q_rand, neighbor);
      }
    }
  }

  std::cout << "Graph constructed\n";
  std::list<ContinuousArmCfg> generated_plan;
  generated_plan.push_back(arm_goal_cont);

  DiscreteArmCfg astar_goal = graph.getNearestConnectedNeighbor(
      map, x_size, y_size, arm_goal, numofDOFs);
  DiscreteArmCfg astar_start = graph.getNearestConnectedNeighbor(
      map, x_size, y_size, arm_start, numofDOFs);

  std::list<ContinuousArmCfg> a_star_search_result;

  if ((astar_goal.size() != 0) || (astar_start.size() != 0)) {
    a_star_search_result = aStarSearch(graph, astar_start, astar_goal);
  }

  generated_plan.insert(generated_plan.end(), a_star_search_result.begin(),
                        a_star_search_result.end());

  generated_plan.push_back(arm_start_cont);
  generateFinalPlan(generated_plan, plan);
  *planlength = generated_plan.size();

  if (a_star_search_result.size() == 0) {
    std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                 "~~~~~~~~~~\n";
    std::cout << "PLAN NOT FOUND! RETURNING DEFAULT\n";
    std::cout << "\n\n\n\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                 "~~~~~~~~~~~~~~~~~~~~\n";
  }
}

// clang-format off
//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. B    e careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
