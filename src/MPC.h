#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

/*********************************************************************
*********************************************************************/
// define parameters to be tuned later
//Set the timestep length and duration
#define STEP_SIZE 14
#define DT 0.05 // 0.05

// reference parameters
#define REF_CTE 0
#define REF_EPSI 0
#define REF_V 65 //60

// weights for cost function elements
#define WT_CTE 5 //1.0
#define WT_EPSI 10.0 //10.0
#define WT_V 0.5 // 1.0

#define WT_DELTA 1.0 //1.0, 60
#define WT_A 4.0 // 1.0, 15

#define WT_DELTA_DIFF 700
#define WT_A_DIFF 1.0 // 1.0

/*********************************************************************
*********************************************************************/
// define constant parameters
#define LF 2.67 // This is the length from front to CoG that has a similar radius.

#define MAX_LIMITS 1.0e19
#define STEER_LIMIT_RAD 0.436332
#define STEER_LIMIT_DEG 25
#define ACTUATOR_LIMIT 1.0

#define CURVE_ORDER 3
#define NUM_VARIABLES 6
#define NUM_ACTUATORS 2

const int latency_steps = 2; //(100ms/DT)
/*********************************************************************
*********************************************************************/

struct Result {
		vector<double> x;
		vector<double> y;
		vector<double> delta;
		vector<double> a;
};


/*********************************************************************
*********************************************************************/

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  //vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  double prev_delta {0};
  double prev_a {0};
};

#endif /* MPC_H */


