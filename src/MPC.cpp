//----------
// Model Prediction Controller (MPC) Class for MPC project. This module talks to the Udacity simulator and receives data
// back from the Simulator's vehicle through WebSocket messages. This was adapted from starter code from
// Udacity which I modified to handle the results of the "telemetry" message.
//
// State Vector for this model is: [px, py, psi, v, cte, epsi]
//
// Note: Since this is an event driven operation (i.e. process messages from the Udacity Simulator Server) and you have to pass
// objects to the Websocket message handler, I have chosen to implement support routines like saving data, plotting data, etc.
// as methods of the MPC class
//
// Note: Important Constants for this code are as follows:
//
// size_t N = 25;     // 16(@60) 50 (@50mph)  50  100 75   40
// double dt = 0.03;  // .05    .02          .03 .03  .05   .05
//
// This is the length from front to CoG that has a similar radius.
// const double Lf = 2.67;
//
// Reference Car Velocity in mph
// double ref_v = 50;  // Up to 60 works
//----------
#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "utils.hpp"  // My utility library

//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

using CppAD::AD;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180.0; }
//double rad2deg(double x) { return x * 180.0 / pi(); }

//-----
// Constants Section !!!
// TODO: Set the timestep length and duration
//-----
size_t N = 25;    // 16(@60) 50 (@50mph)  50  100 75   40
double dt = 0.03; // .05    .02          .03 .03  .05   .05

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
// Reference Car Velocity in mph
const double ref_v = 50;  // Up to 60 works


const double delta_w = 600000.0;
const double cte_w   = 2.0;




// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


    vector<double> px_sav,py_sav;      // Save points for diagnostic data
    vector<double> ptsx_sav,ptsy_sav;






// Sub-Class ????
class FG_eval {
 
public:
  
    // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
      // The cost is stored is the first element of `fg`.
      // Any additions to the cost should be added to `fg[0]`.
      fg[0] = 0;
      
      // Reference State Cost
      // TODO: Define the cost related the reference state and
      // any anything you think may be beneficial.
      // The part of the cost based on the reference state.
      for (int t = 0; t < N; t++) {
          fg[0] += cte_w * CppAD::pow(vars[cte_start + t], 2);
          fg[0] += CppAD::pow(vars[epsi_start + t], 2);
          fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      }
      
      // Minimize the use of actuators.
      for (int t = 0; t < N - 1; t++) {
          fg[0] += CppAD::pow(vars[delta_start + t], 2);
          fg[0] += CppAD::pow(vars[a_start + t], 2);
      }
      
      // Minimize the value gap between sequential actuations.
      for (int t = 0; t < N - 2; t++) {
          fg[0] += delta_w * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
          fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }
      
      
      //
      // Setup Constraints
      //
      // NOTE: In this section you'll setup the model constraints.
      
      // Initial constraints
      //t
      // We add 1 to each of the starting indices due to cost being located at
      // index 0 of `fg`.
      // This bumps up the position of all the other values.
      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];
      
      // The rest of the constraints
      for (int t = 1; t < N; t++) {
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> v0 = vars[v_start + t - 1];
          AD<double> cte0 = vars[cte_start + t - 1];
          AD<double> epsi0 = vars[epsi_start + t - 1];

          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> v1 = vars[v_start + t];
          AD<double> cte1 = vars[cte_start + t];
          AD<double> epsi1 = vars[epsi_start + t];
          
          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];
          AD<double> a0 = vars[a_start + t - 1];
          
          // THIS IS UNIQUE CHECK PSIDES0!!!!!!!
          AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
          AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0*coeffs[2]*x0 + 3.0*coeffs[3]*x0*x0);
          
          // Here's `x` to get you started.
          // The idea here is to constraint this value to be 0.
          //
          // NOTE: The use of `AD<double>` and use of `CppAD`!
          // This is also CppAD can compute derivatives and pass
          // these to the solver.
          // Recall the equations for the model:
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
          // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
          // v_[t+1] = v[t] + a[t] * dt
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          //---
          // TODO: Setup the rest of the model constraints
          // Global Kinematic Model
          fg[1 + x_start + t]    = x1 -    (x0 + v0 * CppAD::cos(psi0) * dt);
          fg[1 + y_start + t]    = y1 -    (y0 + v0 * CppAD::sin(psi0) * dt);
          fg[1 + psi_start + t]  = psi1 -  (psi0 + (v0/Lf) * delta0 * dt);
          fg[1 + v_start + t]    = v1 -    (v0 + a0 * dt);
          fg[1 + cte_start + t]  = cte1 -  (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
          fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + (v0/Lf) * delta0 * dt);
      }

  }
}; // FG_eval sub-class

//
// MPC class definition implementation.
//
MPC::MPC() {}


MPC::~MPC() {}

//


//
// MPC Method Solve the state path
//
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    
    bool ok = true;
    //size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];
    
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  //size_t n_vars = 0;
  // TODO: Set the number of constraints
  //size_t n_constraints = 0;
    
    // Number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 6 + (N - 1) * 2;
    
    // Number of constraints
    size_t n_constraints = N * 6;
    

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;
    
    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // TODO: Set lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
        //vars_lowerbound[i] = -__DBL_EPSILON__;
        //vars_upperbound[i] = __DBL_EPSILON__;
    }
    
    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
         vars_upperbound[i] = 0.436332;
        //vars_lowerbound[i] = deg2rad(-25.0);
        //vars_upperbound[i] = deg2rad(25.0);
    }
    
    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;
    
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
    
    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          1.0\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
   // bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  cout << "MPC: Cost=" << cost << endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
    
    /*
    return {solution.x[x_start + 1],   solution.x[y_start + 1],
        solution.x[psi_start + 1], solution.x[v_start + 1],
        solution.x[cte_start + 1], solution.x[epsi_start + 1],
        solution.x[delta_start],   solution.x[a_start]};
    */
    
    vector<double> output;
    
    output.push_back(solution.x[delta_start]); // First actuation
    output.push_back(solution.x[a_start]);     // Second actuation
    
    for (int i=1; i<N; i++) {
        output.push_back(solution.x[x_start + i]);
        output.push_back(solution.x[y_start + i]);
    
    }
    return output;
}



//
//
//
void MPC::SaveData(vector<double> &ptsx, vector<double> &ptsy, double &px, double &py) {

    for (int i=0; i<ptsx.size(); i++) {
        ptsx_sav.push_back(ptsx[i]);
        ptsy_sav.push_back(ptsy[i]);
    }
    
    px_sav.push_back(px);
    py_sav.push_back(py);
}


//
// Not working because of some linking to python problem
//
void MPC::PlotData() {
    
    // Plot values
    // NOTE: feel free to play around with this.
    // It's useful for debugging!
    /*
    plt::subplot(3, 1, 1);
    plt::title("CTE");
    plt::plot(cte_vals);
    plt::subplot(3, 1, 2);
    plt::title("Delta (Radians)");
    plt::plot(delta_vals);
    plt::subplot(3, 1, 3);
    plt::title("Velocity");
    plt::plot(v_vals);

    //plt::subplot(1, 1, 1);
    plt::title("Car Path in Simulator");
    plt::xlabel("X Pos");
    plt::ylabel("Y Pos");
    //plt::plot(ptsx_sav);
    //plt::plot(px_sav,".-");
    
    plt::show();
     
     */
}

