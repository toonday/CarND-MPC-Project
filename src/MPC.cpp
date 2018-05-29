#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG (Center of Gravity) that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0.0;
double ref_epsi = 0.0;
double ref_vel = 60.0;

size_t x_start = 0;
size_t y_start = x_start + (1 * N);
size_t psi_start = x_start + (2 * N);
size_t vel_start = x_start + (3 * N);
size_t cte_start = x_start + (4 * N);
size_t epsi_start = x_start + (5 * N);
size_t rot_actuator_start = x_start + (6 * N);
size_t lin_actuator_start = x_start + (7 * N) - 1;

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
	
	// ===============================================
	// Set the Cost Function fg[0]
	// ===============================================
    fg[0] = 0.0;
	
	double tune_params[8] = {
		9000.0 /*cte*/,
		9000.0 /*epsi*/,
		1.0 /*vel*/,
		5.0 /*rot_actuator_start*/,
		5.0 /*lin_actuator_start*/,
		8000000.0 /*rot_actuator_start_delta*/,
		80.0 /*lin_actuator_start_delta*/,
	};

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += tune_params[0] * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += tune_params[1] * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += tune_params[2] * CppAD::pow(vars[vel_start + t] - ref_vel, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += tune_params[3] * CppAD::pow(vars[rot_actuator_start + t], 2);
      fg[0] += tune_params[4] * CppAD::pow(vars[lin_actuator_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += tune_params[5] * CppAD::pow(vars[rot_actuator_start + t + 1] - vars[rot_actuator_start + t], 2);
      fg[0] += tune_params[6] * CppAD::pow(vars[lin_actuator_start + t + 1] - vars[lin_actuator_start + t], 2);
    }
	
	// ===============================================
	// Setup Constraints
	// ===============================================
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + vel_start] = vars[vel_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
	
	for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> fut_x = vars[x_start + t];
      AD<double> fut_y = vars[y_start + t];
      AD<double> fut_psi = vars[psi_start + t];
      AD<double> fut_vel = vars[vel_start + t];
      AD<double> fut_cte = vars[cte_start + t];
      AD<double> fut_epsi = vars[epsi_start + t];

      // The state at time t.
      AD<double> curr_x = vars[x_start + t - 1];
      AD<double> curr_y = vars[y_start + t - 1];
      AD<double> curr_psi = vars[psi_start + t - 1];
      AD<double> curr_vel = vars[vel_start + t - 1];
      AD<double> curr_cte = vars[cte_start + t - 1];
      AD<double> curr_epsi = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> curr_rot_actuator = vars[rot_actuator_start + t - 1];
      AD<double> curr_lin_actuator = vars[lin_actuator_start + t - 1];

      AD<double> desired_y = coeffs[0] + coeffs[1] * curr_x + coeffs[2] * CppAD::pow(curr_x, 2) + coeffs[3] * CppAD::pow(curr_x, 3);
      AD<double> desired_psi = CppAD::atan((3 * coeffs[3] * CppAD::pow(curr_x, 2)) + (2 * coeffs[2] * curr_x) + coeffs[1]);

      fg[1 + x_start + t] = fut_x - (curr_x + (curr_vel * CppAD::cos(curr_psi) * dt));
      fg[1 + y_start + t] = fut_y - (curr_y + (curr_vel * CppAD::sin(curr_psi) * dt));
      fg[1 + psi_start + t] = fut_psi - (curr_psi - (curr_vel * curr_rot_actuator / Lf * dt));
      fg[1 + vel_start + t] = fut_vel - (curr_vel + (curr_lin_actuator * dt));
      fg[1 + cte_start + t] = fut_cte - ((desired_y - curr_y) + (curr_vel * CppAD::sin(curr_epsi) * dt));
      fg[1 + epsi_start + t] = fut_epsi - ((curr_psi - desired_psi) - (curr_vel * curr_rot_actuator / Lf * dt));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double vel = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[vel_start] = vel;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // ===============================================
  // Set lower and upper limits for variables
  // ===============================================
  for (size_t i = 0; i < rot_actuator_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  for (size_t i = rot_actuator_start; i < lin_actuator_start; i++) {
    vars_lowerbound[i] = -deg_to_rad(25.0) * Lf;
    vars_upperbound[i] = deg_to_rad(25.0) * Lf;
  }
  
  for (size_t i = lin_actuator_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // ===============================================
  // Set lower and upper limits for inital constraints
  // ===============================================
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[vel_start] = vel;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[vel_start] = vel;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
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
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  vector<double> result;

  result.push_back(solution.x[rot_actuator_start]);
  result.push_back(solution.x[lin_actuator_start]);

  for (size_t i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
