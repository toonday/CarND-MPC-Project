#ifndef MPC_H
#define MPC_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

inline double deg_to_rad (double deg) {
    return deg * M_PI / 180.0;
}

inline double rad_to_deg (double rad) {
    return rad * 180.0 / M_PI;
}

#endif /* MPC_H */
