#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>
#include <iomanip> 
#include <math.h>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  // Vector that stores the delta values for control parameters
  vector<double> dp;
  // Vector that stores the control parameters
  vector<double> controls;
  // Stop twiddling if the sum of dp values exceeds the threshold
  double thresh;
  // Increase by 1 for each motion step, has three element, one for each control parameters
  vector<int> steps;
  int step;
  // Indicates wheter the previous run for twiddle has completed
  bool completed;
  // Do twiddle per this number of cycles
  int twiddle_cycle;
  // Initialize best_err and the sum of square of total erro, to be used for twiddle
  double best_err;
  double sum_square_error;
  // Indicates which round of twiddling is being performed
  int round;
  // Indicates whether twiddling for all 3 parameters has been done once
  vector<bool> done;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Assign a specific control paramter
  */
  void assign_control_param(int index, double value);
  
  /*
  * Implement twiddle to tune parameters
  */
  void twiddle();
  
  /*
  * Output control parameters
  */
  void output_param();
};

#endif /* PID_H */
