#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // Initialize delta values for control parameters
  dp = {0.01, 0.0001, 0.1};
  // Vector that stores the control parameters
  controls = {Kp, Ki, Kd};
  // Stop twiddling if the sum of dp values are less than this number
  thresh = 0.002;
  // Initialize the step value to be 0 for all control parameters
  steps = {0, 0, 0};
  step = 0;
  // Indicates wheter the previous run for twiddle has completed
  completed = true;
  // Do twiddle per this number of cycles
  twiddle_cycle = 100;
  // Initialize best_err and the sum of square of total error over a certain number of steps
  best_err = 10000.0;
  sum_square_error = 0.0;
  // Inidicates which round of twiddling is being performed
  round = 1;
  // Indicates whether twiddling for all 3 parameters has been done once
  done = {false, false, false};

  p_error = i_error = d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // p_error in the line below would be the previous cte
  d_error = cte - p_error; 
  p_error = cte;
  i_error += cte;
  
  // Update the number of steps, regardless of which parameter is being twiddled
  if (step < twiddle_cycle) {
    step ++;
    sum_square_error += cte*cte;
  } else {
    // re-initialize after reaching the last step
    step = 0; 
    sum_square_error = 0.0;
  }
  
}

double PID::TotalError() {
  return - Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::assign_control_param(int index, double value) {
  if (index == 0) {
    Kp = value;
  } else if (index == 1) {
    Ki = value;
  } else if (index == 2) {
    Kd = value;
  }
}

void PID::twiddle() {
  // looping through each control parameter
  for (int i = 0; i < controls.size(); i++) {
    // for debugging: to see if the second and third loop were reached
    if (i==1) {
      cout << "1" << endl;
    } else if (i==2) {
      cout << "2" << endl;
    }
    // if the i'th parameter has already been twiddled, continue to the next one
    if (done[i]) {
      continue;
    }
    // first round of twiddling
    if (round == 1) {
      // update the i'th control parameter
      if (steps[i] == 0) {
        controls[i] += dp[i];
        assign_control_param(i, controls[i]);
        steps[i] ++;
      }
      // do error comparision after finishing the twiddle cycle
      else if (steps[i] == twiddle_cycle - 1) {
        if (sum_square_error < best_err) {
          best_err = sum_square_error;
          dp[i] *= 1.1;
          done[i] = true;
        }
        else {
          controls[i] -= 2*dp[i];
          assign_control_param(i, controls[i]);
          round = 2; // need to do an extra round of twiddling for that parameter
        }
        // re-initialize steps[i]
        steps[i] = 0; 
      }
      // update steps[i]
      else {
        steps[i] ++;
      }    
    }
    if (round == 2) {
      if (steps[i] == twiddle_cycle - 1) {
        if (sum_square_error < best_err) {
          best_err = sum_square_error;
          dp[i] *= 1.1;
        }
        else {
          controls[i] += dp[i];
          assign_control_param(i, controls[i]);
          dp[i] *= 0.9;
        }
        steps[i] = 0;
        done[i] = true;
        round = 1; // re-initialize round
      }
      else {
        steps[i] ++;
      }
    }
    
    if (done[i]) {
      // reset the done vector once all three parameters are twiddled
      if (i==2) {
        done = {false, false, false};
        cout << "done twiddling!" << endl;
      }
      continue;
    }
    
    /* break out the loop if twiddling for the i'th parameter is not completed, 
    so that the other parameters would not be modified.*/
    break;
  }
}

void PID::output_param() {
  cout << "Kp: " << setprecision(7) << Kp << endl;
  cout << "Ki: " << setprecision(7) << Ki << endl;
  cout << "Kd: " << setprecision(7) << Kd << endl;
}
