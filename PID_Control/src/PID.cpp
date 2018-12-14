#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  p_error = i_error = d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // p_error in the line below would be the previous cte
  d_error = cte - p_error; 
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return - Kp * d_error - Ki * i_error - Kd * d_error;
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

