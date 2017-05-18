#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = i_error + cte;
  d_error = cte - last_cte;
  last_cte = cte;
}

double PID::TotalError() {
  double total_error;
  total_error = -Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;
  return total_error;
}

double PID::GetSteerValue(double steer_value){
  if (steer_value < -1){
    steer_value = -1;
  }else if(steer_value > 1){
    steer_value = 1;
  }
  return steer_value;
}
