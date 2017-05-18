#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

  best_err_ = 1e-9;


  Kp_changed_ = 0;
  Kp_changed2_ = 0;
  Kp_changed3_ = 0;

  Ki_changed_ = 0;
  Ki_changed2_ = 0;
  Ki_changed3_ = 0;

  Kd_changed2_ = 0;
  Kd_changed3_ = 0;

  time_steps_ = 0;
}

PID::~PID() {}



void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  dp_ = Kp_/2;
  di_ = Ki_/2;
  dd_ = Kd_/2;

  i_error = 0;
  total_error_ = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = i_error + cte;
  d_error = cte - last_cte;
  last_cte = cte;

  total_error_ += cte * cte;
}

double PID::TotalError() {
  time_steps_ += 1;
  return total_error_ / time_steps_; // compensate the number of loops
}

double PID::GetSteerValue(){
  double steer_value = -Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;

  if (steer_value < -1){
    steer_value = -1;
  }else if(steer_value > 1){
    steer_value = 1;
  }
  return steer_value;
}



void PID::Twiddle(){

  double sum_d_pid = dp_ + di_ + dd_;
  double err = TotalError();

  if (Kp_changed_ == 0){
      // Kp
      // run one time
      if (Kp_changed2_ == 0){
        Kp_ += dp_;

        Kp_changed2_ = 1;
        return;
      }

      if(err < best_err_){
        best_err_ = err;
        dp_ *= 1.1;

      }else{

        if(Kp_changed3_ == 0){
          Kp_ -= 2 * dp_;
          Kp_changed3_ = 1;
          // run one time
          return;
        }

        if(err < best_err_){
          best_err_ = err;
          dp_ *= 1.1;
        }else{
          Kp_ += dp_;
          dp_ *= 0.9;
        }
      }

      Kp_changed_ = 1;
  }


  if (Ki_changed_ == 0 && Kp_changed_ == 1){
      // Kp
      // run one time
      if (Ki_changed2_ == 0){
        Ki_ += di_;

        Ki_changed2_ = 1;
        return;
      }

      if(err < best_err_){
        best_err_ = err;
        di_ *= 1.1;

      }else{

        if(Ki_changed3_ == 0){
          Ki_ -= 2 * di_;
          Ki_changed3_ = 1;
          // run one time
          return;
        }

        if(err < best_err_){
          best_err_ = err;
          di_ *= 1.1;
        }else{
          Ki_ += di_;
          di_ *= 0.9;
        }
      }

      Ki_changed_ = 1;
  }

  if (Ki_changed_ == 1 && Kp_changed_ == 1){
      // Kp
      // run one time
      if (Kd_changed2_ == 0){
        Kd_ += dd_;

        Kd_changed2_ = 1;
        return;
      }

      if(err < best_err_){
        best_err_ = err;
        dd_ *= 1.1;

      }else{

        if(Kd_changed3_ == 0){
          Kd_ -= 2 * dd_;
          Kd_changed3_ = 1;
          // run one time
          return;
        }

        if(err < best_err_){
          best_err_ = err;
          dd_ *= 1.1;
        }else{
          Kd_ += di_;
          dd_ *= 0.9;
        }
      }

      Kp_changed_ = 0;
      Ki_changed_ = 0;
  }



}
