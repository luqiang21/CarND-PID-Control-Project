#include "PID.h"
// #define minN 100
#include <iostream>


/*
* TODO: Complete the PID class.
*/

PID::PID() {

  best_err_ = 1e9;


  Kp_changed_ = 0;
  Kp_changed2_ = 0;
  Kp_changed3_ = 0;

  Ki_changed_ = 0;
  Ki_changed2_ = 0;
  Ki_changed3_ = 0;

  Kd_changed2_ = 0;
  Kd_changed3_ = 0;


}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  dp_ = Kp_/4;
  di_ = Ki_/4;
  dd_ = Kd_/4;

  i_error_ = 0;
  time_steps_ = 0;

  total_error_ = 0;

}

void PID::UpdateError(double cte) {
  p_error_ = cte;
  i_error_ = i_error_ + cte;
  d_error_ = cte - last_cte_;
  last_cte_ = cte;

  // if(time_steps_ > minN){
    total_error_ += cte * cte;
  // }
  time_steps_ += 1;

  total_error_ /= (time_steps_ );//- minN);

}


double PID::GetSteerValue(){
  double steer_value = -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;

  if (steer_value < -1){
    steer_value = -1;
  }else if(steer_value > 1){
    steer_value = 1;
  }
  return steer_value;
}


void PID::Reset(){
  Kp_changed_ = 0;
  Kp_changed2_ = 0;
  Kp_changed3_ = 0;

  Ki_changed_ = 0;
  Ki_changed2_ = 0;
  Ki_changed3_ = 0;

  Kd_changed2_ = 0;
  Kd_changed3_ = 0;
}

using namespace std;
void PID::Twiddle(){

  double err = total_error_;

  if (Kp_changed_ == 0){
      // Kp
      // run one time
      if (Kp_changed2_ == 0){
        Kp_ += dp_;

        Kp_changed2_ = 1;
        std::cout << "Kp 1st run" << std::endl;
        return;
      }

      if(err < best_err_){
        best_err_ = err;
        dp_ *= 1.1;
        // Reset();
        std::cout << "Kp better result, 1st run, end" << std::endl;
        Kp_changed_ = 1;

        return;
      }else{

          if(Kp_changed3_ == 0){
            Kp_ -= 2 * dp_;
            Kp_changed3_ = 1;
            // run one time
            std::cout << "Kp 2nd run" << std::endl;

            return;
          }

          if(err < best_err_){
            best_err_ = err;
            dp_ *= 1.1;
            // Reset();
            std::cout << "Kp better result, end" << std::endl;
            Kp_changed_ = 1;

            return;

          }else{
            Kp_ += dp_;
            dp_ *= 0.9;
            // Reset();
            std::cout << "Kp 2nd run, no better result, end" << std::endl;
            Kp_changed_ = 1;

            return;

          }
      }

  }


  if (Ki_changed_ == 0 && Kp_changed_ == 1){
      // Kp
      // run one time
      if (Ki_changed2_ == 0){
        Ki_ += di_;

        Ki_changed2_ = 1;
        std::cout << "Ki 1st run" << std::endl;

        return;
      }

      if(err < best_err_){
        best_err_ = err;
        di_ *= 1.1;
        // Reset();
        std::cout << "Ki  better result, 1st run, end" << std::endl;
        Ki_changed_ = 1;

        return;
      }else{

        if(Ki_changed3_ == 0){
          Ki_ -= 2 * di_;
          Ki_changed3_ = 1;
          // run one time
          std::cout << "Ki 2nd run" << std::endl;

          return;

        }

        if(err < best_err_){
          best_err_ = err;
          di_ *= 1.1;
          // Reset();
          std::cout << "Ki, better result, end" << std::endl;
          Ki_changed_ = 1;

          return;

        }else{
          Ki_ += di_;
          di_ *= 0.9;
          // Reset();
          std::cout << "Ki 2nd run, no better result, end" << std::endl;
          Ki_changed_ = 1;

          return;

        }
      }

  }

  if (Ki_changed_ == 1 && Kp_changed_ == 1){
      // Kp
      // run one time
      if (Kd_changed2_ == 0){
        Kd_ += dd_;

        Kd_changed2_ = 1;
        std::cout << "Kd 1st run" << std::endl;

        return;;
      }

      if(err < best_err_){
        best_err_ = err;
        dd_ *= 1.1;
        Reset();
        std::cout << "Kd  better result, 1st run, end" << std::endl;

        return;
      }else{

        if(Kd_changed3_ == 0){
          Kd_ -= 2 * dd_;
          Kd_changed3_ = 1;
          // run one time
          std::cout << "Kd 2nd run" << std::endl;

          return;
        }

        if(err < best_err_){
          best_err_ = err;
          dd_ *= 1.1;
          Reset();
          std::cout << "Kd, better result, end" << std::endl;
          return;
        }else{
          Kd_ += dd_;
          dd_ *= 0.9;
          Reset();
          std::cout << "Kd 2nd run, no better result, end" << std::endl;
          return;
        }
      }

  }



}
