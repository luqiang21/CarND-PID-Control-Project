#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#define TWIDDLE 1
#define N 2000

/*
  From wiki:
  P accounts for present values of the error. For example, if the error is
  large and positive, the control output will also be large and positive.
  I accounts for past values of the error. For example, if the current output is
  not sufficiently strong, the integral of the error will accumulate over time,
  and the controller will respond by applying a stronger action.
  D accounts for possible future trends of the error, based on its current rate of change.

  First time I set (Kp, Ki, Kd) to like (1, 1. 1), and apply twiddle
  every 2000 time steps. Compute the error for 2000 time steps, and then restart.
  The car just drive using large steering angle, and
  immediately out of road and doing circular motion.

  I observed the error for each item, I realized that the i_error is very large,
  so I tried Ki for 0.001. This time, the car drive well for the straight road,
  goes off the road when it is near the curve. So I decided to use 1000 time steps
  to tune for better parameters.

  Every time, I used the best parameters with the lowest error as the initial values
  for next run.

  I used 0.01,0.001,1 for parameters and 1,1,1 for their corresponding increments.
  After long time, I obtained good parameters: 0.159581  0   1.39897.

  By adjusting the initial values several times, I obtained the final best parameters.

  I also observed that, if the Kp is too big, the path oscillation is to strong.


*/

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // double Kp = .1;
  // double Ki = 0.000001;
  // double Kd = 1.5;
  double Kp = 1;
  double Ki = 0.001;
  double Kd = 1;
  // pid.Init(Kp, Ki, Kd);

  // pid.Init(0, 0, 0);

  // pid.Init(1, 0, 1.28187);
  // pid.Init(.01, 0.001, 1);
  pid.Init(0.159581,  0,   1.39897);
  pid.dp_ = 0.1;
  pid.di_ = 0.0001;
  pid.dd_ = 0.1;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          These are the lower and upper barriers for the input, so -1 == -25 degree 1 == 25 degree
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // TODO if CTE is greater than 2, restart, and compute error every N=100 steps.

          // std::cout<<"time step: " <<pid.time_steps_ <<std::endl;
          // if(TWIDDLE == 1 && pid.time_steps_ == N){

          if(TWIDDLE == 1 && std::abs(cte) > 2.2 && pid.time_steps_ > 0){
            std::cout << "CTE is " << cte << std::endl;
            pid.Twiddle();

            std::cout << "current error " << pid.total_error_  << " best error " << pid.best_err_ << std::endl;
            std::cout << "dp, di, dd: "<< std::endl;
            std::cout << pid.dp_ << " " << pid.di_ << " " << pid.dd_ << std::endl;
            std::cout << "Kp, Ki, Kd: "<< std::endl;
            std::cout << pid.Kp_ << "  " << pid.Ki_ << "   "<< pid.Kd_ << std::endl;
            std::cout<<"p_error_ " << "i_error_ " << "d_error_ "<<std::endl;
            std::cout << pid.p_error_ << "  "<<pid.i_error_<<"  "<<pid.d_error_<<std::endl;

            std::cout << "time step is " << pid.time_steps_ << std::endl;
            std::cout << std::endl;


            pid.time_steps_ = 0;
            pid.total_error_ = 0;
            pid.i_error_ = 0;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }else{
            pid.UpdateError(cte);
            steer_value = pid.GetSteerValue();
            // steer_value = 2 * (1 / (1 + exp(-steer_value))) - 1; //sigmoid


          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
