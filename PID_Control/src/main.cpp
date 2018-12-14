#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <string>
#include <vector>

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
  pid.Init(0.0, 0.0, 0.0);
  vector<double> dp = {1.0, 1.0, 1.0};
  // Stop twiddling if the sum of dp values are less than 0.001
  double tol = 0.001;
  // Increase by 1 for each motion step
  int steps = 1;
  // Initialize best_err and the sum of square of total error over 100 steps
  double best_err = 100.0;
  double sum_square_error = 0.0;
  // Indicates weather control parameter change successfully reduced error in the previous 100 steps
  vector<bool> success = {true, true, true};

  h.onMessage([&pid, &dp, &tol, &steps, &best_err, &sum_square_error, &success](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    steps++;
    // Reset to zero if reached 100 steps
    if (steps == 100) {
      steps = 1;
    }
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
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          // Twiddle
          double sum_dp = dp[0] + dp[1] + dp[2];
          vector<double> controls = {pid.Kp, pid.Ki, pid.Kd};
          
          
          if (sum_dp < tol ) {
            for (int i = 0; i < controls.size(); i++) {
              // Update control parameters every 100 steps
              if (success[i]) {
                if (steps == 1) {
                  // Increase the i'th parameter only if error was successfuly reduced in the previous 100 steps
                  controls[i] += dp[i];
                  pid.assign_control_param(i, controls[i]);
                  // Re-initialize the sum of square error to zero
                  sum_square_error = 0.0;
                } 
                // Update parameters at the 100's step
                else if (steps == 100) {
                  sum_square_error += pow(steer_value, 2.0);
                  if (sum_square_error < best_err) {
                    success[i] = true;
                    best_err = sum_square_error;
                    dp[i] *= 1.1;
                  } else {
                    success[i] = false;
                    controls[i] -= 2*dp[i];
                    pid.assign_control_param(i, controls[i]);
                  }
                } else {
                  sum_square_error += pow(steer_value, 2.0);
                }
              }
              // If error was not successfully reduced
              else {
                if (steps == 1) {
                  // Re-initialize the sum of square error to zero
                  sum_square_error = 0.0;
                } else if (steps == 100) {
                  sum_square_error += pow(steer_value, 2.0);
                  if (sum_square_error < best_err) {
                    best_err = sum_square_error;
                    dp[i] *= 1.1;
                  } else {
                    controls[i] += dp[i];
                    dp[i] *= 0.9;
                  }
                } else {
                  sum_square_error += pow(steer_value, 2.0);
                }
              }
              
              /*
              if (steps == 1) {
                // Increase the i'th parameter only if error was successfuly reduced in the previous 100 steps
                if (success){
                  controls[i] += dp[i];
                  pid.assign_control_param(i, controls[i]);
                }
                // Re-initialize the sum of square error to zero
                sum_square_error = 0.0;
              } 
              // Update parameters at the 100's step
              else if (steps == 100) {
                sum_square_error += pow(pid.TotalError, 2.0);
                if (sum_square_error < best_err) {
                  success = true;
                  best_err = sum_square_error;
                  dp[i] *= 1.1;
                } else {
                  success = false;
                  controls[i] -= 2*dp[i];
                  pid.assign_control_param(i, controls[i]);
                }
              } else {
                sum_square_error += pow(pid.TotalError, 2.0);
              }
              */
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
