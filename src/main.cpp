#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>

using std::vector;


// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
   //static double Kp_initial = -0.091;
   //static double Ki_initial = -0.0005;
   //static double Kd_initial = -1.693;

   static double Kp_initial = -0.162158;
   static double Ki_initial = -0.000008;
   static double Kd_initial = -2.5;

   static double thro = 0.3;
   static double speed_limit = 10;

   pid.Init(Kp_initial, Ki_initial, Kd_initial);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static unsigned int timesteps = 0;
    static double error = 0.0;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          static bool p_flag = false;
          static bool i_flag = false;
          static bool d_flag = false;
          static bool do_twiddle = true;
          static int count_out;
          vector<double> result;
          //pid.Restart(ws);


          if (do_twiddle){
            if (timesteps > 1000){
                  if (!p_flag && !i_flag && !d_flag){
                      result = pid.Twiddle(0.2,Kp_initial,0,error);
                      Kp_initial = result[1];
                      count_out = int(result[0]);
                      if (count_out > 2) p_flag = true;
                  }
                  else if (p_flag && !i_flag && !d_flag){
                      result = pid.Twiddle(0.2,Ki_initial,1, error);
                      Ki_initial = result[1];
                      count_out = int(result[0]);
                      if (count_out > 2) i_flag = true;
                  }
                  else if (p_flag && i_flag && !d_flag){
                      result = pid.Twiddle(0.2,Kd_initial,2, error);
                      Kd_initial = result[1];
                      count_out = int(result[0]);
                      if (count_out > 2){
                          p_flag = false;
                          i_flag = false;
                          d_flag = false;
                      }
                  }

                  pid.Init(Kp_initial, Ki_initial, Kd_initial);
                  pid.Restart(ws);
                  timesteps = 0;
                  error = 0.0;
                  if (result[2] < 0.0000000001) do_twiddle = false;
                  return;
            }
            else{
              timesteps++;
              error += pow(cte, 2);
            }

          }


           //Update the error in system using cte.
          pid.UpdateError(cte);

          //Derive the steering angle in order to minimize the error (or cte).
          steer_value = pid.TotalError();
/*
          if (speed_limit < speed)
              thro = 0;
          else
              thro = 0.3;
*/
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: "<< steer_value
          << " Kp: "<< pid.Kp<< " Ki: "<< pid.Ki<< " Kd: "<< pid.Kd<<std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = thro;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
