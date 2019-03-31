#include "PID.h"
#include <vector>
#include <uWS/uWS.h>

using std::vector;
//using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;

   p_error = 0;
   i_error = 0;
   d_error = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   double steer_value = (Kp * p_error) + (Ki * i_error) + (Kd * d_error);
   if (steer_value < -1){
     steer_value = -1;
   }else if (steer_value > 1){
     steer_value = 1;
   }

  return steer_value;  // TODO: Add your total error calc here!
}


void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


vector<double> PID::Twiddle(double tol, double p, int i, double error) {
  /**
   * TODO: Calculate and return the total error
   */
   vector<double> result;
   static bool is_init_twiddle = false;
   static unsigned count = 0;
   static vector<double> dp = {0.001,0.001,0.001};
   static double best_error = error;
   static double tole = 3;


   if (!is_init_twiddle){
     is_init_twiddle = true;
   }
   else {
     if (count > 2)
       count = 0;

     if (count == 0){
       p += dp[i];
     }
     else {
         if(count == 1){
             if (error < best_error){
               best_error = error;
               dp[i] *= 1.1;
             }
             else{
               p -= 2*dp[i]; }
             }
             else{
                if (count == 2){
                  if (error < best_error){
                    best_error = error;
                    dp[i] *= 1.1;
                }
                  else{
                    p += dp[i];
                    dp[i] *= 0.9;
                  }
                }
            }
     }
  count ++;
  }
  tole = dp[0]+dp[1]+dp[2];
  result = {double(count), p, tole, error};
  return result;  // TODO: Add your total error calc here!
}
