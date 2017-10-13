#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

/*
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
*/

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double p[3] = {0.13,0.006,5};
  double dp[3] = {0.01,0.001,1.};
  double err=0;
  int n = 0;
  int tol = 0.0001;
  int i=0;
  enum PIDState { started, increase, update_i, update_d };
  PIDState pid_state = PIDState::started;
  double best_err=std::numeric_limits<double>::max();
  pid.Init(p[0], p[1], p[2]); 
  

  h.onMessage([&pid,&p,&dp,&i,&err,&best_err,&n,&tol,&pid_state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          err += pow(cte,2);
          if(n==100){
            switch(pid_state){
              case PIDState::started:
                best_err = err;
                pid_state = PIDState::increase;
                break;
              case PIDState::increase:
                p[i] = p[i] + dp[i];
                pid_state = PIDState::update_i;
                break;
              case PIDState::update_i:
                if(err<best_err){
                  best_err = err;
                  dp[i]=dp[i]*1.1;
                  i = (i+1)%3;
                  pid_state = PIDState::increase;
                }else{
                  p[i] = p[i] - 2*dp[i];
                  pid_state = PIDState::update_d;
                }
                break;
              case PIDState::update_d:
                if(err<best_err){
                  best_err = err;
                  dp[i]=dp[i]*1.1;
                }else{
                  p[i] = p[i] + dp[i];
                  dp[i]=dp[i]*0.9;
                }
                i = (i+1)%3;
                pid_state = PIDState::increase;
                break;
            }
            pid.Init(p[0], p[1], p[2]);//update p values for pid controller
            std::cout << "I: " << i << " state:" << pid_state << std::endl;
            std::cout << "P {" << p[0] << ","<< p[1] <<","<< p[2] << "}" << std::endl;
            std::cout << "DP {" << dp[0] <<","<< dp[1] << "," << dp[2] << "}" << std::endl;
            std::cout << "ERROR " << err << ", BEST ERR "<< best_err << std::endl;
            std::cout << "-----------------------------------------"<< std::endl;
            err=0;
            n=0;
          }else{
            n+=1;
          }
          



          // Update error values with cte
          pid.UpdateError(cte);

          
          //steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
          steer_value = pid.TotalError();
          if(steer_value > 1.0) steer_value=1.0;
          if(steer_value < -1.0) steer_value=-1.0;
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
