#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits.h>

static bool DEBUG = false;

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

void twiddle(int simsteps, double tolerance)
{
  uWS::Hub h;
  double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  double dp = 1.0, di = 1.0, dd = 1.0;

  PID pidSteer;
  pidSteer.Init(Kp, Ki, Kd);
  double totalError = 0.0;
  double bestError = std::numeric_limits<double>::max();
  unsigned int nRuns = 0;

  h.onMessage([&pidSteer, &totalError, &bestError, simsteps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
           * Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe use
           * another PID controller to control the speed!
           */
          bool computeCumulativeError = false;
          if (pidSteer.GetNumSteps() >= simsteps/2) {
        	  computeCumulativeError = true;
          }
          pidSteer.UpdateError(cte, computeCumulativeError);
          double steer_value = pidSteer.ComputeControlValue();
          double throttle = 0.3;
          // DEBUG
          if (DEBUG) {
			  std::cout << pidSteer.GetNumSteps() << "> CTE: " << cte << " Steering Value: "
					  << steer_value << ", Throttle = " << throttle << "CurrentError = "
					  << pidSteer.CurrentError() << std::endl;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;

          if (simsteps > 0 && pidSteer.GetNumSteps() >= simsteps) {
        	  auto msg = "42[\"reset\"," + msgJson.dump() + "]";
        	  std::cout << msg << std::endl;
        	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        	  /* Twiddle State Computation */
        	  //Reset PID internal state
        	  //Reset Sim Steps
        	  //Recompute dp
        	  totalError = pidSteer.TotalError();
        	  if (totalError < bestError) {
        		  bestError = totalError;
        		  switch (nRuns % 3) {
        		  case 0:
        			  pidSteer.ResetInternalState(pidSteer.Kp * 1.1, pidSteer.Ki, pidSteer.Kd);
        			  break;
        		  case 1:
        			  pidSteer.ResetInternalState(pidSteer.Kp, pidSteer.Ki * 1.1, pidSteer.Kd);
        			  break;
        		  case 2:
        			  pidSteer.ResetInternalState(pidSteer.Kp, pidSteer.Ki, pidSteer.Kd * 1.1);
        			  break;
        		  }
        	  } else {

        	  }

          }
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
    return pidSteer.TotalError();
  }
  h.run();
}


void runSimulation(int simsteps, double tolerance)
{
  uWS::Hub h;
  double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  double dp = 1.0, di = 1.0, dd = 1.0;

  PID pidSteer;
  pidSteer.Init(Kp, Ki, Kd);
  double totalError = 0.0;

  h.onMessage([&pidSteer, &totalError, simsteps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
           * Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe use
           * another PID controller to control the speed!
           */
          bool computeCumulativeError = false;
          if (pidSteer.GetNumSteps() >= simsteps/2) {
        	  computeCumulativeError = true;
          }
          pidSteer.UpdateError(cte, computeCumulativeError);
          double steer_value = pidSteer.ComputeControlValue();
          double throttle = 0.3;
          // DEBUG
          if (DEBUG) {
			  std::cout << pidSteer.GetNumSteps() << "> CTE: " << cte << " Steering Value: "
					  << steer_value << ", Throttle = " << throttle << "CurrentError = "
					  << pidSteer.CurrentError() << std::endl;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;

          if (simsteps > 0 && pidSteer.GetNumSteps() >= simsteps) {
        	  auto msg = "42[\"reset\"," + msgJson.dump() + "]";
        	  std::cout << msg << std::endl;
        	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        	  totalError = pidSteer.TotalError();
        	  return;
          }
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
    return;
  }
  h.run();
}

int main()
{
	std::vector<double> pid_coeffs = twiddle();
	double err = runSimulation(10, pid_coeffs[0], pid_coeffs[1], pid_coeffs[2]);
	std::cout << "err = " << err << std::endl;
}
