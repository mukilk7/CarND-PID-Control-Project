#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits.h>

static bool DEBUG = true;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

/*
 * The entire twiddle loop implementation state is maintained
 * partially in this function and in the PID controller. Global
 * state like best error is maintained in this function whereas
 * PID controller specific state is maintained internally in
 * PID.
 */
void twiddle(int simsteps, double tolerance) {
	uWS::Hub h;
	std::vector<double> d{1.0,1.0,1.0};
	std::vector<double> k{0.0 + d[0], 0.0, 0.0};
	PID pidSteer;
	pidSteer.Init(k);
	pidSteer.currCoeff = 0;
	pidSteer.currOpState = 0;
	double bestError = std::numeric_limits<double>::max();
	static int iteration = 0;
	static bool firstRun = true;

	h.onMessage(
			[&pidSteer, &bestError, &d, simsteps, tolerance](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
					double throttle = 0.05;
					// DEBUG
					if (DEBUG) {
						std::cout << pidSteer.GetNumSteps() << "> CTE: " << cte << " Steering Value: "
								<< steer_value << ", Throttle = " << throttle << ", CurrentError = "
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

						//Check if sum(d*) is below tolerance and quit
						if ((d[0] + d[1] + d[2]) <= tolerance || iteration == 500) {
							std::cout << "{Kp, Ki, Kd} = {" << pidSteer.Kp <<
									", " << pidSteer.Ki <<
									", " << pidSteer.Kd << "}" << std::endl;
							exit(0);
						}

						double totalError = pidSteer.TotalError();
						if (firstRun || totalError < bestError) {
							bestError = totalError;
							//bump up d[*currCoeff] by 1.1 for next use
							d[pidSteer.currCoeff] *= 1.1;
							//incr currCoeff
							pidSteer.currCoeff = (pidSteer.currCoeff + 1) % 3;
							//set currOpState to LOOP_START i.e., 0
							pidSteer.currOpState = 0;
							firstRun = false;
						} else {
							//if currOpState = LOOP_START
							//then set currOpState to UNDONE_DIR i.e., 1
							//decrement *currCoeff by 2 * d[*currCoeff]
							//retain currentCoeff
							//if currOpState = UNDONE_DIR
							//then set currOpState to LOOP_START
							//increment *currCoeff by d[*currCoeff]
							//bump down d[*currCoeff] by 0.9 for next use
							//incr currCoeff
							if (pidSteer.currOpState == 0) {
								pidSteer.currOpState = 1;
								std::vector<double> coeffs{pidSteer.Kp, pidSteer.Ki, pidSteer.Kd};
								coeffs[pidSteer.currCoeff] -= (2 * d[pidSteer.currCoeff]);
								pidSteer.Init(coeffs);
							} else if(pidSteer.currOpState == 1) {
								pidSteer.currOpState = 0;
								std::vector<double> coeffs{pidSteer.Kp, pidSteer.Ki, pidSteer.Kd};
								coeffs[pidSteer.currCoeff] += d[pidSteer.currCoeff];
								d[pidSteer.currCoeff] *= 0.9;
								pidSteer.currCoeff = (pidSteer.currCoeff + 1) % 3;
								pidSteer.Init(coeffs);
							}
						}
						//*currCoeff += d[*currCoeff]
						//Re-Init PID for next loop
						std::vector<double> coeffs{pidSteer.Kp, pidSteer.Ki, pidSteer.Kd};
						coeffs[pidSteer.currCoeff] += d[pidSteer.currCoeff];
						pidSteer.Init(coeffs);
						std::cout << "Iteration " << iteration++ << ": " <<
								", Best Error = " << bestError <<
								", {Kp, Ki, Kd} = {" << pidSteer.Kp <<
								", " << pidSteer.Ki <<
								", " << pidSteer.Kd << "}" << std::endl;

					}
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << ", step = " << pidSteer.GetNumSteps()
											<< ", CTE = " << cte << std::endl;
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
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return;
	}
	h.run();
}

void drive() {
	uWS::Hub h;

	PID pidSteer;
	//std::vector<double> coeffs{1.5, 0.35, 0.25};
	std::vector<double> coeffs{0.12, 0, 2.5};
	pidSteer.Init(coeffs);

	h.onMessage(
			[&pidSteer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
					pidSteer.UpdateError(cte, false);
					double steer_value = pidSteer.ComputeControlValue();
					double throttle = 0.03;
					// DEBUG
					if (DEBUG) {
						std::cout << pidSteer.GetNumSteps() << "> CTE: " << cte << " Steering Value: "
								<< steer_value << ", Throttle = " << throttle << ", CurrentError = "
								<< pidSteer.CurrentError() << std::endl;
					}
					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle;

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
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return;
	}
	h.run();
}

int main() {
	//double err = runSimulation(10, pid_coeffs[0], pid_coeffs[1], pid_coeffs[2]);
	std::cout << "In main" << std::endl;
	//twiddle(200, 0.2);
	drive();
}
