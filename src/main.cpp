#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "helper_functions.h"
#include "json.hpp"

//#define DEBUG

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


double curvature(Eigen::VectorXd coeffs, double x) {
	double firstDerivative = 3 * coeffs[3] * (x * x) + 2 * coeffs[2] * x + coeffs[1];
	double secondDerivative = 6 * coeffs[3] * x + 2 * coeffs[2];
	double R = pow((1 + pow(firstDerivative, 2.0)), 1.5) / abs(secondDerivative);

	return R;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  bool first = true;
  double steer_value;
  double throttle_value;

  h.onMessage([&mpc, &first, &steer_value, &throttle_value](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
#ifdef DEBUG
    cout << sdata << endl;
#endif
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double mpsV = mph2mps(v);

          // Transform waypoints from the map's coordinate system to the
          // vehicle's coordinate system
          for (int i = 0; i < (int)ptsx.size(); i++) {
        	  double x = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
        	  double y = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
        	  ptsx[i] = x;
        	  ptsy[i] = y;
          }

          // After coordinate system transformation, the vehicle's position
          // becomes the origin.
          px = py = psi = 0.0;

          Eigen::VectorXd waypointsX = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd waypointsY = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());
          auto coeffs = polyfit(waypointsX, waypointsY, 3);

          double cte = polyeval(coeffs, px) - py;
          double epsi = psi - atan(3 * coeffs[3] * (px * px) + 2 * coeffs[2] * px + coeffs[1]);

          Eigen::VectorXd state(6);
          if (!first) {
        	  // Compensate the actuation latency
        	  const double dt = 0.1;
        	  double x0 = 0.0;
        	  double y0 = 0.0;
        	  double psi0 = 0.0;
        	  double v0 = mpsV;

        	  px = v0 * cos(psi0) * dt;
        	  py = v0 * sin(psi0) * dt;
        	  psi = (v0 / Lf) * steer_value * dt;
        	  // Assume the velocity does not change during the 100 ms latency.
        	  mpsV = v0;

        	  double f0 = polyeval(coeffs, x0);
        	  double psides0 = atan(3 * coeffs[3] * (x0 * x0) + 2 * coeffs[2] * x0 + coeffs[1]);
        	  cte = f0 - y0 + v0 * sin(epsi) * dt;
        	  epsi = (psi0 - psides0) + (v0 / Lf) * steer_value * dt;

        	  state << px, py, psi, mpsV, cte, epsi;
          } else {
        	  first = false;
        	  state << 0, 0, 0, mpsV, cte, epsi;
          }


          /*
           * Slow down the vehicle if the turn is too sharp
           */
#ifdef DEBUG
          for (int i = 0; i < waypointsY.size(); i++)
            cout << "R=" << curvature(coeffs, waypointsY[i]) << "\n";
#endif
          double maxVelocity = 110;
          double radiusOfCurvature = curvature(coeffs, waypointsY[0]);
          if (radiusOfCurvature < 50)
            maxVelocity = 70;
          else if (radiusOfCurvature < 100)
            maxVelocity = 95;

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          vector<double> actuation = mpc.Solve(state, coeffs, maxVelocity);
          steer_value = actuation[0];
          throttle_value = actuation[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = (-1) * steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = std::move(mpc.x_vals);
          vector<double> mpc_y_vals = std::move(mpc.y_vals);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = std::move(ptsx);
          vector<double> next_y_vals = std::move(ptsy);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
