#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

#include "utils.hpp"

// for convenience
using json = nlohmann::json;
using namespace Eigen;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
//double deg2rad(double x) { return x * pi() / 180.0; }
//double rad2deg(double x) { return x * 180.0 / pi(); }


//----------
//  Helper Method Section
//
//
//----------

// Checks if the SocketIO event has JSON data.  If there is data the JSON object in string format will be returned,
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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


void WorldToVehicleTransform(const double &x_wrld, const double &y_wrld, const double &theta_wrld,
                             const double &x_in,   const double &y_in, double &x_out, double &y_out) {
    
    // Pre-compute rotation
    double cos_t = cos(theta_wrld);
    double sin_t = sin(theta_wrld);
    
    // Align origin Center to vehicle
    double delta_x = x_in - x_wrld;
    double delta_y = y_in - y_wrld;
    
    // NOTE THE NEGATIVE VEHICLE ANGLE - MUST FIX!!!!
    //rotate(delta_x, delta_y, out_x, out_y, -veh_psi);

    // 2D translate & rotate world to vehicle
    // x_prime = x*cos(theta) - y*sin(theta)
    // y_prime = x*sin(theta) + y*cos(theta)
    
    x_out = delta_x*cos_t - delta_y*sin_t;
    y_out = delta_x*sin_t + delta_y*cos_t;
    
    
    /*
    x_out = x_in*cos_t - y_in*sin_t - x_wrld;
    y_out = x_in*sin_t + y_in*cos_t - y_wrld;
    */
    
    
    
    return;
    
}

// Method to convert the planned path (waypoints) from map space to vehicle space

void ConvertMapWaypointsToVehicleCoord(vector<double> &ptsx, vector<double> &ptsy, const double &theta_wrld,
                    const double &x_in, const double &y_in, vector<double> &ptsx_v, vector<double> &ptsy_v) {
    
    double ptsx_v_point, ptsy_v_point;   // Planned path point (x,y) in vehicle space
    
    
    // Pre-compute rotation
    double theta = -(theta_wrld + pi()); // In Map Space x-axis points down, in Vehicle Space x-axis points straight up. Also
                                         // by negative rotate, y-axis points to left which is correct for vehicle orientation
    double cos_t = cos(theta);
    double sin_t = sin(theta);
   
    for (int i=0; i<ptsx.size(); i++) {
    
        // Align origin to vehicle center
        double delta_x = x_in - ptsx[i];
        double delta_y = y_in - ptsy[i];
    
        // Compute transform
        ptsx_v_point = delta_x*cos_t - delta_y*sin_t;
        ptsy_v_point = delta_x*sin_t + delta_y*cos_t;
        
        // Add to output vector
        ptsx_v.push_back(ptsx_v_point);
        ptsy_v.push_back(ptsy_v_point);
    }
}


void VehicleToWorld(const double &xvehicle_wrld, const double &yvehicle_wrld, const double &thetavehicle_wrld,
                    const double &x_in, const double &y_in, double &x_out, double &y_out) {
    
    //double out_x_, out_y_;
    
    // Pre-compute rotation
    double cos_t = cos(thetavehicle_wrld);
    double sin_t = sin(thetavehicle_wrld);
    
    //rotate(in_x, in_y, out_x_, out_y_, veh_psi);
    // 2D translate & rotate world to vehicle
    // x_prime = x*cos(theta) - y*sin(theta)
    // y_prime = x*sin(theta) + y*cos(theta)
    x_out = x_in*cos_t - y_in*sin_t;
    y_out = x_in*sin_t + y_in*cos_t;
    
    // Add vehicle location in world
    x_out += xvehicle_wrld;
    y_out += yvehicle_wrld;
    
    return;
}









//----------
//  Main Method
//----------
int main() {
  
    uWS::Hub h;
    MPC mpc;  // MPC CLASS is initialized here!
    int iters = 20;
    
    //-----
    // Message Handler Section before start
    //  onMessage is triggered each time the Udacity Simulator sends data through WebSocket
    //----
    
    // Message hook & processing from Udacity Simulator
    h.onMessage([&mpc,&iters](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        
        //int iters = 5;
        
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message. The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    
                    // 0. Identify incoming data (j[1] is the data JSON object)
                    vector<double> ptsx = j[1]["ptsx"];  // Planned x path (waypoints) in map space (meters)
                    vector<double> ptsy = j[1]["ptsy"];  // Planned y path (waypoints) in map space (meters)
                    double px = j[1]["x"];               // Vehicle x position in map space (meters)
                    double py = j[1]["y"];               // Vehicle y position in map space (meters)
                    double psi = j[1]["psi"];            // Vehicle orientation in map space (radians)
                    double v = j[1]["speed"];            // Vehicle speed (mph) orientation in map space

                    // 1. Convert Planned Path line (Waypoints) from World/Map space to Vehicle space
                    vector<double> ptsx_v = {};
                    vector<double> ptsy_v = {};
                    //Eigen::VectorXd  ptsx_v;
                    //Eigen::VectorXd  ptsy_v;
                    //ptsx_v.resize(ptsx.size());
                    //ptsy_v.resize(ptsx.size());
                    ConvertMapWaypointsToVehicleCoord(ptsx,ptsy,psi,px,py,ptsx_v,ptsy_v);

                    
                    // 2. Fit a smooth polynomial to planned path (in vehicle coords now)
                    //Eigen::VectorXd  ptsx_v_plan;
                    //Eigen::VectorXd  ptsy_v_plan;
                    
                    //Eigen::VectorXd ptsx_v_plan(ptsx_v.data());  // Convert vector data to Eigen::VectorXd
                    //Eigen::VectorXd ptsy_v_plan(ptsy_v.data());
                    
                    //double * ptrx = &ptsx[0];
                    //Eigen::Map<Eigen::VectorXd> ptsx_v_plan(ptsx_v, ptsx_v.size());
                    
                    /*
                    for (int i=0; i<ptsx_v.size(); i++) {
                        ptsx_v_plan.addTo(ptsx_v[i]);
                        ptsy_v_plan.addTo(ptsy_v[i]);
                    }
                    */
                    double *ptrx = &ptsx_v[0];
                    double *ptry = &ptsy_v[0];
                    
                    Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, ptsx_v.size());
                    Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, ptsx_v.size());
                     
                    
                    auto coeffs = polyfit(ptsx_transform,ptsy_transform,3);
                    cout << "Main: coeffs=" << coeffs << endl;
                    //Eigen::VectorXd coeffs = polyfit(ptsx_v_plan,ptsy_v_plan,3);
                    
                    // 3. Prepare data for Model Prediction C solver
                    // New code
                    //Need to covert vector<double> to Eigen::VectorXdEigen::VectorXd coeffs;
                    //auto coeffs = polyfit(<#Eigen::VectorXd xvals#>, <#Eigen::VectorXd yvals#>, <#int order#>)(ptsx, ptsy, 2);
                    
                    
                    //double y2 = polyeval(coeffs,px);
                    //cout << "Main: py=" << py << " y2=" << y2 << endl;
                    //double cte = polyeval(coeffs, px) - py;
                    double cte = polyeval(coeffs, 0);
                    //double epsi = psi - atan(coeffs[1]);
                    double epsi = -atan(coeffs[1]);
                    cout << "Main: cte=" << cte << " epsi=" << epsi << endl;
                    
                    
                    Eigen::VectorXd state(6);
                    //state << px, py, psi, v, cte, epsi;
                    state << 0, 0, 0, v, cte, epsi;
                    //state << px, py, psi, v, cte, epsi;
                    
                    std::vector<double> x_vals = {state[0]};
                    std::vector<double> y_vals = {state[1]};
                    std::vector<double> psi_vals = {state[2]};
                    std::vector<double> v_vals = {state[3]};
                    std::vector<double> cte_vals = {state[4]};
                    std::vector<double> epsi_vals = {state[5]};
                    std::vector<double> delta_vals = {};
                    std::vector<double> a_vals = {};
                    
                    //TODO: Calculate steering angle and throttle using MPC. Both are in between [-1, 1].
                    double steer_value;
                    double throttle_value;


                   // for (size_t i = 0; i < iters; i++) {
                        //std::cout << "Main: Iteration " << i << std::endl;
                        
                        
                        auto vars = mpc.Solve(state, coeffs);
                    
                    /*
                        x_vals.push_back(vars[0]);
                        y_vals.push_back(vars[1]);
                        psi_vals.push_back(vars[2]);
                        v_vals.push_back(vars[3]);
                        cte_vals.push_back(vars[4]);
                        epsi_vals.push_back(vars[5]);
                        
                        delta_vals.push_back(vars[6]);
                        a_vals.push_back(vars[7]);
                        
                        state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
                         state << 0, 0, 0, vars[3], vars[4], vars[5];
                        std::cout << "x = " << vars[0] << std::endl;
                        std::cout << "y = " << vars[1] << std::endl;
                        std::cout << "psi = " << vars[2] << std::endl;
                        std::cout << "v = " << vars[3] << std::endl;
                        std::cout << "cte = " << vars[4] << std::endl;
                        std::cout << "epsi = " << vars[5] << std::endl;
                        std::cout << "delta = " << vars[6] << std::endl;
                        std::cout << "a = " << vars[7] << std::endl;
                        std::cout << std::endl;
                        
                        //steer_value = vars[2];    // Update estimate for steer angle
                        //throttle_value = vars[7];  // Udate estimate for throttle
                        
                     */
                        steer_value = -vars[0]/(deg2rad(25));  // Multiply by -1 because turn in reverse in simulator
                        throttle_value = vars[1];
                        
                        cout << "MPC: Sol steer=" << steer_value << " throttle=" << throttle_value << endl;
                        
                    //}

                    //steer_value = -vars[0]/(deg2rad(25));
                    //throttle_value = vars[1];

                    
                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    //steer_value = 0.0;
                    msgJson["steering_angle"] = steer_value;
                    throttle_value = .1;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    /*
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    */
                    msgJson["next_x"] = ptsx_v;
                    msgJson["next_y"] = ptsy_v;

                    // Message to Simulator Server
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                   // std::cout << msg << std::endl;
          
                    // Latency
                    // The purpose is to mimic real driving conditions where the car does actuate the commands instantly.
                    // Feel free to play around with this value but should be to drive around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                } // if telemetry
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } // if s
        } // if sdata
    }); // .onMessage

    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    //
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected & ready to run vehicles w/ MPC controller!!!" << std::endl;
    });

    //
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    
    //
    // Start of Main method!
    //
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Starting. Listening to port=" << port << " ..." << std::endl;
    } else {
        std::cerr << "Failed to listen to port. Existing..." << std::endl;
        return -1;
    }
  
    h.run();  // Entry point to running webSocket communication
    
} // Main
