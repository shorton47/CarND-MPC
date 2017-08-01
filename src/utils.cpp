//
//  utils.cpp
//  MPCProject
//
//  Created by Steve Horton on 7/30/17.
//
//

#include <math.h>
//#include <iostream>
//#include <vector>


#include "utils.hpp"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180.0; }

double rad2deg(double x) { return x * 180.0 / pi(); }

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

