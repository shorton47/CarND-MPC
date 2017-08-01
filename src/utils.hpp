//
//  utils.hpp
//  MPCProject
//
//  Created by Steve Horton on 7/30/17.
//
//

#ifndef utils_hpp
#define utils_hpp


#endif /* utils_hpp */

#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;



double deg2rad(double x);

double rad2deg(double x);

void ConvertMapWaypointsToVehicleCoord(vector<double> &ptsx, vector<double> &ptsy, const double &theta_wrld,
                                       const double &x_in, const double &y_in, vector<double> &ptsx_v, vector<double> &ptsy_v);
