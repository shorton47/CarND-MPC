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
