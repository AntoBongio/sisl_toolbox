#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

#define GENERICCURVE 0
#define STRAIGHTLINE 1
#define CIRCLE 2

#define RIGHT true
#define LEFT false

struct overBound {

    bool upperFlag = false;
    double upperBound = 0;
    bool lowerFlag = false;
    double lowerBound = 0;

    void setUpper(double abscissa_m, double endParameter_m_) {
        upperFlag = true;
        upperBound = abscissa_m - endParameter_m_;
    };

    void setLower(double abscissa_m, double startParameter_m_) {
        lowerFlag = true;
        lowerBound = startParameter_m_ - abscissa_m;
    };
};
