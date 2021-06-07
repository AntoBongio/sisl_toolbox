#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

#define GENERICCURVE 0
#define STRAIGHTLINE 1
#define CIRCLE 2

struct Parameters {

   // Generic Curve parameters constructor (data needed to build a curve using the newCurve() routine)
   Parameters(int type, int degree, std::vector<double> knots, std::vector<Eigen::Vector3d> points, std::vector<double> weights, 
      std::vector<double> coefficients = {}) :
      type{type}, dimension{3}, order{3}, degree{degree}, knots{knots}, points{points}, weights{weights}, coefficients{coefficients} {}   

   // StraightLine parameters constructor
   Parameters(int type, Eigen::Vector3d startPoint, Eigen::Vector3d endPoint, int dimension = 3, int order = 3) :
      type{type}, startPoint{startPoint}, endPoint{endPoint} , dimension{dimension}, order{order} {}

   // Circle parameters constructor
   Parameters(int type, double angle, Eigen::Vector3d axis, Eigen::Vector3d startPoint, Eigen::Vector3d centrePoint, 
      int dimension = 3, int order = 3) :
      type{type}, angle{angle}, axis{axis}, startPoint{startPoint}, centrePoint{centrePoint}, dimension{dimension}, order{order}  {}

   int type; // 0 -> Generic Curve generated through newCurve(), 1 -> StraightLine, 2 -> Circle
   int dimension; // GenericCurve, StraightLine, Circle
   int order; // GenericCurve, StraightLine, Circle

   Eigen::Vector3d startPoint; // StraightLine, Circle
   Eigen::Vector3d endPoint; // StraightLine
   double angle; // Circle
   Eigen::Vector3d axis; // Circle
   Eigen::Vector3d centrePoint; // Circle

   
   int degree; // GenericCurve
   std::vector<double> knots; // GenericCurve
   std::vector<Eigen::Vector3d> points; // GenericCurve
   std::vector<double> weights; // GenericCurve
   std::vector<double> coefficients; // GenericCurve

};