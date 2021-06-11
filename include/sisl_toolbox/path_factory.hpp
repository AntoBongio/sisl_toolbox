#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>



#include <sisl_toolbox/defines.hpp>


/** MOMENTANEO: */ 
#include "sisl_toolbox/persistence_manager.hpp"

class Path;

class PathFactory
{
public:

    static std::shared_ptr<Path> NewHippodrome(std::vector<Eigen::Vector3d> points);
    static std::shared_ptr<Path> NewPolygon(std::vector<Eigen::Vector3d> points);
    static std::shared_ptr<Path> NewSpiral(Eigen::Vector3d centrePoint, Eigen::Vector3d startPoint, double radiusOffset);
    static std::shared_ptr<Path> NewSerpentine(double angle, double offset, std::vector<Eigen::Vector3d>& polygonVerteces);
    

private: 
    /** 
     * @brief Convert an angle in degrees to [0, 360.0) interval
     * 
     * @param[in] angle The angle to be converted passed by copy
     * 
     * @return The along curve distance in meters from the stating point of the curve to abscissa parameter.
     */ 
    static auto convertToAngleInterval (double angle) {
        while(angle < 0) { angle += 360.0; }
        return std::fmod(angle, 360.0);
    };

    /** 
     * @brief Given the vertices of a polygon, compute the rectangle surrounding the figure (with 3 decimals precision).
     * 
     * @param[in] polygonVerteces const reference to the vector containing polygon's vertices
     * 
     * @return A tuple with: (maxX, minX, maxY, minY)
     */ 
    //static std::tuple<double, double, double, double> evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces);

    static std::tuple<double, double, double, double> evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces) {
        double maxX{polygonVerteces[0][0]}; 
        double minX{polygonVerteces[0][0]};
        double maxY{polygonVerteces[0][1]};
        double minY{polygonVerteces[0][1]};
        for(auto i = 1; i < polygonVerteces.size(); i++) {
            if(maxX < polygonVerteces[i][0])
                maxX = polygonVerteces[i][0];
            if(minX > polygonVerteces[i][0])
                minX = polygonVerteces[i][0];   

            if(maxY < polygonVerteces[i][1])
                maxY = polygonVerteces[i][1];
            if(minY > polygonVerteces[i][1])
                minY = polygonVerteces[i][1];  
        }
        // Set the vertices precision
        return std::make_tuple(std::round(maxX * 1000) / 1000, std::round(minX * 1000) / 1000, 
                               std::round(maxY * 1000) / 1000, std::round(minY * 1000) / 1000);
    };

    /** 
     * @brief Compute distance between two points
     * 
     * @param[in] vec1 First point expresed as Eigen::Vector3d
     * @param[in] vec2 Second point expresed as Eigen::Vector3d
     * 
     * @return Distance between two points
     */ 
    static auto Distance (Eigen::Vector3d const& vec1, Eigen::Vector3d const& vec2) {
        return std::sqrt(std::pow(vec1[0] - vec2[0], 2) + std::pow(vec1[1] - vec2[1], 2) + std::pow(vec1[2] - vec2[2], 2));
    };

     /** 
     * @brief Transform an angle in degrees in radians
     * 
     * @param[in] angle degrees
     * 
     * @return Angle in radians
     */ 
    static auto FromDegToRad (double angle) {
        return angle * M_PI / 180.0;
    };


};

