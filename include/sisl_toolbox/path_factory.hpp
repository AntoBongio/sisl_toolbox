#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>


#define RIGHT 1
#define LEFT 2

class Path;

/**
 * @class PathFactory (abstract)
 *
 * @brief Factory implementing the logic to automatically build different paths: [Polygonal Chain, Polygon, 
 *        Hippodrome, Spiral, Race Track, Spiral]. Each Method returns a shared_ptr<Path> pointing to an object
 *        representing the specific object requested.
 */
class PathFactory
{
public:

    /**
     * @brief Generate a PolygonalChain. It needs at least 2 points, otherwise it throws an exception.
     * 
     * @param[in] points The points chracterizing the polygonal chain.
     * 
     * @return A shared_ptr pointing to a Polygonal Chain described as a Path object.
     */
    static std::shared_ptr<Path> NewPolygonalChain(std::vector<Eigen::Vector3d> points);

    /**
     * @brief Generate a Polygon. It needs at least 3 points, otherwise it throws an exception.
     * 
     * @param[in] points The points chracterizing the polygon.
     * 
     * @return A shared_ptr pointing to a Polygon described as a Path object.
     */ 
    static std::shared_ptr<Path> NewPolygon(std::vector<Eigen::Vector3d> points);

    /**
     * @brief Generate an Hippodrome starting from 4 points. If the points are not 4, an exception is thrown.
     * 
     * @param[in] points The points chracterizing the hippodrome.
     * 
     * @return A shared_ptr pointing to an Hippodrome described as a Path object.
     */ 
    static std::shared_ptr<Path> NewHippodrome(std::vector<Eigen::Vector3d> points);

    /**
     * @brief Generate a Spiral made starting from circular Arc of angle 3.14 or -3.14 and radius 
     *        progressively smaller than a quantity equal to radiusOffset.
     * 
     * @param[in] centrePoint The centre of the spiral.
     * @param[in] startPoint The start point of the spiral.
     * @param[in] radiusOffset The radius difference between each successive circular arc.
     * 
     * @return A shared_ptr pointing to a Spiral described as a Path object.
     */
    static std::shared_ptr<Path> NewSpiral(Eigen::Vector3d centrePoint, Eigen::Vector3d startPoint, double radiusOffset);
    
    /**
     * @brief Generate a Race Track. It is composed of straight lines with an angle w.r.t the x-axis specified by "angle", 
     *        circular arcs with alternating radius equal to firstRadius and secondRadius, with the turning direction always the same
     *        and defined by the direction parameter (RIGHT/LEFT).
     * 
     * @param[in] angle Angle of the path w.r.t. the x-axis.
     * @param[in] direction The turning direction.
     * @param[in] firstRadius Radius of the first circular arc.
     * @param[in] secondRadius Radius of the second circular arc.
     * @param[in] polygonVerteces The polygon defining the area that must be filled with the Race Track.
     * 
     * @return A shared_ptr pointing to a Race Track described as a Path object.
     */    
    static std::shared_ptr<Path> NewRaceTrack(double angle, int direction, double firstRadius, double secondRadius, 
                                            std::vector<Eigen::Vector3d>& polygonVerteces);

    /**
     * @brief Generate a Serpentine. It is composed of straight lines with an angle w.r.t the x-axis specified by "angle", 
     *        circular arc with radius equal to offset/2 and with the turning direction starting from the one defined by 
     *        the direction parameter (RIGHT/LEFT) and and alternates thereafter.
     * 
     * @param[in] angle Angle of the path w.r.t. the x-axis.
     * @param[in] direction The turning direction.
     * @param[in] offset Distance among the straight lines, also defining the circular arc diameter.
     * @param[in] polygonVerteces The polygon defining the area that must be filled with the Race Track.
     * 
     * @return A shared_ptr pointing to a Serpentine described as a Path object.
     */   
    static std::shared_ptr<Path> NewSerpentine(double angle, int direction, double offset, 
                                                std::vector<Eigen::Vector3d>& polygonVerteces);

private: 
    /** 
     * @brief Convert an angle in degrees to [0, 360.0) interval
     * 
     * @param[in] angle The angle to be converted passed by copy
     * 
     * @return The along curve distance in meters from the stating point of the curve to abscissa parameter.
     */ 
    static auto ConvertToAngleInterval (double angle) {
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
    static std::tuple<double, double, double, double> evalRectangleBoundingBox (std::vector<Eigen::Vector3d> const& polygonVerteces) {
        double maxX{polygonVerteces[0][0]}; 
        double minX{polygonVerteces[0][0]};
        double maxY{polygonVerteces[0][1]};
        double minY{polygonVerteces[0][1]};
        for(std::size_t i = 1; i < polygonVerteces.size(); i++) {
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
    static auto DegToRad (double angle) {
        return angle * M_PI / 180.0;
    };

    static auto RadToDeg (double angle) {
        return angle * 180.0 / M_PI;
    };

};

