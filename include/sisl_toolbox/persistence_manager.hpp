#pragma once

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Dense>

/**
 * @class PersistenceManager
 *
 * @brief Abstract class used to save a Curve or a Path. 
 */
class PersistenceManager {

    public:

        /**
         * @brief Save the points passed in a file. Throw an exception if it fails to open the file.
         * 
         * @param[in] points Shared ptr of vector of Eigen::Vector3d containing the points to save.
         * @param[in] path Path where to save the object.
         * 
         * @return Success 
         */
        static bool SaveObj(std::shared_ptr<std::vector<Eigen::Vector3d>> points, std::string const& path);

    private:
};