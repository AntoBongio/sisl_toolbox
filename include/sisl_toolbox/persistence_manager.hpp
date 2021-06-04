#pragma once

#include <iostream>
#include <memory>
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Dense>

class PersistenceManager {

    public:

        static bool SaveObj(std::shared_ptr<std::vector<Eigen::Vector3d>> points, std::string const& path);

    private:
};