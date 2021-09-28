#include "test/test_path.hpp"
#include "sisl_toolbox/generic_curve.hpp"
#include <vector>

#include <iomanip>


int main() {

    /***************** Path creation *****************/

    
    // unsync the I/O of C and C++.
    std::ios_base::sync_with_stdio(false);

    double x_start = 0.0;
    double y_start = 0.0;
    double heading_start = 0.0;
    heading_start = heading_start * M_PI / 180.0;

    double look_ahead = 0.5;

    double x_final = 4.0;
    double y_final = 3.0;
    double heading_final = 170;
    heading_final = heading_final * M_PI / 180.0;

    std::vector<Eigen::Vector3d> points;
    points.push_back(Eigen::Vector3d{x_start, y_start, 0.0});
    points.push_back(Eigen::Vector3d{x_start + look_ahead * cos(heading_start), y_start + look_ahead * sin(heading_start), 0.0});
    points.push_back(Eigen::Vector3d{x_start + 2 * look_ahead * cos(heading_start), y_start + look_ahead * 2 * sin(heading_start), 0.0});

    points.push_back(Eigen::Vector3d{x_final + 2 * look_ahead * cos(heading_final), y_final + look_ahead * 2 * sin(heading_final), 0.0});
    points.push_back(Eigen::Vector3d{x_final + look_ahead * cos(heading_final), y_final + look_ahead * sin(heading_final), 0.0});
    points.push_back(Eigen::Vector3d{x_final, y_final, 0.0});

    std::vector<double> weights {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    std::vector<double> knots {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0};

    std::vector<double> coefficients {};

    try {
        auto genericCurve = std::make_shared<GenericCurve>(3, knots, points, weights, coefficients);
        auto path = std::make_shared<Path>();
        path->AddCurveBack(genericCurve); 
        std::cout << *(path) << std::endl;

        PersistenceManager::SaveObj(path->Sampling(20), "/home/marco/pasqua_ros2_devel/src/Virtual_Frame_Controller/sisl_toolbox/script/path.txt");

    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }

    return 0;
}