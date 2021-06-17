#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <ctrl_toolbox/HelperFunctions.h>

#include <jsoncpp/json/json.h>
#include <chrono>
#include <iomanip>

#include "sisl_toolbox/curve_factory.hpp"
#include "sisl_toolbox/path_factory.hpp"
#include "sisl_toolbox/curve.hpp"
#include "sisl_toolbox/persistence_manager.hpp"
#include "sisl_toolbox/straight_line.hpp"
#include "sisl_toolbox/circle.hpp"
#include "sisl_toolbox/path.hpp"
#include "sisl_toolbox/defines.hpp"