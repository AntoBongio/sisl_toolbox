#include "sisl_toolbox/persistence_manager.hpp"


bool PersistenceManager::SaveObj(std::shared_ptr<std::vector<Eigen::Vector3d>> points, std::string const& path) {

    try {
        auto file = std::make_shared<std::ofstream>(path.c_str(), std::ofstream::out);

        if (!(*file))
            throw std::runtime_error("Unable to open Output file: " + path);

        for(int i = 0; i < points->size(); ++i) {
            *file << (*points)[i][0] << " " << (*points)[i][1] << " " << (*points)[i][2] << "\n";
        }

        file->close();

    } catch (std::exception& e) {
        std::cerr << "Exception thrown: " << e.what() << std::endl;
        return -1;
    }

}
