#include "test/test_serpentine.hpp"


int main(int argc, char** argv) {   
    
    std::vector<Parameters> curveDefinition;

    std::string pathJson = "/home/antonino/Desktop/sisl_toolbox/test/path.json";

    Json::Value root, curveRoot;
    Json::Reader reader;

    std::ifstream file(pathJson);
    file >> root;
    reader.parse(file, root, true);
        
    ctb::LatLong centroid{root["centroid"][0].asDouble(), 
                          root["centroid"][1].asDouble()};
    bool reverse = root["direction"].asInt() ? true : false;     
    
    for(auto curve: root["curves"]) {

        reader.parse(curve.toStyledString(), curveRoot); // provare senza toStyledString

        int degree {curveRoot["degree"].asInt()}; 

        std::vector<double> weights;
        for(auto i = 0; i < curveRoot["weigths"].size(); i++) {
            weights.push_back(curveRoot["weigths"][i].asDouble());
        }

        std::vector<Eigen::Vector3d> points(curveRoot["points"].size(), Eigen::Vector3d::Zero());
        for(auto i = 0; i < curveRoot["points"].size(); i++) {
            ctb::LatLong pointLatLong;
            pointLatLong.latitude = curveRoot["points"][i][0].asDouble();
            pointLatLong.longitude = curveRoot["points"][i][1].asDouble();
            ctb::LatLong2LocalUTM(pointLatLong, 0.0, centroid, points[i]);
        }

        std::vector<double> knots;
        for(auto i = 0; i < curveRoot["knots"].size(); i++) {
            knots.push_back(curveRoot["knots"][i].asDouble());
        }

        curveDefinition.emplace_back(degree, knots, points, weights);
    }

    auto firstPath = std::make_shared<Path>(curveDefinition);
    
    std::cout << "Length: " << std::fixed << std::setprecision(3) << firstPath->Length() << std::endl; 
    firstPath->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/path.txt");
    
    std::shared_ptr<Path> pathSection;

    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    firstPath->MoveCurrentState(800.0, point);
    std::cout << "Requested extraction of 300.0m offset --> final pathSection length: 600.0 m" << std::endl;
    std::cout << std::endl;

    firstPath->ExtractSection(600.0, pathSection);
    std::cout << std::endl << "Path Section length: " << std::fixed << std::setprecision(3) << pathSection->Length()  << " m" << std::endl; 

    pathSection->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/pathSection.txt");
    
    return 0;
}