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
    
    
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    std::ofstream outputFile2;
    outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt");    

    firstPath->MoveCurrentState(300, point);
    outputFile2 << "300 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(200, point);
    outputFile2 << "200 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(-100, point);
    outputFile2 << "-100 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(-200, point);
    outputFile2 << "-200 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(-27, point);
    outputFile2 << "-27 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(132, point);
    outputFile2 << "132 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    firstPath->MoveCurrentState(-2000, point);
    outputFile2 << "-2000 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    outputFile2.close();
    
    return 0;
}