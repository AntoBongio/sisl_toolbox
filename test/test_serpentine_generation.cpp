#include "test/test_serpentine.hpp"
#include <unordered_set>


int main(int argc, char** argv) {   

    ctb::LatLong centroid{44.39173292288923, 8.945241571195552};

    std::vector<ctb::LatLong> pointLatLong {
        ctb::LatLong(44.39103097698746, 8.94579379703305) , 
        ctb::LatLong(44.39130994954367, 8.946484085232179) ,
        ctb::LatLong(44.3921468857443, 8.946245637251524) , 
        ctb::LatLong(44.39244386538219, 8.94470188296125) ,
        ctb::LatLong(44.3915259324859, 8.943999052589279) ,
        ctb::LatLong(44.39074299756334, 8.94435048387282)};


    /*
    std::vector<Eigen::Vector3d> polygon(pointLatLong.size(), Eigen::Vector3d::Zero());
    for(auto i = 0; i < polygon.size(); i++) {
        ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygon[i]);
        polygon[i][2] = 0;
    }

    std::ofstream outputFile;
    outputFile.open ("/home/antonino/Desktop/sisl_toolbox/script/polygon.txt");    
    for(const auto & vertex: polygon) {
        outputFile << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
    }
    outputFile.close();
    */

    

    std::vector<Eigen::Vector3d> polygonVerteces(pointLatLong.size(), Eigen::Vector3d::Zero());
    for(auto i = 0; i < polygonVerteces.size(); i++) {
        ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygonVerteces[i]);
        polygonVerteces[i][2] = 0;
    }

    // Test unordered set

    double angle{270.0}; // Fare test negli angoli limite
    double offset{30.0};
    
    auto serpentine = std::make_shared<Path>(angle, offset, polygonVerteces);
    
    
    //std::cout << "Length: " << std::fixed << std::setprecision(3) << serpentine->Length() << std::endl; 
    //serpentine->SavePath(120, "/home/antonino/Desktop/sisl_toolbox/script/serpentine.txt");


    return 0;
}