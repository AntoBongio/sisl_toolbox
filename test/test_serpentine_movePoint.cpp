#include "test/test_serpentine.hpp"


int main(int argc, char** argv) {   
    
    ctb::LatLong centroid{44.39173292288923, 8.945241571195552};

    std::vector<ctb::LatLong> pointLatLong {
        ctb::LatLong(44.39103097698746, 8.94579379703305) , 
        ctb::LatLong(44.39130994954367, 8.946484085232179) ,
        ctb::LatLong(44.3921468857443, 8.946245637251524) , 
        ctb::LatLong(44.39244386538219, 8.94470188296125) ,
        ctb::LatLong(44.3915259324859, 8.943999052589279) ,
        ctb::LatLong(44.39074299756334, 8.94435048387282)};


    std::vector<Eigen::Vector3d> polygonVerteces(pointLatLong.size(), Eigen::Vector3d::Zero());
    for(auto i = 0; i < polygonVerteces.size(); i++) {
        ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygonVerteces[i]);
        polygonVerteces[i][2] = 0;
    }

    double angle{45.0}; 
    double offset{10.0};

    auto serpentine = PathFactory::NewSerpentine(angle, offset, polygonVerteces);

    PersistenceManager::SaveObj(serpentine->Sampling(350), "/home/antonino/Desktop/sisl_toolbox/script/serpentine.txt");
    
    std::cout << "Serpentine length: " << serpentine->Length() << std::endl;


    //auto curve = CurveFactory::NewCurve<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{10, 10, 0}, Eigen::Vector3d{7, 7, 1});
    
    // Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    // std::ofstream outputFile2;
    // outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt");    

    // firstPath->MoveCurrentState(300, point);
    // outputFile2 << "300 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(200, point);
    // outputFile2 << "200 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(-100, point);
    // outputFile2 << "-100 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(-200, point);
    // outputFile2 << "-200 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(-27, point);
    // outputFile2 << "-27 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(132, point);
    // outputFile2 << "132 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // firstPath->MoveCurrentState(-2000, point);
    // outputFile2 << "-2000 " << point[0] << " " << point[1] << " " << point[2] << "\n";

    // outputFile2.close();
    
    return 0;
}