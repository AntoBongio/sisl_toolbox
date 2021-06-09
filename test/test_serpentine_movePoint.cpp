#include "test/test_serpentine.hpp"


int main(int argc, char** argv) {   
    
    // ctb::LatLong centroid{44.39173292288923, 8.945241571195552};

    // std::vector<ctb::LatLong> pointLatLong {
    //     ctb::LatLong(44.39103097698746, 8.94579379703305) , 
    //     ctb::LatLong(44.39130994954367, 8.946484085232179) ,
    //     ctb::LatLong(44.3921468857443, 8.946245637251524) , 
    //     ctb::LatLong(44.39244386538219, 8.94470188296125) ,
    //     ctb::LatLong(44.3915259324859, 8.943999052589279) ,
    //     ctb::LatLong(44.39074299756334, 8.94435048387282)};


    // std::vector<Eigen::Vector3d> polygonVerteces(pointLatLong.size(), Eigen::Vector3d::Zero());
    // for(auto i = 0; i < polygonVerteces.size(); i++) {
    //     ctb::LatLong2LocalNED(pointLatLong[i], 0.0, centroid, polygonVerteces[i]);
    //     polygonVerteces[i][2] = 0;
    // }

    // double angle{45.0}; 
    // double offset{10.0};

    // auto serpentine = PathFactory::NewSerpentine(angle, offset, polygonVerteces);

    // PersistenceManager::SaveObj(serpentine->Sampling(350), "/home/antonino/Desktop/sisl_toolbox/script/serpentine.txt");
    
    // std::cout << "Serpentine length: " << serpentine->Length() << std::endl;

    auto curve = CurveFactory::NewCurve<Circle>(6.28, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{10, 10, 0}, Eigen::Vector3d{2, 2, 1});

    PersistenceManager::SaveObj(curve->Sampling(150), "/home/antonino/Desktop/sisl_toolbox/script/path.txt");

    double startValueAbscissa_m{5};
    double offset_m{8};
    Eigen::Vector3d movedPoint{};
    double movedAbscissa_m{0};
    overBound overBound;

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(startValueAbscissa_m, offset_m);
    
    std::ofstream outputFile2;
    outputFile2.open ("/home/antonino/Desktop/sisl_toolbox/script/movePoint.txt");    
    outputFile2 << offset_m << " " << movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    // curve->FromAbsMetersToPos(movedAbscissa_m + offset_m, movedPoint);
    outputFile2 << offset_m << " " << movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    outputFile2 << offset_m << " " <<  movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    outputFile2 << offset_m << " " <<  movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    outputFile2 << offset_m << " " <<  movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    outputFile2 << offset_m << " " <<  movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    std::tie(movedPoint, movedAbscissa_m, overBound) = curve->MovePoint(movedAbscissa_m, offset_m);
    outputFile2 << offset_m << " " <<  movedPoint[0] << " " << movedPoint[1] << " " << movedPoint[2] << "\n";

    outputFile2.close();

    double beyondLower;
    double beyondUpper;
    auto curveSection = curve->ExtractCurveSection(20, 150, beyondLower, beyondUpper);
    
    PersistenceManager::SaveObj(curveSection->Sampling(150), "/home/antonino/Desktop/sisl_toolbox/script/pathSection.txt");
    
    return 0;
}