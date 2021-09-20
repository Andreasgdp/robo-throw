#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./imageProcessing/ImageProcessing.h"

using namespace std;

int main()
{

    ImageProcessing imageprocessing;
    imageprocessing.run();
    //imageprocessing.loadImagePC();
    //imageprocessing.calibrate(imageprocessing.loadImagePC());
//    RobotConnection conn("127.0.0.1");
//    std::vector<double> test = conn.getActualJointPoses();

//    for (size_t i = 0; i < test.size(); i++) {
//        std::cout << test.at(i);
    //}
    return 0;
}
