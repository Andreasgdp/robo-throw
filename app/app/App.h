#ifndef APP_H
#define APP_H
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "../imageProcessing/ImageProcessing.h"
#include "../robotConnection/RobotConnection.h"

class App
{
public:
    App(std::string IP);
    void initializeApp(const cv::Mat &calibrationImg = cv::Mat());
    void findAndGrabObject(const cv::Mat &objectImg = cv::Mat());


private:
    std::string IP;
    std::vector<double> jointPoses;
    ImageProcessing imgProcessor;
    RobotConnection roboConn;
    // add member variable for jointPoseGetter
    // add member variable for gripperHandling
    // add member variable for api
    // add member variable for image
    // add member variable for processed image
    // add member variable for object posision
    // add member variable for target pos

};

#endif // APP_H
