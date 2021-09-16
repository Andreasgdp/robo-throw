#include "App.h"

App::App(std::string IP) : roboConn(IP)
{
    this->IP = IP;
}

void App::initializeApp(const cv::Mat &calibrationImg)
{
    if (!this->roboConn.isConnected()) throw "Connection could not be established with ip: " + this->IP;

    if (calibrationImg.cols == 0 && calibrationImg.rows == 0) {
        // Use imageProcessing to get image from camera
    }

    // use imageProcessing to calibrate camera
}
