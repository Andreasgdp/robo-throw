#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H
#include "opencv2/opencv.hpp"
#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "vector"
#include <pylon/PylonIncludes.h>
#include <algorithm>

class ImageProcessing
{
public:
    ImageProcessing();
    void calibrate();
    std::vector<double> getBallCoords();
    std::vector<cv::Mat> pylonPic(int imgAmt);
    cv::Point ballDetection(cv::Mat src);
    void chessboardDetection(std::vector<cv::Mat> imgVec);
    std::vector<cv::Mat> loadLocalImg();
    std::vector<cv::Point> cornersTempleMatching(cv::Mat ref);
    void cornersHoughCircles(cv::Mat src);
    cv::Mat cropImg(cv::Mat img);
    std::vector<double> coordConvert(cv::Point imgPos, cv::Mat img) ;
    cv::Mat rotateImg(cv::Mat img);


private:
    std::vector<cv::Point> cropCornerPoints;
    std::vector<std::vector<cv::Point3f>> Q;    //Checkerboard voordinates
    std::vector<cv::Mat> _calibrationMat;       //vector of calibration Mat's

};

#endif // IMAGEPROCESSING_H
