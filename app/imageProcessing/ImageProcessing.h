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
    void cornersHoughCircles(cv::Mat src);
    void chessboardDetection(std::vector<cv::Mat> imgVec);
    cv::Mat   cropImg(cv::Mat img);
    cv::Mat   rotateImg(cv::Mat img);
    cv::Point ballDetection(cv::Mat src);
    std::vector<double>    getBallCoords();
    std::vector<double>    coordConvert(cv::Point imgPos, cv::Mat img) ;
    std::vector<cv::Mat>   grabImage(int imgAmt);
    std::vector<cv::Mat>   loadCalibImages();
    std::vector<cv::Point> cornersTempleMatching(cv::Mat ref);

private:
    std::vector<cv::Point> _cropCornerPoints;   // Coordinates for table corners
    std::vector<std::vector<cv::Point3f>> Q;    //Checkerboard voordinates
};

#endif // IMAGEPROCESSING_H
