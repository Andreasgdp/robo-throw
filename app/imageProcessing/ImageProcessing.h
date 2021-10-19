#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H
#include "opencv2/opencv.hpp"
#include "iostream"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "vector"
#include <pylon/PylonIncludes.h>


class ImageProcessing
{
public:
    ImageProcessing();
    void calibrate();
    std::vector<cv::Mat> pylonPic();
    cv::Point ballDetection(cv::Mat src);
    void getCornersV2(std::vector<cv::Mat> imgVec);
    std::vector<cv::Mat> loadLoaclimg();

    cv::Mat cropImg(cv::Mat img);
    void cordConvert(cv::Point imgPos) ;


private:
    int _sim = false;
    const cv::Size BoardSize{6,9};
    int _camHeight = 139, _camWith =80; //cm

    bool autoImg = true; //

    std::vector<std::vector<cv::Point3f>> Q; //Checkerboard voordinates
    int imgAmt = 10; // Ammount of images to take
    bool showimg = false; //show images at every step
    std::vector<cv::Mat> _calibrationMat; //vector of calibration Mat's

};

#endif // IMAGEPROCESSING_H
