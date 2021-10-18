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
    std::vector<cv::Mat> pylonPic(std::vector<cv::Mat> calibration);
    std::vector<cv::Mat> getCornersV2(std::vector<cv::Mat> imgVec);
    std::vector<cv::Mat> loadLoaclimg();


private:
    int _sim = false;
    const cv::Size BoardSize{6,9};
    bool isCalib = false;
    bool preCalib = false;
    std::vector<std::vector<cv::Point3f>> Q; //Checkerboard voordinates
    int imgAmt = 10; // Ammount of images to take
    bool showimg = false; //show images at every step
};

#endif // IMAGEPROCESSING_H
