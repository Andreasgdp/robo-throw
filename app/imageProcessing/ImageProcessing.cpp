#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::loadImage() {
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam");

    // this will contain the image from the webcam
    cv::Mat frame;

    // capture the next frame from the webcam
    camera >> frame;

    // display the frame until you press a key
    while (1) {
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }
}

cv::Mat ImageProcessing::loadLocalImage(std::string imageFileName, std::string imageFileType) {
    // Read the image file given the file path
    // TODO: Change cv::IMREAD_COLOR if needed for the project
    cv::Mat image = cv::imread("../app/imageProcessing/images/" + imageFileName + "." + imageFileType, cv::IMREAD_COLOR);

    // Error Handling
    if (image.empty()) throw "Image File Not Found";
    return image;
}

void ImageProcessing::showImage(cv::Mat image, std::string windowName) {
    // Show Image inside a window with the window name provided
    cv::imshow("Image show", image);

    // Wait for any keystroke
    cv::waitKey(0);
}


#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::loadImage() {
    // open the first webcam plugged in the computer
    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam");

    // this will contain the image from the webcam
    cv::Mat frame;

    // capture the next frame from the webcam
    camera >> frame;

    // display the frame until you press a key
    while (1) {
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }
}

std::vector<cv::Mat> ImageProcessing::loadImagePC(){
    //cv::namedWindow("Output",1);
    std::vector<cv::Mat> pics;
            cv::Mat pic = cv::imread("/home/robot/Desktop/RoboTrow/robo-throw/app/imageProcessing/table.jpg",-1);
    pics.push_back(pic);
    //cv::imshow("output",pic);
    //cv::waitKey(0);
    return pics;
}

void ImageProcessing::getBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> > &foundCorners){

    for(std::vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++){
        //goes through vector of images (pointers to)
        std::vector<cv::Point2f> pointBuf;
        //vector storing checkerboard corners
        bool found = cv::findChessboardCorners(*iter,cv::Size(17,24),pointBuf,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE );
        //finds board corners using inbuild opencv function
        //Size(17,24) refers to the number of interctions between squares on the checkerboard, vertical/horizontal
        //CALIB_CB_ADAPTIVE_THRESH - converts image to black and white
        //CALIB_CB_NORMALIZE_IMAGE Not sure what it dose - opencv themselves says "Normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding."
        if(found){
            //if any corners are found, this will save and show them
            foundCorners.push_back(pointBuf);
            cv::drawChessboardCorners(*iter,cv::Size(17,24),pointBuf,found);
            cv::imshow("looking for corners",*iter);
            cv::waitKey(0);
        }
    }
}

void ImageProcessing::calibrate(cv::Mat frame)
{
    const float squareEdgeLength = 0.02; //meters - Stores the side lengths from calibration checker board - 0.02 = placeholder
    const cv::Size checkerboardDimentions = cv::Size(17,24);
    //17x24




    cv::imshow("test",frame);
    cv::waitKey(0);
}

void ImageProcessing::run(){
    //tmp function to run shit
    std::vector<std::vector<cv::Point2f>> corners, rejektedCorners;
    ImageProcessing imp;

    imp.getBoardCorners(imp.loadImagePC(),corners);
}


