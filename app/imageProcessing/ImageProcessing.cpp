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

