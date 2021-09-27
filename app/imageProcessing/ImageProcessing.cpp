#include "ImageProcessing.h"


ImageProcessing::ImageProcessing(){}

const cv::Size BoardSize(6,9);

cv::Mat ImageProcessing::loadImage() {
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
//    while (1) {
//        // show the image on the window
//        cv::imshow("Webcam", frame);
//        // wait (10ms) for a key to be pressed
//        if (cv::waitKey(10) >= 0)
//            break;
    //}
    return frame;
}

//cv::Mat ImageProcessing::loadLocalImage(std::string imageFileName, std::string imageFileType) {
//    // Read the image file given the file path
//    // TODO: Change cv::IMREAD_COLOR if needed for the project
//    cv::Mat image = cv::imread("../app/imageProcessing/images/" + imageFileName + "." + imageFileType, cv::IMREAD_COLOR);

//    // Error Handling
//    if (image.empty()) throw "Image File Not Found";
//    return image;
//}

void ImageProcessing::showImage(cv::Mat image, std::string windowName) {
    // Show Image inside a window with the window name provided
    cv::imshow("Image show", image);

    // Wait for any keystroke
    cv::waitKey(0);
}

cv::Mat ImageProcessing::loadImagePC(std::string number){
    //cv::namedWindow("Output",1);
    //std::vector<cv::Mat> pics;
            cv::Mat pic = cv::imread("../app/imageProcessing/images/image-00"+ number +".jpg",-1);
    //pics.push_back(pic);
    //cv::imshow("output",pic);˝
    //cv::waitKey(0);
    return pic;
}

void ImageProcessing::getBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f> > &foundCorners){

    for(std::vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++){
        //goes through vector of images (pointers to)
        std::vector<cv::Point2f> pointBuf;
        //vector storing checkerboard corners
        bool found = cv::findChessboardCorners(*iter,BoardSize,pointBuf,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE );
        //finds board corners using inbuild opencv function
        //Size(17,24) refers to the number of interctions between squares on the checkerboard, vertical/horizontal
        //CALIB_CB_ADAPTIVE_THRESH - converts image to black and white
        //CALIB_CB_NORMALIZE_IMAGE Not sure what it dose - opencv themselves says "Normalize the image gamma with equalizeHist before applying fixed or adaptive thresholding."
        if(found){
            //if any corners are found, this will save and show them
            foundCorners.push_back(pointBuf);
            cv::drawChessboardCorners(*iter,BoardSize,pointBuf,found);
            cv::imshow("looking for corners",*iter);
            cv::waitKey(0);
        }
    }
}

void ImageProcessing::calibrate()
{
    const float squareEdgeLength = 0.02; //meters - Stores the side lengths from calibration checker board - 0.02 = placeholder
    const cv::Size checkerboardDimentions = BoardSize;
    //17x24

    //tmp function to run shit
    std::vector<cv::Mat> pitchers;
    std::vector<std::vector<cv::Point2f>> corners, rejektedCorners;
    ImageProcessing imp;
    for(int i =1; i<6;i++){
        pitchers.push_back(imp.loadImagePC(std::to_string(i)));
    }


    imp.getBoardCorners(pitchers,corners);

}

void ImageProcessing::run(){
    //tmp function to run shit
    std::vector<cv::Mat> pitchers;
    std::vector<std::vector<cv::Point2f>> corners, rejektedCorners;
    ImageProcessing imp;
    for(int i =1; i<6;i++){
        pitchers.push_back(imp.loadImagePC(std::to_string(i)));
    }


    imp.getBoardCorners(pitchers,corners);
}

std::vector<cv::Mat> ImageProcessing::pylonPic(){
    int myExposure = 30000;
    std::vector<cv::Mat> imgVector;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;

        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;

        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        while ( camera.IsGrabbing() && imgVector.size()<31)
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

                // Create an OpenCV display window.
                cv::namedWindow( "myWindow", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

                // Display the current image in the OpenCV display window.
                cv::imshow( "myWindow", openCvImage);

                if(cv::waitKey(1) == 'p'){              // take picture
                    imgVector.push_back(openCvImage);
                    cv::imshow( "myWindow0", openCvImage);
                } else if(cv::waitKey(1) == 'q'){       //quit
                    camera.Close();
                    break;
                }
            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
        << e.GetDescription() << std::endl;
    }

    return imgVector;

}


