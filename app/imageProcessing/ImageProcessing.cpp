#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::calibrate()
{
    this->pylonPic();
    std::vector<cv::Mat> tempPic = this->pylonPic();

    std::cout << tempPic.size() << std::endl; // debugger v2


    for (unsigned int i = 0; i < tempPic.size(); i++) {
        cv::imshow( "myWindow1", tempPic[i]); // debugger v2
        cv::imwrite("../app/imageProcessing/images/Gay_Ish" + std::to_string(i) + ".jpg", tempPic[i]);
    }

    this->getCornersV2();
}

std::vector<cv::Mat> ImageProcessing::pylonPic(){
    std::vector<cv::Mat> imgVector;

    if (_sim == false) {
        int myExposure = 30000;

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

            while ( camera.IsGrabbing() && imgVector.size()<5)
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

    } else {
        cv::Mat openCvImage;

        for (int i = 1; i <= 5; i++) {
            cv::Mat pic = cv::imread("../app/imageProcessing/images/image-00" + std::to_string(i) + ".jpg",-1);
            imgVector.push_back(pic);
        }
        return imgVector;
    }
}

void ImageProcessing::getCornersV2()
{

    std::vector<cv::String> fileNames;
    cv::glob("../app/imageProcessing/images/Gay_Ish*.jpg", fileNames, false);
    cv::Size patternSize(6, 9);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm

    //int checkerBoard[2] = {6,9};
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i = 1; i<=patternSize.height; i++){
        for(int j = 1; j<=patternSize.width; j++){
            objp.push_back(cv::Point3f(j,i,0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : fileNames) {
        std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = cv::imread(fileNames[i]);
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }

        // Display
        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("chessboard detection", img);
        cv::waitKey(0);

        i++;
    }


    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
            cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1280,800);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mapX, mapY);

    // Show lens corrected images
    for (auto const &f : fileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

        cv::Mat imgUndistorted;

        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

        // Display
        cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0);
    }


}



