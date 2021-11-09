#include "ImageProcessing.h"

ImageProcessing::ImageProcessing(){}

void ImageProcessing::calibrate()
{
    std::string input;
    std::cout<<"Run new calibration? [y/n]";
    std::cin>> input;

    if(input=="y"){
        this->chessboardDetection(this->pylonPic());
        std::cout << "Ready for cropping? (y)" << std::endl;
        std::cin >> input;
    } else if(input=="n")
        this->chessboardDetection(this->loadLocalImg());

    input = "";
    imgAmt = 1;
    std::vector<cv::Point> tempPoints;

    cropCornerPoints = this->cornersHoughCircles(this->pylonPic()[0]);

//    while (true) {
//        cropCornerPoints = this->cornersTempleMatching(this->pylonPic()[0]);
//        if (cv::waitKey() == 'y') {
//            cv::destroyAllWindows();
//            break;
//        }
//        else
//            cv::destroyAllWindows();

//    }

//    cv::Mat tmp = this->cropImg(this->pylonPic()[0]);
//    cv::Point ball = this->ballDetection(tmp);
//    while(ball.x == -1 && ball.y==-1){
//        std::cout<<"take new image when ready, press p"<<std::endl;
//        if(cv::waitKey()=='p'){
//            tmp = this->cropImg(this->pylonPic()[0]);
//            ball = this->ballDetection(tmp);
//        }
//    }

//    cv::imshow("Result", tmp);
//    cv::waitKey();

//    this->getBallCoords();

}

std::vector<double> ImageProcessing::getBallCoords()
{
    cv::Mat img = this->pylonPic()[0].clone();
    std::cout<<"lort";
    cv::Mat crop = this->cropImg(img);
    cv::Point imgPoints = this->ballDetection(crop);
    std::vector<double> realCoords = this->coordConvert(imgPoints,crop);

    return realCoords;
}

std::vector<cv::Mat> ImageProcessing::pylonPic(){
    std::vector<cv::Mat> imgVector;
    imgVector.clear();
    int myExposure = 20000;

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
            //            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        //        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
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
        //        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        while ( camera.IsGrabbing() && imgVector.size()<imgAmt)
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);


            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                if(imgVector.size()>=imgAmt || cv::waitKey(1) == 'q'){
                    camera.Close();
                    break;
                }
                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);
                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
                //If calibration has run _calibrationMat will be bigger than 0 and this code runs
                if(_calibrationMat.size()>0){
                    cv::Mat mapX, mapY;
                    mapX = _calibrationMat[0].clone();
                    mapY = _calibrationMat[1].clone();
                    cv::Mat imgUndistorted;
                    cv::remap(openCvImage, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
                    //cv::Mat crop = this->cropImg(imgUndistorted).clone();

                    if(!autoImg ){cv::imshow( "Undistorted image"+std::to_string(imgVector.size()), imgUndistorted);}
                    if(cv::waitKey(1) == 'p' || autoImg){
                        //cv::Mat tmp=imgUndistorted.clone();
                        imgVector.push_back(imgUndistorted);
                        if(showimg || !autoImg){cv::destroyWindow("Undistorted image"+std::to_string(imgVector.size()-1));}
                        if(imgVector.size()>=imgAmt){
                            camera.Close();
                            break;
                        }
                    }
                }
                //If not calibrated take X amount op pics
                else{
                    //                    cv::Rect iCrop(100, 10, 900, 600);
                    //                    cv::Mat cropImg = openCvImage(iCrop);
                    cv::imshow( "myWindow"+std::to_string(imgVector.size()), openCvImage);}
                if(cv::waitKey() == 'p'){
                    cv::Mat tmp=openCvImage.clone();
                    imgVector.push_back(tmp);
                    cv::destroyWindow("myWindow"+std::to_string(imgVector.size()-1));
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

void ImageProcessing::chessboardDetection(std::vector<cv::Mat> imgVec) {
    for(int i = 0; i<imgVec.size();i++){
        cv::imwrite("../app/imageProcessing/images/calibration" + std::to_string(i) + ".jpg", imgVec.at(i));
    }

    std::vector<std::vector<cv::Point2f>> q(imgVec.size());
    std::vector<std::vector<cv::Point3f>> Q;
    // 1. Generate checkerboard (world) coordinates Q. The board has 25 x 18
    // fields with a size of 15x15mm

    // int checkerBoard[2] = {6,9};
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i = 1; i<=BoardSize.height; i++){
        for(int j = 1; j<=BoardSize.width; j++){
            objp.push_back(cv::Point3f(j,i,0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : imgVec) {
        //std::cout << std::string(f) << std::endl;

        // 2. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = f.clone();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(gray, BoardSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if(!patternFound){
            std::cout<<"No chessboard found on image" + std::to_string(i)<<std::endl;
            std::cout<<"Take new images!"<<std::endl;
            break;
        }

        // 2. Use cv::cornerSubPix() to refine the found corner detections
        if(patternFound){
            cv::cornerSubPix(gray, q[i],cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objp);
        }
        if(showimg){
            // Display
            cv::drawChessboardCorners(img, BoardSize, q[i], patternFound);
            cv::imshow("chessboard detection", img);
            cv::waitKey(0);
        }
        i++;
    }


    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
            cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440,1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
    // Show lens corrected images
    cv::imwrite("../app/imageProcessing/images/mapX.jpg", mapX);
    cv::imwrite("../app/imageProcessing/images/mapY.jpg", mapY);
    for (auto const &f : imgVec) {

        //std::cout << std::string(f) << std::endl;

        cv::Mat img = f.clone();
        //std::cout<<"test"<<std::endl;
        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        // Display
        if(showimg){
            cv::imshow("undistorted image", imgUndistorted);
            cv::waitKey(0);
        }
    }
    cv::destroyAllWindows();
    _calibrationMat.push_back(mapX);
    _calibrationMat.push_back(mapY);
}

std::vector<cv::Mat> ImageProcessing::loadLocalImg()
{
    std::vector<cv::Mat> imgVec;
    std::vector<cv::String> fileNames;
    cv::glob("../app/imageProcessing/images/calibration*.jpg", fileNames, false);
    for(int i = 0; i<fileNames.size();i++){
        imgVec.push_back(cv::imread(fileNames[i]));
        //cv::imshow( "myWindow" +std::to_string(i), imgVec[i]);
        //cv::waitKey(0);
    }
    return imgVec;
}


cv::Mat ImageProcessing::cropImg(cv::Mat img)
{
    // first upper left
    //seckond upper right
    int y = (cropCornerPoints[0].y+cropCornerPoints[1].y)/2; // Average Y height
    int x = (cropCornerPoints[1].x+cropCornerPoints[2].x)/2;// Average x with
    cv::Mat crop = img(cv::Range(y,cropCornerPoints[2].y),cv::Range(cropCornerPoints[0].x, x)).clone(); // Slicing to crop the image

    return crop;
}

cv::Mat ImageProcessing::Threshold(cv::Mat image)
{
    cv::Mat img, crop, gray, out, outNorm, outNormSc;
    //cv::imwrite("../app/imageProcessing/images/fuck_shit.jpg",notimg);
    //img = cv::imread("../app/imageProcessing/images/fuck_shit.jpg").clone();
    //cv::Mat crop = img(cv::Range(300,600),cv::Range(300,1150)).clone();
    //first range top/bottom, seckond left/right

    img=image.clone();

    //MANUAL THRESHOLDING - RGB
    crop = img(cv::Range(400,800),cv::Range(500,900)).clone();
    cv::Mat imgMod = img.clone();
    //cv::vec3b m = cv::mean(crop);
    uchar deviation = 30;

    long avrgTmp1=0, avrgTmp2=0, avrgTmp3=0;
    long iter=1;

    for (int r = 0; r < crop.rows; r++) {
        for (int c = 0; c < crop.cols; c++) {
            avrgTmp1 = avrgTmp1+crop.at<cv::Vec3b>(r,c)[0];
            avrgTmp2 = avrgTmp2+crop.at<cv::Vec3b>(r,c)[1];
            avrgTmp3 = avrgTmp3+crop.at<cv::Vec3b>(r,c)[2];
            iter++;
        }
    }
    avrgTmp1=avrgTmp1/iter;
    avrgTmp2=avrgTmp2/iter;
    avrgTmp3=avrgTmp3/iter;


    for (int r = 0; r < imgMod.rows; r++) {
        for (int c = 0; c < imgMod.cols; c++) {


            if(imgMod.at<cv::Vec3b>(r,c)[0] > avrgTmp1 + deviation){imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;
                imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;}

            else if(imgMod.at<cv::Vec3b>(r,c)[1] > avrgTmp2 + deviation){imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;
                imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;}

            else if(imgMod.at<cv::Vec3b>(r,c)[2] > avrgTmp3 + deviation){imgMod.at<cv::Vec3b>(r,c)[2] = imgMod.at<cv::Vec3b>(r,c)[2]*0;
                imgMod.at<cv::Vec3b>(r,c)[0] = imgMod.at<cv::Vec3b>(r,c)[0]*0;
                imgMod.at<cv::Vec3b>(r,c)[1] = imgMod.at<cv::Vec3b>(r,c)[1]*0;}
        }
    }
    cv::imshow("bin", imgMod);
    cv::waitKey();


    //MANUAL THRESHOLDING - GRAY
    cv::cvtColor(imgMod, gray, cv::COLOR_BGR2GRAY);

    crop = gray(cv::Range(400,800),cv::Range(500,900)).clone();
    cv::Mat grayMod = gray.clone();
    cv::Scalar m = cv::mean(crop);

    for (int r = 0; r < gray.rows; r++) {
        for (int c = 0; c < gray.cols; c++) {
            if(gray.at<uchar>(r,c)> m[0]+deviation){grayMod.at<uchar>(r,c) = 255;}
            else if(gray.at<uchar>(r,c)< m[0]-deviation){grayMod.at<uchar>(r,c) = 255;}
            else{grayMod.at<uchar>(r,c)=0;}


        }
    }
    cv::imshow("bin", grayMod);
    cv::waitKey();


    //    cv::Mat tmpMat;
    //    out =  cv::Mat::zeros(grayMod.size(), CV_32FC1);
    //    cv::cornerHarris(grayMod, out, 3, 3, 0.02);

    //    cv::normalize(out, outNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    //    cv::convertScaleAbs(outNorm, outNormSc);

    //    for (int j = 0; j < outNorm.rows; j++) {
    //        for (int i = 0; i < outNorm.cols; i++) {
    //            if ((int)outNorm.at<float>(j, i) >150) {
    //                circle(img, cv::Point(i, j), 4, cv::Scalar(0, 0, 255), 2, 8, 0);
    //            }
    //        }
    //    }
    //    cv::imshow("Output Harris", img);
    //    cv::waitKey();


    return grayMod;
}

std::vector<double> ImageProcessing::coordConvert(cv::Point imgPos, cv::Mat img)
{

    //cam height 139 cm
    float x, y;
    float xWith = img.cols, yWith = img.rows;
    float realX = 80; //cm
    float lengthPerPixel = realX/xWith;
    x = imgPos.x*lengthPerPixel;
    if(x<40){
        x=x+((40-x)/40*0.4);
    }
    else if(x>40){
        x=x+((40-x)/40*0.4);
    }
    y = imgPos.y*lengthPerPixel;

    std::vector<double> points;
    points.push_back(x);
    points.push_back(y);


    std::cout<<"x position: " + std::to_string(x) + " y position: " + std::to_string(y)<<std::endl;
    std::cout<<"rows: " + std::to_string(img.rows) + " cols: " + std::to_string(img.cols)<<std::endl;
    std::cout<<"point x: " + std::to_string(imgPos.x) + " point y: " + std::to_string(imgPos.y)<<std::endl;
    return points;
}

void ImageProcessing::lastStand(cv::Mat img)
{
    cv::Mat image, gray, crop;
    cv::Mat output, output_norm, output_norm_scaled;
    //    image = cv::imread("../app/imageProcessing/images/fuck_shit.jpg").clone();
    image = img.clone();
    crop = image(cv::Range(350,700),cv::Range(300,1100)).clone();

    //cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);


    // Detecting corners using the goodFeaturesToTrack built in function
    std::vector<cv::Point2f> corners;
    goodFeaturesToTrack(crop,
                        corners,
                        100,            // Max corners to detect
                        0.1,           // Minimal quality of corners
                        10,             // Minimum Euclidean distance between the returned corners
                        cv::Mat(),          // Optional region of interest
                        10,              // Size of an average block for computing a derivative covariation matrix over each pixel neighbothood
                        false,          // Use Harri Detector or cornerMinEigenVal - Like when you create your own
                        0.04);          // Free parameter for the Harris detector


    // Drawing a circle around corners
    for (size_t i = 0; i < corners.size(); i++){
        circle(crop, corners[i], 4, cv::Scalar(0, 255, 0), 2, 8, 0);
    }

    // Displaying the result
    cv::imshow("Output Shi-Tomasi", crop);
    cv::waitKey();




}

std::vector<cv::Point> ImageProcessing::cornersTempleMatching(cv::Mat img)
{
    std::vector<cv::Mat> results, tpl;
    std::vector<cv::Point> points;
    cv::Mat res, gref, gtpl;

    cv::Mat tplR = cv::imread("../app/imageProcessing/images/tokenRight.jpg").clone();
    cv::Mat tplL = cv::imread("../app/imageProcessing/images/tokenLeft1.jpg").clone();
    cv::Mat ref = img.clone();

    tpl.push_back( tplR.clone());
    tpl.push_back( tplL.clone());

    for(int i=0;i<tpl.size();i++){
        cv::cvtColor(ref, gref, cv::COLOR_BGR2GRAY);
        cv::cvtColor(tpl[i], gtpl, cv::COLOR_BGR2GRAY);

        const int low_canny = 30;
        cv::Canny(gref, gref, low_canny, low_canny*3);
        cv::Canny(gtpl, gtpl, low_canny, low_canny*3);

        cv::Mat res_32f(ref.rows - tpl[i].rows + 1, ref.cols - tpl[i].cols + 1, CV_32FC1);
        matchTemplate(gref, gtpl, res_32f, cv::TM_CCOEFF_NORMED);

        res_32f.convertTo(res, CV_8U, 255.0);

        int size = ((tpl[i].cols + tpl[i].rows) / 4) * 2 + 1; //force size to be odd
        cv::adaptiveThreshold(res, res, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, size, -100);

        double minval, maxval;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);
        points.push_back(cv::Point(maxloc.x + (tpl[i].cols/2), maxloc.y + (tpl[i].rows/2)));
        cv::circle( ref, points[i], 10, cv::Scalar(0,100,255), 5, cv::LINE_AA); // Draws center
    }

    cv::imshow("Correct corners? (y/n)", ref);
    return points;
}

cv::Point ImageProcessing::ballDetection(cv::Mat src) {
    std::vector<cv::Point> points;
    std::vector<cv::Mat> bufferImages;
    cv::Mat src_grey;
    cvtColor(src, src_grey, cv::COLOR_BGR2GRAY);
    cv::medianBlur(src_grey, src_grey, 5);

    std::vector<cv::Vec3f> circles;
    HoughCircles(src_grey, circles, cv::HOUGH_GRADIENT, 1, src_grey.rows/16, 100, 30, 15, 30); // The last two parameters is min & max radius
    for( size_t i = 0; i < circles.size(); i++ ) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        points.push_back(center);
        circle( src_grey, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA); // Draws center
        circle( src_grey, center, c[2], cv::Scalar(255,0,255), 3, cv::LINE_AA); // Draws radius
        bufferImages.push_back(src_grey.clone());
    }
    if(points.size() > 1) {
        for (unsigned int i = 0; i < bufferImages.size(); i++) {
            cv::imshow("showImage" + std::to_string(i), bufferImages.at(i));
            cv::waitKey();
            cv::destroyWindow("showImage" + std::to_string(i));
        }
        std::cout << "Put in the correct picture number: ";
        std::string input;
        std::cin >> input;
        std::cout << "Picture number " + input + " have been selected" << std::endl;

        return points.at(std::stoi(input));
    } else if (points.size() == 1){

        cv::imshow("testo", src_grey);
        cv::waitKey();


        return points.at(0);
    } else {
        std::cout << "No table tennis ball found!" << std::endl;
        cv::imshow("PointsNotFound", src_grey);
        cv::waitKey();
        return cv::Point(-1, -1);
    }
}

std::vector<cv::Point> ImageProcessing::cornersHoughCircles(cv::Mat src){
    cv::Mat image_hsv;
    cv::cvtColor(src, image_hsv, cv::COLOR_BGR2HSV);
    cv::medianBlur(image_hsv, image_hsv, 9);

    cv::Mat lower_red_hue_range, upper_red_hue_range, red_hue_image;

    cv::inRange(image_hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), lower_red_hue_range);
    cv::inRange(image_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    cv::imshow("red_hue_image", red_hue_image);
    cv::waitKey();

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

    if(circles.size() == 0) std::exit(-1);
    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        int radius = std::round(circles[current_circle][2]);

        cv::circle(image_hsv, center, radius, cv::Scalar(0, 255, 0), 5);
    }

    cv::imshow("image_hsv", image_hsv);
    cv::waitKey();


//    cv::cvtColor(lower_red_hue_range, ballsDetectBGR, cv::COLOR_HSV2BGR);

    std::vector<cv::Point> points;
//    points.push_back(this->ballDetection(ballsDetectBGR));


    return points;
}







