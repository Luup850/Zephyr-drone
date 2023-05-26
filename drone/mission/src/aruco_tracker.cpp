//#include <opencv2/opencv.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <pthread.h>
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

#include <aruco_tracker.hpp>


//using namespace cv;
//using namespace std;

Tracker::Tracker(int camID)
{
    cam.open(camID);
    cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
    //cam.set(cv::CAP_PROP_FPS, 10);

    // set resolution to 360p
    //cam.set(cv::CAP_PROP_FRAME_WIDTH, 1);
    //cam.set(cv::CAP_PROP_FRAME_HEIGHT, 1);

    // Get camera default framerate

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cam.read(frame);
    frame_hud = frame.clone();
    //start = clock();
    generic_clock = clock();
    printf("Tracker constructor\n");
}

bool Tracker::update()
{
    //cv::Mat frame_old = frame.clone();

    cam.read(frame);

    // Rectify frame with camera matrix and distortion matrix
    //cv::undistort(frame, frame_hud, cameraMat, distMat);

    if(DRAW_HUD)
        frame_hud = frame.clone();

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // Define detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    // Detect markers
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.135, cameraMat, distMat, rvecs, tvecs);

    // Draw fps onto frame_hud
    if(DRAW_HUD)
    {
        cv::putText(frame_hud, std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::aruco::drawDetectedMarkers(frame_hud, markerCorners, markerIds);

        // Draw x y and z for first object
        if (markerIds.size() > 0)
        {
            auto x = tvecs[0][0] * 100.0;
            auto y = tvecs[0][1] * 100.0;
            auto z = tvecs[0][2] * 100.0;

            // limit x y z to 2 decmials
            x = (int)(x * 100 + .5);
            x = (float)x / 100;
            y = (int)(y * 100 + .5);
            y = (float)y / 100;
            z = (int)(z * 100 + .5);
            z = (float)z / 100;

            auto pitch = rvecs[0][0] * 57.2957795;
            auto yaw = rvecs[0][1] * 57.2957795;
            auto roll = rvecs[0][2] * 57.2957795;

            // limit pitch yaw roll to 2 decmials
            pitch = (int)(pitch * 100 + .5);
            pitch = (float)pitch / 100;
            yaw = (int)(yaw * 100 + .5);
            yaw = (float)yaw / 100;
            roll = (int)(roll * 100 + .5);
            roll = (float)roll / 100;

            cv::putText(frame_hud, "x: " + std::to_string(x), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame_hud, "y: " + std::to_string(y), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame_hud, "z: " + std::to_string(z), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            cv::putText(frame_hud, "pitch: " + std::to_string(pitch), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame_hud, "yaw: " + std::to_string(yaw), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame_hud, "roll: " + std::to_string(roll), cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // Draw x and y lines on aruco marker
            cv::line(frame_hud, markerCorners[0][0], markerCorners[0][1], cv::Scalar(0, 255, 0), 2);
        }
    }

    // Calculate FPS
    fps_counter++;
    if (clock() - fps_clock > CLOCKS_PER_SEC)
    {
        //printf("FPS: %d\n", fps_counter);
        fps = fps_counter;
        fps_counter = 0;
        fps_clock = clock();
    }

    /*----------------------------------------------------------------------
        Inverse transforms and stuff
    ----------------------------------------------------------------------*/

    // Transform from camera frame to aruco frame
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rvecs[0][0], rvecs[0][1], rvecs[0][2]);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << tvecs[0][0], tvecs[0][1], tvecs[0][2]);

    // Create a transformation matrix using Rodrigues rotation conversion
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    // Create a 4x4 transformation matrix by combining rotation and translation
    cv::Mat transformationMatrix = cv::Mat::eye(4, 4, CV_64F);
    rotationMatrix.copyTo(transformationMatrix(cv::Rect(0, 0, 3, 3)));  // Copy rotation to top-left 3x3 submatrix
    tvec.copyTo(transformationMatrix(cv::Rect(3, 0, 1, 3)));

    cv::Mat point = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

    // Deep copy transformation matrix
    cv::Mat transformationMatrixInverse = cv::Mat::eye(4, 4, CV_64F);

    cv::invert(transformationMatrix, transformationMatrixInverse);

    

    cv::Mat point_inverse = transformationMatrix.inv() * point;
    // Transform point from camera frame to aruco frame
    cv::Mat point_transformed = transformationMatrix * point;

    printf("------------------[FPS: %.2d]--------------------\n", fps);
    printf("x: %.2f, y: %.2f, z: %.2f\n", point_transformed.at<double>(0, 0) * 100.0, point_transformed.at<double>(1, 0) * 100.0, point_transformed.at<double>(2, 0) * 100.0);
    // This is the actual pos of the marker relative to global frame
    printf("x: %.2f, y: %.2f, z: %.2f Inverse\n", point_inverse.at<double>(0, 0) * 100.0, point_inverse.at<double>(1, 0) * 100.0, point_inverse.at<double>(2, 0) * 100.0);

    if(DRAW_HUD)
    {
        return drawHUD();
    }
    else
    {
        return true;
    }
}

bool Tracker::drawHUD()
{
    cv::namedWindow( "Live", cv::WINDOW_AUTOSIZE );
    cv::imshow("Live", frame_hud);

    if(cv::waitKey(5) >= 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
