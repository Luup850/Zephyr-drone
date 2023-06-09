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
#include <math.h>

#include <aruco_tracker.hpp>
#include <chrono>

//using namespace cv;
//using namespace std;

Tracker::Tracker(int camID)
{
    cam.open(camID);
    cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cam.set(cv::CAP_PROP_XI_AUTO_WB, 0);
    cam.set(cv::CAP_PROP_TEMPERATURE,10);
    cam.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);


    //cam.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
    //cam.set(cv::CAP_PROP_EXPOSURE, 1);
    //cam.set(cv::CAP_PROP_XI_AUTO_WB, 0.0);

    printf("Auto exposure: %lf\n", cam.get(cv::CAP_PROP_AUTO_EXPOSURE));
    printf("Exposure: %lf\n", cam.get(cv::CAP_PROP_EXPOSURE));
    printf("Auto white balance: %lf\n", cam.get(cv::CAP_PROP_XI_AUTO_WB));

    // Print resolutioon
    //printf("Resolution: %d x %d\n", (int)cam.get(cv::CAP_PROP_FRAME_WIDTH), (int)cam.get(cv::CAP_PROP_FRAME_HEIGHT));
    //cam.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    //cam.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    printf("Resolution: %d x %d\n", (int)cam.get(cv::CAP_PROP_FRAME_WIDTH), (int)cam.get(cv::CAP_PROP_FRAME_HEIGHT));
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

// Returns true if execution was successfully completed
// Used to time the discretized controllers such that their sampletime can be dynamically adjusted
bool Tracker::update()
{
    //printf("Called update\n");
    //foundMarker = false;
    //cv::Mat frame_old = frame.clone();
    cam.read(frame);

    // Normalize frame
    //cv::normalize(frame, frame, -50, 205, cv::NORM_MINMAX, CV_8UC1);

    // Apply sharpness
    //cv::Mat kernel = (cv::Mat_<float>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    //cv::filter2D(frame, frame, -1, kernel);

    // Normalize frame
    //cv::normalize(frame, frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    if(frame.empty())
    {
        printf("Frame empty\n");
        return false;
    }

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

    // Define object points
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-0.16/2.f, 0.16/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(0.16/2.f, 0.16/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(0.16/2.f, -0.16/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-0.16/2.f, -0.16/2.f, 0);

    // Aruco image scaling is done here. Old value was 0.135
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.135, cameraMat, distMat, rvecs, tvecs);
    //for (int i = 0; i < markerCorners.size(); i++) 
    //{
    //    printf("Marker id: %d\n", markerCorners.size());
    //    printf("Got marker\n");
    //    cv::solvePnP(objPoints, markerCorners.at(i), cameraMat, distMat, rvecs.at(i), tvecs.at(i));
    //}

    if(markerIds.size() > 0)
    {
        foundMarker = true;
        // Align camera with drone frame
        x_l = tvecs[0][1];
        y_l = -tvecs[0][0];
        z_l = -tvecs[0][2];

        // Offset and scaling fix
        /*
            y:-0.47 to -1.43 is equal to 6 cm.
            x:2.04 to 6.65 is equal to 6 cm.

            Y: 1/6 = 0.1666666666666667
            X: 4.61/6 = 0.7683333333333333
            Middle is x5.1 y0.51
        */
        //x_l = (tvecs[0][0] - 5.0) * 0.7683;
        //y_l = tvecs[0][1] * 0.16667;
        //z_l = -tvecs[0][2];
        //printf("Found marker\n");
    }
    else
    {
        foundMarker = false;
        //printf("No marker found\n");
    }

    // Draw fps onto frame_hud
    if(DRAW_HUD)
    {
        cv::putText(frame_hud, std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        // Draw small red cross in center of frame
        cv::line(frame_hud, cv::Point(frame_hud.cols / 2 - 5, frame_hud.rows / 2), cv::Point(frame_hud.cols / 2 + 5, frame_hud.rows / 2), cv::Scalar(0, 0, 255), 2);
        cv::line(frame_hud, cv::Point(frame_hud.cols / 2, frame_hud.rows / 2 - 5), cv::Point(frame_hud.cols / 2, frame_hud.rows / 2 + 5), cv::Scalar(0, 0, 255), 2);

        cv::aruco::drawDetectedMarkers(frame_hud, markerCorners, markerIds);

        // Draw x y and z for first object
        if (foundMarker)
        {
            double x = x_l * 100.0;
            double y = y_l * 100.0;
            double z = z_l * 100.0;

            // limit x y z to 2 decmials
            //x = (int)(x * 100 + .5);
            //x = (float)x / 100;
            //y = (int)(y * 100 + .5);
            //y = (float)y / 100;
            //z = (int)(z * 100 + .5);
            //z = (float)z / 100;

            //auto pitch = rvecs[0][0] * 57.2957795;
            //auto yaw = rvecs[0][1] * 57.2957795;
            //auto roll = rvecs[0][2] * 57.2957795;

            // limit pitch yaw roll to 2 decmials
            //pitch = (int)(pitch * 100 + .5);
            //pitch = (float)pitch / 100;
            //yaw = (int)(yaw * 100 + .5);
            //yaw = (float)yaw / 100;
            //roll = (int)(roll * 100 + .5);
            //roll = (float)roll / 100;

            cv::putText(frame_hud, "x: " + std::to_string(x), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            cv::putText(frame_hud, "y: " + std::to_string(y), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            cv::putText(frame_hud, "z: " + std::to_string(z), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

            cv::putText(frame_hud, "X: " + std::to_string(arucoPos[0] * 100.0), cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            cv::putText(frame_hud, "Y: " + std::to_string(arucoPos[1] * 100.0), cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            //cv::putText(frame_hud, "Z: " + std::to_string(arucoPos[2] * 100.0), cv::Point(10, 210), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

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
    if(foundMarker)
    {

        // Tvecs[0] to cv::Mat
        cv::Mat tvecMat = cv::Mat::zeros(3, 1, CV_64F);
        tvecMat.at<double>(0, 0) = x_l;
        tvecMat.at<double>(1, 0) = y_l;
        tvecMat.at<double>(2, 0) = z_l;

        cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
        cv::Vec3d rollPitchYaw = cv::Vec3d(*roll, *pitch, *yaw);
        cv::Rodrigues(rollPitchYaw, rotMat);

        // Inverse rotation
        cv::Mat pose_mat = rotMat.t() * tvecMat;
        arucoPos[0] = pose_mat.at<double>(0, 0);
        arucoPos[1] = pose_mat.at<double>(1, 0);
        arucoPos[2] = pose_mat.at<double>(2, 0);

        if(ARUCO_DEBUG_PRINT and tick % 20 == 0)
        {
            // Prints in centimeters
            printf("Pitch: %.2f \t Yaw: %.2f \t Roll: %.2f\n", *pitch * 57.2957795, *yaw * 57.2957795, *roll * 57.2957795);
            printf("x: %.2f \t y: %.2f \t z: %.2f\n", arucoPos[0] * 100.0, arucoPos[1] * 100.0, arucoPos[2] * 100.0);
        }
    }

    if(ARUCO_DEBUG_PRINT and tick % 20 == 0)
    {
        printf("------------------[FPS: %.2d]--------------------\n", fps);   
    }
    if(ARUCO_LOG and enable_log)
    {
        log();
    }

    tick++;
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
    
    cv::waitKey(5);
    return true;
}

void Tracker::log()
{
    if(firstLog and enable_log)
    {
        firstLog = false;
        printf("ARUCO: Logging to file...\n");
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        char *time = ctime(&now_c);
        time[strlen(time)-1] = '\0';
        char filename[100];
        sprintf(filename, "aruco_logs/%s.log", time);
        fp = fopen(filename, "a+");
        fprintf(fp, "Units are seconds, meters, and radians\n");
        fprintf(fp, "Time, FPS, x, y, z, x_raw, y_raw, z_raw, pitch, yaw, roll");
        fprintf(fp, "\n");
        log_prev_time = clock();
        log_start_time = clock();
    }
    else if(enable_log)
    {
        if((difftime(clock(), log_prev_time)/CLOCKS_PER_SEC) > 0.250)
        {
            // This is where we log stuff
            //printf("Logging...\n");
            double time = difftime(clock(), log_start_time)/CLOCKS_PER_SEC;
            fprintf(fp, "%.2f, %d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", 
                        time, fps, arucoPos[0], arucoPos[1], arucoPos[2], x_l, y_l, z_l, *pitch, *yaw, *roll);
            log_prev_time = clock();
        }
    }
}