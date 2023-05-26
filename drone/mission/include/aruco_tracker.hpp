#ifndef ARUCO_TRACKER_HPP
#define ARUCO_TRACKER_HPP
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#define DRAW_HUD false

class Tracker
{
    public:
        // Camera ID (Usually 0)
        Tracker(int camID);

        bool update();

        // Distortion matrix
        cv::Mat distMat = (cv::Mat_<double>(1,5) << 0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872);
        // Camera matrix
        cv::Mat cameraMat = (cv::Mat_<double>(3,3) << 809.7256631082, 0.0, 266.4705282079, 0, 817.1412927192, 273.6516853926, 0, 0, 1);

        // Camera frame
        cv::Mat frame;
        // Camera frame with HUD
        cv::Mat frame_hud;

        int fps_counter = 0;
        int fps = 0;

        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::VideoCapture cam;

    private:
        bool drawHUD();
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        clock_t fps_clock, generic_clock;

};
#endif