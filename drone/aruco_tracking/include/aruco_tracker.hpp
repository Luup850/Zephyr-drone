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
        Tracker(int camID);

        void update();
        // {0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872};
        cv::Mat distMat = (cv::Mat_<double>(1,5) << 0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872);

        // {{809.7256631082, 0.0, 266.4705282079}, {0, 817.1412927192, 273.6516853926}, {0, 0, 1}};
        cv::Mat cameraMat = (cv::Mat_<double>(3,3) << 809.7256631082, 0.0, 266.4705282079, 0, 817.1412927192, 273.6516853926, 0, 0, 1);

        cv::Mat frame;
        cv::Mat frame_hud;

        int fps_counter = 0;
        int fps = 0;

        // Debug stuff
        int counter = 0;
        cv::Mat frame_2;
        clock_t f1;
        float t_diff, t0, t1, t2;

    cv::VideoCapture cam;

    private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    clock_t start;

};
#endif