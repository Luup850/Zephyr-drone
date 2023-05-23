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

class Tracker
{
    public:
        Tracker(int camID = 0);

        void init();
        void update();
        void draw();

        cv::Mat frame;
        cv::Mat frame_hud;

    private:
    cv::VideoCapture cam;
    cv::aruco::Dictionary dictionary;

};
#endif