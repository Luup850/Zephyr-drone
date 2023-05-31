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
#define DRAW_HUD true
#define ARUCO_DEBUG_PRINT false

class Tracker
{
    public:
        // Camera ID (Usually 0)
        Tracker(int camID);

        double x_c,y_c,z_c;
        double x_a,y_a,z_a;

        // Position of aruco code relative to camera with a rotation applied
        // Rotation is defined by pitch yaw and role
        double arucoPos[3];

        // Local frame (relative to camera) fixed so that x y and z fit the drones frame.
        // Given in meters.
        double x_l,y_l,z_l;

        // This is the rotation that is applied inverse to the aruco code
        // This gives the position relative to the drone but ignoring the pitch yaw and roll of the drone.
        double *pitch,*yaw,*roll;

        // Updates the tracker
        // Tracker takes a new frame and looks for aruco codes
        bool update();

        // Distortion matrix (Normal res) (640x480)
        cv::Mat distMat = (cv::Mat_<double>(1,5) << 0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872);
        // Camera matrix (Normal res)
        cv::Mat cameraMat = (cv::Mat_<double>(3,3) << 809.7256631082, 0.0, 266.4705282079, 0, 817.1412927192, 273.6516853926, 0, 0, 1);

        // (480x360) scaling 0.75
        //cv::Mat distMat = (cv::Mat_<double>(1,5) << 0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872);
        // Camera matrix (Normal res)
        //cv::Mat cameraMat = (cv::Mat_<double>(3,3) << 809.7256631082 * 0.75, 0.0, 266.4705282079 * 0.75, 0.0, 817.1412927192 * 0.75, 273.6516853926 * 0.75, 0.0, 0.0, 1);

        // Camera frame
        cv::Mat frame;
        // Camera frame with HUD
        cv::Mat frame_hud;

        cv::Mat rvec;
        cv::Mat tvec;

        int fps_counter = 0;
        int fps = 0;

        int tick = 0;

        bool foundMarker = false;

        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::VideoCapture cam;

    private:
        bool drawHUD();
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        clock_t fps_clock, generic_clock;

};
#endif