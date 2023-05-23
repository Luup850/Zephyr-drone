//#include <opencv2/opencv.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
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
    cam.set(cv::CAP_PROP_FPS, 10);

    // Get camera default framerate

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cam.read(frame);
    frame_hud = frame.clone();
    start = clock();
    printf("Tracker constructor\n");
}

void Tracker::update()
{
    // Old frame
    cv::Mat frame_old = frame.clone();

    cam.read(frame);
    frame_hud = frame.clone();
    cv::Mat mat1_gray, mat2_gray;
    cv::cvtColor(frame_old, mat1_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame, mat2_gray, cv::COLOR_BGR2GRAY);
    // Equal
    if(cv::countNonZero(mat1_gray != mat2_gray) == 0)
    {
        printf("Frame is equal\n");
    }
    /*
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // Define detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    // Detect markers
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

    // Draw Frame Axes
    if (DRAW_HUD && markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMat, distMat, rvecs, tvecs);
        for (int i = 0; i < markerIds.size(); i++)
        {
            //cv::drawAxis(frame, cameraMat, distMat, rvecs[i], tvecs[i], 0.1);
            cv::aruco::drawDetectedMarkers(frame_hud, markerCorners, markerIds);
        }
    }

    // Draw fps onto frame_hud
    if(DRAW_HUD)
        cv::putText(frame_hud, std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

    */
    // Calculate FPS
    fps_counter++;
    if (clock() - start > CLOCKS_PER_SEC)
    {
        //printf("FPS: %d\n", fps_counter);
        fps = fps_counter;
        fps_counter = 0;
        start = clock();
    }
}


/*
int main(int argc, char** argv)
{
    VideoCapture cap;
    Mat frame;

    cap.open(0);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //cap.read(frame);
    //namedWindow( "Live", WINDOW_AUTOSIZE );
    //imshow("Live", frame);
    //waitKey(0);
    for (;;)
    {
            
        printf("loop\n");
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        namedWindow( "Live", WINDOW_AUTOSIZE );
        imshow("Live", frame);
        waitKey(25);

        // Save image
        //imwrite("test.jpg", frame);
        //if (waitKey(5) >= 0)
        //    break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;

}*/