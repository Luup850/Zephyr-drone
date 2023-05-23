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
#include <aruco_tracker.hpp>

#include <aruco_tracker.hpp>

//using namespace cv;
//using namespace std;

Tracker::Tracker(int camID)
{
    cam.open(camID);
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    printf("Tracker constructor\n");
}

void Tracker::update()
{
    cam.read(frame);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

    cv::aruco::detectMarkers(frame, &dictionary, markerCorners, markerIds, &detectorParams, rejectedCandidates);

    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(frame_hud, markerCorners, markerIds);
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