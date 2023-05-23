#include <aruco_tracker.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <unistd.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <aruco_tracker.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Tracker* tracker = new Tracker(0);
    
    for(;;)
    {
        tracker->update();
        namedWindow( "Live", WINDOW_AUTOSIZE );
        imshow("Live", tracker->frame_hud);
        waitKey(25);
    }

    return 0;

}