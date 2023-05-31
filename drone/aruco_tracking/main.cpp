#include <aruco_tracker.hpp>


using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Tracker* tracker = new Tracker(0);
    //tracker->update();

    //namedWindow( "Live", WINDOW_AUTOSIZE );
    //imshow("Live", tracker->frame);
    //waitKey(0);
    bool flipflop = false;

    double roll = 0;
    double pitch = 0;
    double yaw = 3.14/2.0;

    tracker->roll = &roll;
    tracker->pitch = &pitch;
    tracker->yaw = &yaw;

    bool running = true;
    while(running)
    {
        running = tracker->update();

        if(flipflop == false and tracker->foundMarker)
        {
            flipflop = true;
            printf("Found marker!\n");
        }
        else if(flipflop == true and !tracker->foundMarker)
        {
            flipflop = false;
            printf("Lost marker!\n");
        }
        //printf("x: %.2lf, \ty: %.2lf, \tz: %.2lf\n", tracker->getX(0), tracker->getY(0), tracker->z_c);
    }

    return 0;

}