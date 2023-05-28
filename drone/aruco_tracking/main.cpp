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
    bool running = true;
    while(running)
    {
        running = tracker->update();

        //printf("x: %.2lf, \ty: %.2lf, \tz: %.2lf\n", tracker->getX(0), tracker->getY(0), tracker->z_c);
    }

    return 0;

}