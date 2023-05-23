#include <aruco_tracker.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Tracker* tracker = new Tracker(0);
    tracker->update();

    //namedWindow( "Live", WINDOW_AUTOSIZE );
    //imshow("Live", tracker->frame);
    //waitKey(0);
    for(int i = 0; i <= 100;i++)
    {
        //namedWindow( "Live", WINDOW_AUTOSIZE );
        //imshow("Live", tracker->frame_hud);
        //waitKey(1);
        tracker->update();
        if(i == 100)
        {
            printf("FPS: %d\n", tracker->fps);
            printf("Cam FPS: %f\n", tracker->cam.get(CAP_PROP_FPS));
            printf("Buffer size: %f\n", tracker->cam.get(CAP_PROP_BUFFERSIZE));
            break;
        }
    }

    return 0;

}