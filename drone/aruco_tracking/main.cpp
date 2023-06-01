#include <aruco_tracker.hpp>


using namespace cv;
using namespace std;

// Prototype functions
void TakeImages(int amount);
void Callibrate();

int main(int argc, char** argv)
{
    TakeImages(40);
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
    /*while(running)
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
    }*/

    return 0;

}

void TakeImages(int amount)
{
    //Tracker* tracker = new Tracker(0);
    //tracker->update();
    //waitKey(0);
    VideoCapture cam;

    cam.open(0);
    cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cam.set(cv::CAP_PROP_XI_AUTO_WB, 0);
    cam.set(cv::CAP_PROP_TEMPERATURE,10);
    // Print resolutioon
    //printf("Resolution: %d x %d\n", (int)cam.get(cv::CAP_PROP_FRAME_WIDTH), (int)cam.get(cv::CAP_PROP_FRAME_HEIGHT));
    cam.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    for (int i = 0; i < amount; i++)
    {
        Mat frame;
        while(true)
        {
            cam.read(frame);
            cv::namedWindow( "Live", cv::WINDOW_AUTOSIZE );
            imshow("Live", frame);
            if (waitKey(30) >= 0) break;
        }
        imwrite("callibImg/" + to_string(i) + ".jpg", frame);
        printf("Saved image %d\n", i);
        //waitKey(0); // Wait for next keypress
    }    
}
/*
// Get camera matrix and distortion coefficients from images in callibImg folder using the chessboard pattern
void Callibrate()
{
    // Chessboard pattern size
    Size patternsize(9,7);
    vector<Point2f> corners; //this will be filled by the detected corners

    vector<vector<Point2f>> imgPoints; // 2D points in image plane.
    vector<vector<Point3f>> objPoints; // 3D points in real world space

    vector<Point3f> objp;
    for(int i = 0; i < patternsize.height; i++)
    {
        for(int j = 0; j < patternsize.width; j++)
        {
            objp.push_back(Point3f(j,i,0));
        }
    }

    vector<String> fn;
    glob("callibImg/*.jpg", fn, false);

    Mat img, gray;
    for (size_t i=0; i<fn.size(); i++)
    {
        img = imread(fn[i]);
        // Grayscale image
        cv::cvtColor(img, gray, COLOR_BGR2GRAY);

        // Find chessboard corners
        bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if(patternfound)
        {
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            imgPoints.push_back(corners);
            objPoints.push_back(objp);
        }
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;

    calibrateCamera(objPoints, imgPoints, img.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "Camera matrix: " << cameraMatrix << endl;
    cout << "Distortion coefficients: " << distCoeffs << endl;

    FileStorage fs("cameraMatrix.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs.release();

    FileStorage fs2("distCoeffs.xml", FileStorage::WRITE);
    fs2 << "distCoeffs" << distCoeffs;
    fs2.release();
}*/