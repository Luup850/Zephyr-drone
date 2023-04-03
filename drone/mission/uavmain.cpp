#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>

#include <cinttypes>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>

//#include "main.h"
#include <optidata.h>
#include "drone.h"

#include "serial_if.h"


bool startNatNetConnection(const char * argv0);
void unpack(char * pData);
void* velocity(void* arg);

UOptitrack * frame = nullptr;
URigid *drone_marker;
double PX, PY, PZ;
double VX, VY, VZ;

// Pitch, yaw, roll
double P, Y, R;

int main(int argc, char **argv)
{
    // Initialization.
    int ret, i, j;
    char buf[BUFFERSIZE];
    serial_if *sf = new serial_if();
    Drone* drone;
    drone = new Drone();

    // Velocity calculations.
    pthread_t vel_thread;
    pthread_create(&vel_thread, NULL, velocity, NULL);
    
    printf("###########################\n");
    printf("#  Starting test mission  #\n");
    printf("###########################\n");
    
    if(sf->open_conn() == -1)
    {
      fprintf(stderr, "Error in open\n");
      return(1);
    }
    
    //usleep(500000);
    bool isOK = startNatNetConnection(argv[0]);

    double error;
    double setpoint = 1.0;
    char str[100];
    int count = 0;

    while(isOK)
    {
        usleep(50000);
        count++;
        error = (setpoint - PZ)*1.5;
        sprintf(str, "ref %f 0 0 0", 82.0 + error);
        sf->sendmsg(str);

        if(count > 10)
        {
            printf("POS: %.3f %.3f %.3f\n", PX, PY, PZ);
            printf("Angles: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795);
        }
    }
    //sf->sendmsg("ref 20 0 0 0");
    //usleep(500000);
    //sf->sendmsg("ref 0 0 0 0");
    return(0);
}


bool startNatNetConnection(const char * argv0)
{ // find IP/name of server and this machine
    const int MIL = 100;
    char ipStr[MIL];
    const char * optitrackServerInAsta = "DESKTOP-OVLVRUD.local";
    // IP of this machine
    strncpy(ipStr, "192.168.1.125", MIL);
    // generate an argc, argv set
    const char * argv1[4] = {argv0, optitrackServerInAsta, ipStr, "\0"};
    // run NatNet connection for optitrackServer
    // also starts threads for package reception
    // takes 3 string parameters (app name, server IP/name, client IP)
    bool started = setup(3, (char **)argv1);
    return started;
}

/**
 * Unpack NatNet frame.
 * Called from receive thread.
 * A new frame is detected 
 * \param pData is pointer to binary frame to int unpack(char* pData) */
void unpack(char * pData)
{ // should maybe be expanded to 
   // at least 2 frames with resource lock
   // activated during unpack/use
   if (frame == nullptr)
     frame = new UOptitrack();
   frame->unpack(pData);
   
   // Marker from OptiTrack
   drone_marker = frame->findMarker(24149);
   if(drone_marker->valid)
   {
        //printf("%lf, %f, %f, %f\n", frame->get_timestamp(), drone_marker->pos[0], drone_marker->pos[1], drone_marker->pos[2]);
        
        double q0 = drone_marker->rotq[3];
        double q1 = drone_marker->rotq[0];
        double q2 = drone_marker->rotq[1];
        double q3 = drone_marker->rotq[2];
        
        //double atti_p = asin(2.0 * (q0 * q2 - q1 * q3));
        //double atti_r = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (pow(q1, 2) + pow(q2, 2)));
        //double atti_y = atan2(2.0 * (q1 * q2 + q0 * q3), 1.0 - 2.0 * (pow(q2, 2) + pow(q3, 2)));
        P = asin(2.0 * (q0 * q2 - q1 * q3));
        R = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (pow(q1, 2) + pow(q2, 2)));
        Y = atan2(2.0 * (q1 * q2 + q0 * q3), 1.0 - 2.0 * (pow(q2, 2) + pow(q3, 2)));
        
        //printf("%lf, %lf, %lf\n", 180 / M_PI * atti_r, 180 / M_PI * atti_p, 180 / M_PI * atti_y);
        PX = drone_marker->pos[0];
        PY = -drone_marker->pos[1];
        PZ = drone_marker->pos[2];
   }
}

void* velocity(void* arg)
{
    double PX_old, PY_old, PZ_old;
    clock_t start = clock();

    while(true)
    {
        PX_old = PX;
        PY_old = PY;
        PZ_old = PZ;
        if(PX != PX_old)
        {
            VX = (PX - PX_old) / (difftime(clock(), start)/CLOCKS_PER_SEC);
            VY = (PY - PY_old) / (difftime(clock(), start)/CLOCKS_PER_SEC);
            VZ = (PZ - PZ_old) / (difftime(clock(), start)/CLOCKS_PER_SEC);
            PX_old = PX;
            PY_old = PY;
            PZ_old = PZ;
            start = clock();
        }
    }
}