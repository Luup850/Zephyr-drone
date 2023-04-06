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
#include "controller.h"
//#include "pid_custom.h"
#include "serial_if.h"


bool startNatNetConnection(const char * argv0);
void unpack(char * pData);
void* velocity(void* arg);

UOptitrack * frame = nullptr;
URigid *drone_marker;
double PX = 0, PY = 0, PZ = 0;
double VX = 0, VY = 0, VZ = 0;

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
    PID* ctrl_h = new PID();
    PID* ctrl_yaw = new PID();
    PID* ctrl_x = new PID();
    PID* ctrl_y = new PID();
    Controller* ctrl_pos = new Controller(&PX, &PY, &PZ, &R, &P, &Y, ctrl_x, ctrl_y);


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

    char str[100];
    int count = 0;
    ctrl_h->set_setpoint(1.5);
    ctrl_h->set_gains(1, 0.1, 1);
    ctrl_h->set_measurement(&PZ);

    ctrl_yaw->set_setpoint(0);
    ctrl_yaw->set_gains(1, 0.01, 0);
    ctrl_yaw->set_measurement(&Y);

    ctrl_x->set_setpoint(PX);
    ctrl_x->set_gains(1.5, 0.3, 0.5);
    ctrl_x->set_measurement(&PX);

    ctrl_y->set_setpoint(PY);
    ctrl_y->set_gains(1.5, 0.3, 0.5);
    ctrl_y->set_measurement(&PY);

    while(isOK)
    {

        usleep(50000);
        ctrl_h->tick();
        ctrl_yaw->tick();
        ctrl_pos->tick();
        double error_h = ctrl_h->out;
        double error_yaw = ctrl_yaw->out;
        double error_roll = -ctrl_pos->out_roll;
        double error_pitch = ctrl_pos->out_pitch;
        // Height roll pitch yaw.
        sprintf(str, "ref %f %f %f %f", 82.0 + error_h, error_roll, error_pitch, error_yaw);
        sf->sendmsg(str);

        if(count > 10)
        {
            printf("POS: %.3f %.3f %.3f\n", PX, PY, PZ);
            // Convert from radians to degrees by multiplying by 57.2957795.
            printf("Angles: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795);
            printf("Error: H=%.3f R=%.3f P=%.3f Y=%.3f\n", error_h +82, error_roll, error_pitch, error_yaw);
            printf("--------------------------------------------------------------------\n");
            count = 0;
        }
        count++;
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
   drone_marker = frame->findMarker(24149); // Marker for the drone.
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

// Calculate velocity using a different thread. This is done to avoid blocking the main thread.
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
            double time_diff = (difftime(clock(), start)/CLOCKS_PER_SEC);
            VX = (PX - PX_old) / time_diff;
            VY = (PY - PY_old) / time_diff;
            VZ = (PZ - PZ_old) / time_diff;
            PX_old = PX;
            PY_old = PY;
            PZ_old = PZ;
            start = clock();
        }
    }
}