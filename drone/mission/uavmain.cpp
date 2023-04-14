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
#include "pid.h"
#include "serial_if.h"

// Defines
#define LOG_TO_FILE true

bool startNatNetConnection(const char * argv0);
void unpack(char * pData);
void* velocity(void* arg);

UOptitrack * frame = nullptr;
URigid *drone_marker;
double PX, PY, PZ;
double VX = 0, VY = 0, VZ = 0;

// Pitch, yaw, roll
double P, Y, R;

int main(int argc, char **argv)
{
    // Logging
    FILE *bs, *fp;
    if(LOG_TO_FILE)
    {
        bs = fopen("log", "w");
        fclose(bs);
        usleep(500000);
        fp = fopen("log", "a+");
    }

    // Initialization.
    int ret, i, j;
    char buf[BUFFERSIZE];
    serial_if *sf = new serial_if();
    Drone* drone;
    drone = new Drone();
    PID* ctrl_h = new PID();
    PID* ctrl_yaw = new PID();
    PID* ctrl_vel_x = new PID();
    PID* ctrl_vel_y = new PID();
    PID* ctrl_x = new PID();
    PID* ctrl_y = new PID();
    Controller* ctrl_pos = new Controller(&PX, &PY, &PZ, &R, &P, &Y, ctrl_vel_x, ctrl_vel_y);


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
    usleep(500000);
    double x_tmp = PX, y_tmp = PY, z_tmp = PZ;

    char str[100];
    int count = 0;

    // PID values from model for height.
    // h1 Kp = 26.7, ti=1.2, td = 1.26. Default values in matlab are kp = 60, ti = 1, td = 1.
    ctrl_h->set_setpoint(2);
    ctrl_h->set_gains(6, 1.2, 1.26); 
    ctrl_h->set_measurement(&PZ);

    ctrl_yaw->set_setpoint(Y);
    ctrl_yaw->set_gains(3, 0, 0.4);
    ctrl_yaw->set_measurement(&Y);

    ctrl_vel_x->set_setpoint(ctrl_x->out);
    ctrl_vel_x->set_gains(0.0823,0,0.2087); // MATLAB vel control PD: 0.0823, 0, 0.5087
    ctrl_vel_x->set_measurement(&VX);

    ctrl_vel_y->set_setpoint(ctrl_y->out);
    ctrl_vel_y->set_gains(0.0823,0,0.2087);
    ctrl_vel_y->set_measurement(&VY);

    ctrl_x->set_setpoint(PX);
    ctrl_x->set_minmax(-1, 1); // Limit speed to 1 m/s
    ctrl_x->set_gains(0.5343, 0, 0); // MATLAB PID pos control: 0.6749, 3.7413, 1.7077. PD 1.2168, 1.1303. P 0.5343
    ctrl_x->windup_limit(-1, 1);
    ctrl_x->set_measurement(&PX);

    ctrl_y->set_setpoint(PY);
    ctrl_y->set_minmax(-1, 1);
    ctrl_y->set_gains(0.5343, 0, 0);
    ctrl_y->windup_limit(-1, 1);
    ctrl_y->set_measurement(&PY);


    // Disable controllers
    //ctrl_h->set_gains(10,0,0);
    //ctrl_yaw->set_gains(10,0,0);
    //ctrl_vel_x->set_gains(10,0,0);
    //ctrl_vel_y->set_gains(10,0,0);
    //ctrl_x->set_gains(0.6, 0, 0.8);
    //ctrl_y->set_gains(0.6, 0, 0.8);
    int tst = 0;
    while(isOK)
    {
        ctrl_vel_x->set_setpoint(ctrl_x->out);
        ctrl_vel_y->set_setpoint(ctrl_y->out);
        usleep(50000);
        ctrl_h->tick();
        ctrl_yaw->tick();
        ctrl_x->tick(); // ctrl_x and y has to be called because ctrl_pos only calls ctrl_vel_x and y.
        ctrl_y->tick();
        ctrl_pos->tick_matlab();
        double error_h = ctrl_h->out;
        double error_yaw = -(ctrl_yaw->out);// * (180/M_PI); // Convert to degrees
        double error_roll = (ctrl_pos->out_roll) * (180/M_PI); // Convert to degrees
        double error_pitch = -(ctrl_pos->out_pitch) * (180/M_PI); // Convert to degrees
        // Height roll pitch yaw.
        sprintf(str, "ref %f %f %f %f", 82.0 + error_h, error_roll, error_pitch, error_yaw);
        sf->sendmsg(str);

        if( count > 5)
        {
            if (LOG_TO_FILE)
            {
                printf("save %d\n", tst);
                fprintf(fp, "[%d]\n", tst);
                fprintf(fp, "POS: %.3f %.3f %.3f, OLD: %.3f %.3f %.3f\n", PX, PY, PZ, x_tmp, y_tmp, z_tmp);
                // Convert from radians to degrees by multiplying by 57.2957795.
                fprintf(fp, "Angles: %.3f %.3f %.3f, Speed: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795, VX, VY, VZ);
                fprintf(fp, "Signal: H=%.3f R=%.3f P=%.3f Y=%.3f\n", error_h +82, error_roll, error_pitch, error_yaw);
                fprintf(fp, "Error XY reg: X=%.3f Y=%.3f\t Error XY vel reg: X=%.3f Y=%.3f \n", ctrl_x->out, ctrl_y->out, ctrl_vel_x->out, ctrl_vel_y->out);
                fprintf(fp, "--------------------------------------------------------------------\n");
            }
            else
            {
                printf("[%d]\n", tst);
                printf("POS: %.3f %.3f %.3f, OLD: %.3f %.3f %.3f\n", PX, PY, PZ, x_tmp, y_tmp, z_tmp);
                // Convert from radians to degrees by multiplying by 57.2957795.
                printf("Angles: %.3f %.3f %.3f, Speed: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795, VX, VY, VZ);
                printf("Signal: H=%.3f R=%.3f P=%.3f Y=%.3f\n", error_h +82, error_roll, error_pitch, error_yaw);
                printf("Error XY reg: X=%.3f Y=%.3f\t Error XY vel reg: X=%.3f Y=%.3f \n", ctrl_x->out, ctrl_y->out, ctrl_vel_x->out, ctrl_vel_y->out);
                printf("--------------------------------------------------------------------\n");
            }
            count = 0;
            tst++;
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
   drone_marker = frame->findMarker(24152); // Marker for the drone 24152.
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
   else
   {
        printf("ERROR: Marker not found!\n");
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
        
        double time_diff = (difftime(clock(), start)/CLOCKS_PER_SEC);

        if(time_diff > 0.02 and PX != PX_old)
        {
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