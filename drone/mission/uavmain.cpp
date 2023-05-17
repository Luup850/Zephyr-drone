#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>

#include <sys/types.h>
#include <cinttypes>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <signal.h>

//#include "main.h"
#include <optidata.h>
#include "drone.h"
#include "controller.h"
#include "pid.h"
#include "serial_if.h"
#include "logger.h"

// Defines
#define LOG_TO_FILE false // False: Log to console, True: Log to file
#define LOG true // Log to console or file
#define LOGGER_TOGGLE true // Logger class
#define TS (1.0/60.0)
#define HOVER_VALUE 430.0 // 430-440 required for hover
#define DRONE_ID 24152

bool startNatNetConnection(const char * argv0);
void unpack(char * pData);
void* velocity(void* arg);
void* controllerTick(void* arg);

UOptitrack * frame = nullptr;
URigid *drone_marker;
double PX, PY, PZ;
double VX = 0, VY = 0, VZ = 0;
double height_measurement; // Height controller is weird. Trying to mimic it.

// Controllers
PID* ctrl_vel_h;
PID* ctrl_h;
PID* ctrl_yaw;
PID* ctrl_vel_x;
PID* ctrl_vel_y;
PID* ctrl_x;
PID* ctrl_y;

// Timer stuff
struct itimerval it;
struct timeval tv;
sigset_t signalset;
int sig;

// Logging
Logger *lg;


// Pitch, yaw, roll
double P, Y, R;

int main(int argc, char **argv)
{
    //// Timer stuff
    // Block timer signal (SIGALRM) to avoid program exits when signal arrives
    sigemptyset(&signalset);
    sigaddset(&signalset, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &signalset, NULL);
    
    // Setup and start timer to run 100 Hz (timer value = 10000 us)
    getitimer(ITIMER_REAL, &it); 
    tv.tv_sec = 0.0;
    tv.tv_usec = 16666.6667;
    it.it_interval = tv;
    it.it_value = tv;
    setitimer(ITIMER_REAL, &it, NULL);


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
    ctrl_vel_h = new PID(RegPLead, TS);
    ctrl_h = new PID(RegP, TS);
    ctrl_yaw = new PID(RegPLead, TS);
    ctrl_vel_x = new PID(RegPLead, TS);
    ctrl_vel_y = new PID(RegPLead, TS);
    ctrl_x = new PID(RegP, TS);
    ctrl_y = new PID(RegP, TS);
    Controller* ctrl_pos = new Controller(&PX, &PY, &PZ, &R, &P, &Y, ctrl_vel_x, ctrl_vel_y);

//{PX, PY, PZ, P, Y, R, error_h, error_roll, error_pitch, error_yaw}
    lg = new Logger("X, Y, Z, Pitch, Yaw, Roll, errorHeight, errorRoll, errorPitch, errorYaw, HLeadOut, HIntegralOut, HeightError, HVelLeadOut->yl[0], ctrl_vel_h->ref", LOGGER_TOGGLE);


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
    double x_tmp = PX, y_tmp = PY, z_tmp/* For set height*/, yaw_tmp = Y * (180.0/M_PI);

    char str[100];
    int count = 0;

    // PID values from model for height.
    // h1 Kp = 26.7, ti=1.2, td = 1.26. Default values in matlab are kp = 60, ti = 1, td = 1.
    // Bouncy but decent results findpid(Gh1a, 32,  3.5, 0.1).
    //ctrl_h->set_gains(20.0, 0.0, 0.7448, 0.07); // Best so far
    //ctrl_h->set_gains(22.3872, 6.66, 3.0237, 0.07);
    // 1.7448 0.7448 0.3448
    ctrl_vel_h->set_gains(44.6684, 0, 0.7559, 0.07);
    ctrl_h->set_gains(16.4106, 0, 0, 0.00);
    ctrl_h->limit_output(0.5, -0.5);
    //ctrl_h->set_gains(21.9636, 6.4475, 1.8021, 0.2); // Pt bedste: 21.9636, 6.4475, 1.8021, 0.2
    //ctrl_h->limit_integral(400,0);
    // MATLAB vel control PD: 0.0823, 0, 0.5087
    ctrl_vel_x->set_gains(0.2917, 0, 0.3468, 0.3);
    ctrl_vel_y->set_gains(0.2917, 0, 0.3468, 0.3);
    // Limit speed to 1 m/s
    // MATLAB PID pos control: 0.6749, 3.7413, 1.7077. PD 1.2168, 1.1303. P 0.5343
    ctrl_x->set_gains(0.9398, 0, 0, 0.2);
    ctrl_y->set_gains(0.9398, 0, 0, 0.2);
    ctrl_x->limit_output(1, -1);
    ctrl_y->limit_output(1, -1);

    ctrl_yaw->set_gains(0.2, 0, 0.2243, 0.5);
    ctrl_yaw->yaw_control = true;

    // Controller reference. Sets all to current pose
    ctrl_x->ref = PX;
    ctrl_y->ref = PY;
    ctrl_yaw->ref = Y;
    ctrl_h->ref = 4.0;
    z_tmp = ctrl_h->ref;

    // Controller measurement
    ctrl_x->measurement = &PX;
    ctrl_y->measurement = &PY;
    ctrl_h->measurement = &PZ;
    ctrl_vel_h->measurement = &VZ;
    ctrl_vel_x->measurement = &VX;
    ctrl_vel_y->measurement = &VY;
    ctrl_yaw->measurement = &Y;

    // Control tick thread
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, controllerTick, NULL);
    
    // Small wait to give time to get ready for takeoff
    printf("Takeoff in...\n");
    for(int i = 3; i > 0; i--)
    {
        printf("%d\n", i);
        sleep(1);

    }

    // Log parameters
    double to_log[] = {HOVER_VALUE, ctrl_x->ref, ctrl_y->ref, ctrl_yaw->ref, ctrl_h->ref, ctrl_h->kp, ctrl_h->tau_i, ctrl_h->tau_d, ctrl_h->alpha, ctrl_vel_x->tau_i, ctrl_vel_x->tau_d, ctrl_vel_x->alpha, ctrl_vel_h->kp, ctrl_vel_h->tau_i, ctrl_vel_h->tau_d};
    lg->log_params("HOVER_VALUE, ctrl_x->ref, ctrl_y->ref, ctrl_yaw->ref, ctrl_h->ref, ctrl_h->kp, ctrl_h->tau_i, ctrl_h->tau_d, ctrl_h->alpha, ctrl_vel_x->tau_i, ctrl_vel_x->tau_d, ctrl_vel_x->alpha, ctrl_vel_h->kp, ctrl_vel_h->tau_i, ctrl_vel_h->tau_d",to_log, 15);

    // Control Loop
    int tst = 0;
    while(isOK)
    {

        usleep(50000);
        ctrl_pos->tick_matlab();
        
        // Divide by 10.24 since drone ref is 0-100 and sim is 0 1024.
        double error_h = ctrl_vel_h->out + HOVER_VALUE;


        //double error_h = 50.0 - 82.0;
        double error_yaw = (ctrl_yaw->out);// * (180.0/M_PI); // Convert to degree
        //double error_yaw = 0;
        double error_roll = (ctrl_pos->out_roll); // Convert to degrees
        double error_pitch = (ctrl_pos->out_pitch); // Convert to degrees
        // Height roll pitch yaw.
        sprintf(str, "ref %.3f %.3f %.3f %.3f", error_h, error_roll, error_pitch, error_yaw);
        sf->sendmsg(str);
        
        double to_log[] = {PX, PY, PZ, P, Y, R, error_h, error_roll, error_pitch, error_yaw, ctrl_h->yl[0], ctrl_h->yi[0], PZ-z_tmp, ctrl_vel_h->yl[0], ctrl_vel_h->ref};
        lg->log(to_log, 13);
        // Logging
        if( count > 5 and LOG == true)
        {
            if (LOG_TO_FILE)
            {
                printf("save %d\n", tst);
                fprintf(fp, "[%d]\n", tst);
                fprintf(fp,"LeadH: %.3f, IntegralH: %.3f\n",ctrl_h->yl[0], ctrl_h->yi[0]);
                fprintf(fp, "POS: %.3f %.3f %.3f, OLD: %.3f %.3f %.3f, OLD heading: %.3f\n", PX, PY, PZ, x_tmp, y_tmp, z_tmp, yaw_tmp);
                // Convert from radians to degrees by multiplying by 57.2957795.
                fprintf(fp, "Angles: %.3f %.3f %.3f, Speed: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795, VX, VY, VZ);
                fprintf(fp, "Signal: H=%.3f R=%.3f P=%.3f Y=%.3f\n", error_h, error_roll, error_pitch, error_yaw);
                fprintf(fp, "Error in ref/measurement: X=%.3f, \t y=%.3f, \t H=%.3f\n", PX-x_tmp, PY-y_tmp, PZ-z_tmp);
                fprintf(fp, "Error XY reg: X=%.3f Y=%.3f\t Error XY vel reg: X=%.3f Y=%.3f\t Error H reg: %.3f \n", ctrl_x->output(), ctrl_y->output(), ctrl_vel_x->output(), ctrl_vel_y->output(), ctrl_h->output());
                fprintf(fp, "--------------------------------------------------------------------\n");
            }
            else
            {
                printf("LeadH: %.3f, IntegralH: %.3f\n",ctrl_h->ref - ctrl_h->yl[0], ctrl_h->yi[0]);
                //printf("IH: %.3f\n",ctrl_h->yi[0]);
                printf("POS: %.3f %.3f %.3f, OLD: %.3f %.3f %.3f, OLD heading: %.3f\n", PX, PY, PZ, x_tmp, y_tmp, z_tmp, yaw_tmp);
                // Convert from radians to degrees by multiplying by 57.2957795.
                printf("Angles: %.3f %.3f %.3f, Speed: %.3f %.3f %.3f\n", P*57.2957795, Y*57.2957795, R*57.2957795, VX, VY, VZ);
                printf("Signal: H=%.3f R=%.3f P=%.3f Y=%.3f\n", error_h, error_roll, error_pitch, error_yaw);
                printf("Error in ref/measurement: X=%.3f, \t y=%.3f, \t H=%.3f\n", PX-x_tmp, PY-y_tmp, PZ-z_tmp);
                printf("Error XY reg: X=%.3f Y=%.3f\t Error XY vel reg: X=%.3f Y=%.3f \n", ctrl_x->output(), ctrl_y->output(), ctrl_vel_x->output(), ctrl_vel_y->output());
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
   drone_marker = frame->findMarker(DRONE_ID); // Marker for the drone 24152.
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

void* controllerTick(void* arg)
{
    //clock_t time = clock();
    while(true)
    {
        sigwait(&signalset, &sig);
        //usleep(TS * 1000000);
        //printf("%.9f\n", (difftime(clock(), time)/CLOCKS_PER_SEC));
        //time = clock();
        ctrl_h->tick();
        ctrl_vel_h->ref = ctrl_h->output();
        ctrl_vel_h->tick();
        ctrl_yaw->tick();
        ctrl_x->tick();
        ctrl_y->tick();
        ctrl_vel_x->ref = ctrl_x->output();
        ctrl_vel_y->ref = ctrl_y->output();
        ctrl_vel_x->tick();
        ctrl_vel_y->tick();
    }
}
