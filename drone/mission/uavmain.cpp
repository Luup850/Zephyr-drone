#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>

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

#include "serial_if.h"

bool startNatNetConnection(const char * argv0);
void unpack(char * pData);

UOptitrack * frame = nullptr;

int main(int argc, char **argv)
{
    int ret, i, j;
    char buf[BUFFERSIZE];
    serial_if *sf = new serial_if();
    
    printf("###########################\n");
    printf("#  Starting test mission  #\n");
    printf("###########################\n");
    
    if(sf->open_conn() == -1)
    {
      fprintf(stderr, "Error in open\n");
      return(1);
    }
    
    usleep(500000);
    
    sf->sendmsg("ref 20 0 0 0");
    usleep(500000);
    sf->sendmsg("ref 0 0 0 0");
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
 }