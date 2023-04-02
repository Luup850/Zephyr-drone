

#include <cinttypes>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <sys/time.h>

#include "main.h"
#include "optidata.h"

# define NOSAMPLES 200

double PX, PY, PZ;

/**
 * copy paste function from 
 * https://stackoverflow.com/questions/212528/get-the-ip-address-of-the-machine
 * to find IP string for this machine 
 * \param ipString is string to return result,
 * \param ipStringLen is maximum length of the provided string */
void findIP(char * ipString, int ipStringLen)
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  
  getifaddrs(&ifAddrStruct);
  
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr) {
      continue;
    }
    if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
      // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, ipString, ipStringLen);
      printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
    } else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
      // is a valid IP6 Address
      tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
      char addressBuffer[INET6_ADDRSTRLEN];
//       inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
//       printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
      inet_ntop(AF_INET6, tmpAddrPtr, ipString, ipStringLen);
    } 
  }
  if (ifAddrStruct!=NULL) 
    freeifaddrs(ifAddrStruct);
}

/**
 * setup parameters and start */
bool startNatNetConnection(const char * argv0)
{ // find IP/name of server and this machine
  const int MIL = 100;
  char ipStr[MIL];
  const char * optitrackServerInAsta = "DESKTOP-OVLVRUD.local";
  // IP of this machine
  findIP(ipStr, MIL);
  strncpy(ipStr, "192.168.1.125", MIL);
  // generate an argc, argv set
  const char * argv1[4] = {argv0, optitrackServerInAsta, ipStr, "\0"};
  // run NatNet connection for optitrackServer
  // also starts threads for package reception
  // takes 3 string parameters (app name, server IP/name, client IP)
  bool started = setup(3, (char **)argv1);
  return started;
}


UOptitrack * frame = nullptr;
URigid *drone_marker;

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
   
   drone_marker = frame->findMarker(24149);
   if(drone_marker->valid)
   {
     //printf("%lf, %f, %f, %f\n", frame->get_timestamp(), drone_marker->pos[0], drone_marker->pos[1], drone_marker->pos[2]);
     
     double q0 = drone_marker->rotq[3];
     double q1 = drone_marker->rotq[0];
     double q2 = drone_marker->rotq[1];
     double q3 = drone_marker->rotq[2];
     
     double atti_p = asin(2.0 * (q0 * q2 - q1 * q3));
     double atti_r = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (pow(q1, 2) + pow(q2, 2)));
     double atti_y = atan2(2.0 * (q1 * q2 + q0 * q3), 1.0 - 2.0 * (pow(q2, 2) + pow(q3, 2)));
     
     //printf("%lf, %lf, %lf\n", 180 / M_PI * atti_r, 180 / M_PI * atti_p, 180 / M_PI * atti_y);
     PX = drone_marker->pos[0];
     PY = drone_marker->pos[1];
     PZ = drone_marker->pos[2];
   }

     
 }


int main(int argc, char *argv[]) 
{ // estavlish connection
  bool isOK = startNatNetConnection(argv[0]);
  int c, cnt = 0;
  char szRequest[512];
  bool bExit = false;
  double data[NOSAMPLES][4];
  timeval tv0;
  sleep(1);

  while (not bExit) 
  {
    while(cnt < NOSAMPLES)
    {
	    gettimeofday(&tv0, NULL);
	    data[cnt][0] = tv0.tv_sec + 1e-6 * tv0.tv_usec;
	    data[cnt][1] = PX;
	    data[cnt][2] = PY;
	    data[cnt][3] = PZ;
	    usleep(66667);
	    cnt++;
    }

    printf("Press q\n");
	  
    c = getchar();
    switch (c) 
    {
      case 'q':
        bExit = true;
        break;
      default:
        break;
    }  
  }

  FILE *fp;
  fp = fopen("opti.log", "w");
  for(int ii = 0; ii < NOSAMPLES; ii++)
  {
    fprintf(fp, "%lf %lf %lf %lf\n", data[ii][0], data[ii][1], data[ii][2], data[ii][3]);
  }
}
