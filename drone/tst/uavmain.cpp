# include <stdio.h>
# include <stdlib.h>
# include <unistd.h>
# include <sys/time.h>
# include <signal.h>

# include "serial_if.h"

bool rolling = true;
void stopper(int signo)
{
  rolling = false;
}

int main(int argc, char **argv)
{
  int ret, i, j;
  char buf[BUFFERSIZE];
  serial_if *sf = new serial_if();
  
  signal(SIGINT, stopper);
  
  printf("###########################\n");
  printf("#  Starting test mission  #\n");
  printf("###########################\n");
  
  if(sf->open_conn() == -1)
  {
    fprintf(stderr, "Error in open\n");
    return(1);
  }
  
  usleep(500000);
  
  sf->sendmsg("sub hbt 1001");
  usleep(1000);
  sf->sendmsg("sub rc 50");
  usleep(1000);
  sf->sendmsg("sub esd 51");
  
  //for(i = 0; i < 10; i++)
  while(rolling)
  {
    //ret = sf->recvmsg(buf);
    //printf("%s", buf);
    //sf->check_sum(buf);
    /*printf(" (%d)\n", i);
    
    for(j = 0; j < ret; j++)
    {
      printf("0x%02X (%d)\n", buf[j], i);
    }
    */
    usleep(500000);
  }
  
  sf->rolling = false;
  sf->sendmsg("sub rc 0");
  sf->sendmsg("sub esd 0");
  usleep(500000);
  sf->close_conn();
  return(0);
}
