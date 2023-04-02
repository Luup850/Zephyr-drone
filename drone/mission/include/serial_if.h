# ifndef SERIAL_IF_DOT_H
# define SERIAL_IF_DOT_H

# include <stdlib.h>
# include <stdio.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <unistd.h>
# include <math.h>
# include <string.h>
# include <termios.h>
# include <errno.h>
# include <pthread.h>

# define BUFFERSIZE 512

class serial_if
{
  public:
  
    serial_if();
    
    bool conn_open;
    bool rolling;
    
    int open_conn();
    int sendmsg(char *msg);
    int recvmsg(char *msg);
    int close_conn();
    int check_sum(char *msg);
    
  private:
    int portif;
    const char *devname = "/dev/ttyACM0";
    pthread_t recvthread_handle;
    
    void *recvthread(void);
    static void *recvthread_helper(void *var){ return(((serial_if *) var)->recvthread()); }
};

# endif //SERIAL_IF_DOT_H
