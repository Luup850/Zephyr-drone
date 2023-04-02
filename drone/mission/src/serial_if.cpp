# include "serial_if.h"


serial_if::serial_if()
{
  conn_open = false;
  rolling = true;
  pthread_create(&recvthread_handle, NULL, &recvthread_helper, this);
}

void *serial_if::recvthread(void)
{
  char buf[BUFFERSIZE];
  int len;
  char *nlchr;
  
  while(rolling)
  {
    if(conn_open)
    {
      len = recvmsg(buf);
      if(len > 0)
      {        
        nlchr = strchr(buf, '\n');
        
        check_sum(buf);
        //printf(buf);
      }
    }
    //usleep(100);
  }
  
  pthread_exit(NULL);
}

int serial_if::open_conn()
{
  portif = open(devname, O_RDWR);
  if(portif == -1)
  {
    fprintf(stderr, "Unable to open port to board\n");
    conn_open = false;
    return(-1);
  }
  else
  { 
    struct termios ntio;
  
    if(tcgetattr(portif, &ntio) < 0) 
    {
      fprintf(stderr, "Unable to get terminal attributes\n");
      return(1);
    }
  
    cfsetispeed(&ntio, B115200);
    cfsetospeed(&ntio, B115200);
  
    // 8N1
    ntio.c_cflag &= ~PARENB;
    ntio.c_cflag &= ~CSTOPB;
    ntio.c_cflag &= ~CSIZE;
    ntio.c_cflag |= CS8;
  
    // no flow control
    ntio.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
    ntio.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    ntio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // turn off s/w flow ctrl

    ntio.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ISIG | ECHOCTL | ECHOKE); // make raw
    ntio.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    ntio.c_cc[VMIN]  = 0;
    ntio.c_cc[VTIME] = 100;
  
    tcsetattr(portif, TCSANOW, &ntio);
    if(tcsetattr(portif, TCSAFLUSH, &ntio) < 0) 
    {
       fprintf(stderr, "Unable to set terminal attributes\n");
       return(2);
    }
  }
  
  conn_open = true;
  return(0);
}

int serial_if::sendmsg(char *msg)
{
  int ret, i, sum = 0;
  char premsg[4];
  int len = strlen(msg);
  
  if(conn_open)
  {
    for(i = 0; i < len; i++)
      sum += msg[i];
    sum = (sum % 99) + 1;
    sprintf(premsg, ";%02d", sum);
    
    write(portif, premsg, 3);
    ret = write(portif, msg, len);
    write(portif, "\r\n", 2);
    return(ret);
  }
  else
  {
    fprintf(stderr, "Port not open, unable to send\n");
    return(-1);
  }
}

int serial_if::recvmsg(char *msg)
{
  int ret;
  
  if(conn_open)
  {
    ret = read(portif, msg, BUFFERSIZE);
    //printf(" -> ret = %d\n", ret);
    if(ret != -1)
    {
      msg[ret] = 0;
    }
    else
    {
      fprintf(stderr, "Recv error\n");
      perror("Failed: ");
    }
    return(ret);
  }
  else
  {
    fprintf(stderr, "Port not open, unable to recv\n");
    return(-1);
  }

}

int serial_if::close_conn()
{
  if(conn_open)
    close(portif);
  conn_open = false;
  return(0);
}

int serial_if::check_sum(char *msg)
{
  char *stchr, *edchr;
  int chk, sum = 0;
  bool res = false;
  
  stchr = strrchr(msg, ';');
  edchr = strrchr(msg, '\0');
  
  if((stchr == NULL) || (edchr == NULL))
  {
    fprintf(stderr, "Unable to find start/end markers in string\n");
    return(-1);
  }
  else
  {
    chk = atoi(stchr + 1);
    
    //printf("chk = %d\n", chk);
    
    stchr += 3;
    while(*stchr >= 0x20)
    {
      //printf("adding %c (0x%02X)\n", *stchr, *stchr);
      sum += *stchr++;
    }
    
    res = ((sum % 99) + 1) == chk;
    //printf("sum = %d  // %d (ok = %d)\n", sum, (sum % 99) + 1, res);
    
    if(res == false)
      fprintf(stderr, "Checksum error in msg = %s\n", msg);
  }
  
  return(0);
}
