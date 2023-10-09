#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <mqueue.h>

#define QUEUE_NAME  "/test_queue"
#define MAX_SIZE    1024
#define MSG_STOP    "stop"


int main(int argc, char **argv)
{
  mqd_t Mqueue;
  struct mq_attr Attr;
  char pBuf[MAX_SIZE + 1];
  bool bStopFlag= false;

  Attr.mq_flags = 0;
  Attr.mq_maxmsg = 10;
  Attr.mq_msgsize = MAX_SIZE;
  Attr.mq_curmsgs = 0;

  Mqueue = mq_open(QUEUE_NAME, O_CREAT | O_RDONLY, 0644, &Attr);
  if (Mqueue == (mqd_t)(-1)) {
    printf("\033[1;31m[%s][%d] :x: Q open error [%d]\033[m\n",
        __FUNCTION__,__LINE__,Mqueue);
    exit(1);
  }

  do {
    ssize_t sizeRead;

    /* receive the message */
    sizeRead = mq_receive(Mqueue, pBuf, MAX_SIZE, NULL);
    if (sizeRead == (mqd_t)(-1)) {
      printf("\033[1;31m[%s][%d] :x: Q read error [%d]\033[m\n",
          __FUNCTION__,__LINE__,Mqueue);
      exit(1);
    }
    pBuf[sizeRead] = '\0';

    // Check the stop flag
    if (! strncmp(pBuf, MSG_STOP, strlen(MSG_STOP)))
    {
      bStopFlag = true;
    }
    else
    {
      printf("Received %s",pBuf);
    }
  } while (bStopFlag != true);

  if (mq_close(Mqueue) ==(mqd_t)(-1)) {
    printf("\033[1;31m[%s][%d] :x: Q closing error [%d]\033[m\n",
        __FUNCTION__,__LINE__,Mqueue);
    exit(1);

  }
  mq_unlink(QUEUE_NAME);

  return 0;
}

