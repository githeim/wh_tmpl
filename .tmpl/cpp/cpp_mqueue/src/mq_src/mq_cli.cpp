#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mqueue.h>

#define QUEUE_NAME  "/test_queue"
#define MAX_SIZE    1024
#define MSG_STOP    "stop"


int main(int argc, char **argv)
{
  mqd_t Mqueue;
  char pBuf[MAX_SIZE];

  /* open the mail queue */
  Mqueue = mq_open(QUEUE_NAME, O_WRONLY);
  if (Mqueue == (mqd_t)(-1)) {
    printf("\033[1;31m[%s][%d] :x: Q open error, "
        "client should be executed after executing server [%d]\033[m\n",
        __FUNCTION__,__LINE__,Mqueue);
    exit(1);
  }

  printf("Send to server(enter \"stop\" to quit)\n");

  do {
    printf("> ");
    fflush(stdout);

    memset(pBuf, 0, MAX_SIZE);
    // Get input from keyboard
    fgets(pBuf, MAX_SIZE, stdin);

    if (mq_send(Mqueue, pBuf, MAX_SIZE, 0) <(mqd_t)(0)) {
      printf("\033[1;31m[%s][%d] :x: msg send error [%d]\033[m\n",
          __FUNCTION__,__LINE__,Mqueue);
      exit(1);
    }

  } while (strncmp(pBuf, MSG_STOP, strlen(MSG_STOP)));

  if (mq_close(Mqueue) ==(mqd_t)(-1)) {
    printf("\033[1;31m[%s][%d] :x: Q closing error [%d]\033[m\n",
        __FUNCTION__,__LINE__,Mqueue);
    exit(1);

  }
  return 0;
}
