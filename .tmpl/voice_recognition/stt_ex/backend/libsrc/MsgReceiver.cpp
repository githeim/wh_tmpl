#include "MsgReceiver.h"
#include <unistd.h>
#include <thread>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <mqueue.h>


#define QUEUE_NAME  "/STT_Q"
#define MAX_SIZE    1024
#define MSG_STOP    "stop"



std::thread* g_pThrMsgReceiver = nullptr;

bool g_bMsgReceiverFlag = false;
bool get_bMsgReceiverFlag() {
  return g_bMsgReceiverFlag;
}
void set_bMsgReceiverFlag(bool bVal) {
  g_bMsgReceiverFlag=bVal;
}

void LoopMsgReceiver(std::vector<std::string> &vecSTT_Msg,
                     std::mutex &mtxSTT_Msg
    ) {
  mqd_t Mqueue;
  struct mq_attr Attr;
  char pBuf[MAX_SIZE + 1];

  Attr.mq_flags = 0;
  Attr.mq_maxmsg = 10;
  Attr.mq_msgsize = MAX_SIZE;
  Attr.mq_curmsgs = 0;

  if (mq_unlink(QUEUE_NAME)) {
    printf("\033[1;33m[%s][%d] :x: No Queue to Clear \033[m\n",__FUNCTION__,__LINE__);
  } else{
    printf("\033[1;33m[%s][%d] :x: Queue Cleared \033[m\n",__FUNCTION__,__LINE__);
  }

  Mqueue = mq_open(QUEUE_NAME, O_CREAT | O_RDONLY| O_NONBLOCK, 0644, &Attr);
  if (Mqueue == (mqd_t)(-1)) {
    printf("\033[1;31m[%s][%d] :x: Q open error [%d]\033[m\n",
        __FUNCTION__,__LINE__,Mqueue);
    exit(1);
  }


 
  while (get_bMsgReceiverFlag()) {
    ssize_t sizeRead;

    /* receive the message */
    sizeRead = mq_receive(Mqueue, pBuf, MAX_SIZE, NULL);
    if (sizeRead == (mqd_t)(-1)) {
      usleep(50000);
      continue;
    }
    pBuf[sizeRead] = '\0';

    // Check the stop flag
    if (! strncmp(pBuf, MSG_STOP, strlen(MSG_STOP)))
    {
      set_bMsgReceiverFlag(false);
    }
    else
    {
      printf("Received \n[%s]\n",pBuf);

      mtxSTT_Msg.lock();
      vecSTT_Msg.push_back(pBuf);
      mtxSTT_Msg.unlock();
    }
  }

}

void StartMsgReceiver(std::vector<std::string> &vecSTT_Msg,
                      std::mutex &mtxSTT_Msg) {
  set_bMsgReceiverFlag(true); 
  g_pThrMsgReceiver = new std::thread(LoopMsgReceiver,std::ref(vecSTT_Msg),std::ref(mtxSTT_Msg));
}
void StopMsgReceiver() {
  set_bMsgReceiverFlag(false); 
  if (g_pThrMsgReceiver !=nullptr) {
    g_pThrMsgReceiver->join();
    delete g_pThrMsgReceiver;
    g_pThrMsgReceiver=nullptr;
  }

}
