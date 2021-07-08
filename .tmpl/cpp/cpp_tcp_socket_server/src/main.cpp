#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>  
#include <unistd.h> 
#include <map>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>

#define PORT_NUM (19973)
#define MAX_CONNECTION (30)

// :x: Responses for client requests
std::map<std::string,std::string> g_mapResponse = 
{
  {std::string("God"),std::string("Bless You")},
  {std::string("One"),std::string("Two")},
  {std::string("Hello"),std::string("Dear")},
  {std::string("Hell"),std::string("World")},
  {std::string("Two"),std::string("Three")},
  {std::string("Three"),std::string("Four")}
};

std::atomic<int> g_iAtomicThreadCount {0};
std::atomic<bool> g_bAtomicEndFlag {false};

void HwndSvc(int iServSocket) {
  g_iAtomicThreadCount++;
  printf("\033[1;32m[%s][%d] :x: start service thread, count : [%d]\033[m\n",
      __FUNCTION__,__LINE__,(int)g_iAtomicThreadCount);

  char pMsg[512];
  memset(pMsg,0x00,512);

  recv(iServSocket,pMsg,sizeof(pMsg),0);
  printf("\033[1;33m[%s][%d] :x: Received msg pMsg =%s sizeof(pMsg) =%d\033[m\n",
      __FUNCTION__,__LINE__,pMsg,(int)sizeof(pMsg));

  std::string strMsg = pMsg;
  // remove line feed ( '\n', '\r' )
  strMsg.erase(std::remove(strMsg.begin(), strMsg.end(), '\n'), strMsg.end());
  strMsg.erase(std::remove(strMsg.begin(), strMsg.end(), '\r'), strMsg.end());

  std::string strResponse ;
  auto item = g_mapResponse.find(strMsg);
  if ( item == g_mapResponse.end() ) {
    strResponse = "Can not find answer";
  } else
    strResponse = item->second;
  printf("\033[1;33m[%s][%d] :x: Response = [%s] size=[%d] \033[m\n",
      __FUNCTION__,__LINE__,strResponse.c_str(),
      (int)sizeof(strResponse.size())
      );

  send(iServSocket,strResponse.c_str(),strResponse.size(),0);
  close(iServSocket);

  g_iAtomicThreadCount--;
}

int main() {
  printf("Project WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");

  int iMainSock, iServSocket;
  struct sockaddr_in servAddr;
  struct sockaddr_storage serverStorage;
  socklen_t addr_size;

  // :x: session socket
  iMainSock = socket(PF_INET, SOCK_STREAM, 0);
  // :x: set Address family = Internet
  servAddr.sin_family = AF_INET;

  // :x: setup port num
  servAddr.sin_port = htons(PORT_NUM);

  // :x: Set IP address to localhost
  servAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
  memset(servAddr.sin_zero, 0, sizeof (servAddr.sin_zero));

  // :x: bind
  if (  bind(iMainSock, (struct sockaddr *) &servAddr, sizeof(servAddr)) ) {
    printf("\033[1;33m[%s][%d] :x: Cannot bind to port %d \033[m\n",
        __FUNCTION__,__LINE__,PORT_NUM);
    return -1;
  }

  if( listen(iMainSock,MAX_CONNECTION) == 0 )  {
    printf("\033[1;33m[%s][%d] :x: Listening \033[m\n",__FUNCTION__,__LINE__);
  }
  else
    printf("\033[1;31m[%s][%d] :x: Error \033[m\n",__FUNCTION__,__LINE__);

  while(1)
  {
    addr_size = sizeof serverStorage;
    iServSocket = accept(iMainSock, (struct sockaddr *) &serverStorage, &addr_size);

    std::thread* pThrServ = new std::thread(HwndSvc,iServSocket);
    usleep(1);
  }
  close(iMainSock);
  return 0;
}
