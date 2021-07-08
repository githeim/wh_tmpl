#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <string>
#define PORT_NUM (19973)

int main(int argc, char const *argv[])
{
  printf("Project WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");

  int iClientSock = 0;
  struct sockaddr_in serv_addr;
  if ((iClientSock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    printf("\n Socket creation error \n");
    return -1;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT_NUM);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  if (connect(iClientSock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
    printf("\nConnection Failed \n");
    return -1;
  }

  std::string strMsg="Hell";
  char pBuffer[1024] = {0};

  memset(pBuffer,0x00,1024);
  send(iClientSock , strMsg.c_str() , strMsg.size() , 0 );
  printf("\033[1;33m[%s][%d] :x: message [%s] sent \033[m\n",
      __FUNCTION__,__LINE__,strMsg.c_str());
  read( iClientSock , pBuffer, 1024);
  printf("\033[1;33m[%s][%d] :x: Response [%s] \033[m\n",
      __FUNCTION__,__LINE__,pBuffer);


  close(iClientSock);
  return 0;
}
