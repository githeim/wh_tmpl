#include <stdio.h>
#include <unistd.h>
#include "PingChecker.h"

#include <chrono>
int main(int argc, char *argv[]) {
  printf("\033[1;33m[%s][%d] :x: WW_PrintLetter_WW \033[m\n",
      __FUNCTION__,__LINE__);
  if (argc !=2)
    return 1;
  long double ldPingTime_ms;
  int ret;
  std::string strAddr=argv[1];
  if (!SetupPingChecker(strAddr)) {
    for (int i = 0 ; i < 8 ; i++) {
      ret = GetPing(ldPingTime_ms);
      if (!ret) {
        printf("\033[1;33m[%s][%d] :x: Ping Time =%Lf \033[m\n",
            __FUNCTION__,__LINE__,ldPingTime_ms);
      }
      printf("\033[1;32m[%s][%d] :x: Delay 0.5 sec \033[m\n",
          __FUNCTION__,__LINE__);
      usleep(500000);
    }
  }
  return 0;
}
