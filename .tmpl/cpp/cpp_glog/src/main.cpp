#include <stdio.h>
#include "libmodule.h"
#include <glog/logging.h>

int main(int argc, char *argv[]) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  printf("Project WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");
  printf("libmodule() = %d\n", testmodule_Test());
  LOG(INFO) << "WW_PrintLetter_WW " ;

  while (1) {
    printf("\033[1;33m[%s][%d] :x: chk  WW_PrintLetter_WW\033[m\n",__FUNCTION__,__LINE__);
    LOG(INFO) << "Chk \033[1;32m"  <<  " WW_PrintLetter_WW  \033[m";
    sleep(1);
  }

  return 0;
}
