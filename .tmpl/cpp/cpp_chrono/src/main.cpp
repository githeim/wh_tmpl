#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <chrono>
int main(int argc, char *argv[]) {
  printf("\033[1;33m[%s][%d] :x: WW_ProjectName_WW \033[m\n",
      __FUNCTION__,__LINE__);

  printf("WW_PrintLetter_WW : Time Measurement (4 seconds)\n");
  printf("\033[1;33m[%s][%d] :x: Start count\033[m\n",
      __FUNCTION__,__LINE__);
  auto start= std::chrono::system_clock::now();
  sleep(4);
  auto end= std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end-start;
  printf("\033[1;33m[%s][%d] :x: %f seconds elapsed\033[m\n",
      __FUNCTION__,__LINE__,diff.count());

  return 0;
}
