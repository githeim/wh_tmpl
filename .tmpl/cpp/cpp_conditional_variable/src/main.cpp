#include <stdio.h>
#include "libmodule.h"
#include <thread>
#include <mutex>
#include <string>
#include <condition_variable>
#include <unistd.h>
#include <vector>

std::mutex g_mtxData;

std::string g_strProducedData;
bool g_bProduceData = false;

std::condition_variable g_cvData;

bool g_bFlagWorker = false;

void Data_Producer() {
  sleep(1);
  for (int i=0 ; i < 100 ; i++) {
    g_strProducedData = std::string("Data :" ) + std::to_string(i);
    printf("\033[1;33m[%s][%d] :x: produce %s\033[m\n",
        __FUNCTION__,__LINE__,g_strProducedData.c_str());
    g_cvData.notify_one();
    
    usleep(10000);
  }
  g_strProducedData = std::string("Data produce End" );
}

void Data_Consumer(int iID) {
  printf("\033[1;36m[%s][%d] :x: I'm worker No [%d] \033[m\n",
      __FUNCTION__,__LINE__,iID);
  std::unique_lock<std::mutex> lkWorker(g_mtxData);
  std::vector<std::string> vecItems;
  while (g_bFlagWorker) {
    //g_cvData.wait(lkWorker,[]{return g_bProduceData;});
    g_cvData.wait(lkWorker);
    printf("\033[1;36m[%s][%d] :x: ID [%d] Consume %s\033[m\n",
        __FUNCTION__,__LINE__,iID,g_strProducedData.c_str());
    vecItems.push_back(g_strProducedData);
  }
  printf("\033[1;33m[%s][%d] :x: Worker [%d] Done \033[m\n",
      __FUNCTION__,__LINE__,iID);
  printf("\033[1;32m[%s][%d] :x: Print out consumed item of worker [%d] "
      "\033[m\n", __FUNCTION__,__LINE__,iID);
  // print out the data received
  for (auto item : vecItems) {
    printf("\033[1;36m[%s][%d] :x: Worker[%d]  [%s] \033[m\n",
        __FUNCTION__,__LINE__,iID,item.c_str());
  }
  
}

int main(int argc, char *argv[]) {
  printf(":x: Project Name : WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");

  // Create Producer
  std::thread thrProducer(Data_Producer);
  g_bFlagWorker = true;

  // Create 10 Workers
  std::vector<std::thread*> vecWorkers;
  for (int i =0 ; i< 10 ; i++ ) {
    vecWorkers.push_back( new std::thread(Data_Consumer, i));
  }

  thrProducer.join();
  g_bFlagWorker = false;
  g_cvData.notify_all();
  printf("\033[1;33m[%s][%d] :x: Notify All \033[m\n",__FUNCTION__,__LINE__);
  int cnt=0;
  for ( auto Worker : vecWorkers) {

    printf("\033[1;32m[%s][%d] :x: worker join %d \033[m\n",__FUNCTION__,__LINE__,cnt);
    cnt++;
    Worker->join();
    delete Worker;
  }
  return 0;
}
