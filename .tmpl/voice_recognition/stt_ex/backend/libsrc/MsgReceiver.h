#ifndef _MSGRECEIVER_H_
#define _MSGRECEIVER_H_ 
#include <vector>
#include <string>
#include <mutex>

void StartMsgReceiver(std::vector<std::string> &vecMsg,std::mutex &mtxSTT_Msg); 
void StopMsgReceiver();

#endif /* ifndef _MSGRECEIVER_H_ */
