#include "MsgHandler.h"


// Speech - Command mappings
std::map<std::string,std::string> g_mapMsgCommand = {
  { "start chrome", "google-chrome"},
  { "start terminal", "gnome-terminal"},
  { "start settings", "gnome-control-center"}

};

bool g_bMsgHandlerFlag = false;
bool get_bMsgHandlerFlag() {
  return g_bMsgHandlerFlag;
}
void set_bMsgHandlerFlag(bool bVal) {
  g_bMsgHandlerFlag=bVal;
}

void LoopMsgHandler(std::vector<std::string> &vecMsg,
                     std::mutex &mtxMsg
) {
  while (get_bMsgHandlerFlag()) {

  }

}

void Handle_STT_Msg(std::string strMsg) {
  system (g_mapMsgCommand[strMsg].c_str());
}


void StartMsgHandler(std::vector<std::string> &vecSTT_Msg,
                      std::mutex &mtxSTT_Msg) {
  set_bMsgHandlerFlag(true);

}
