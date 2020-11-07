#include "CMainDialog.h"
#include <QApplication>
#include <QDesktopWidget>
#include <thread>
#include <unistd.h>

int g_bThreadFlag = true;
int Loop(CMainDialog* pDlg) {
  sleep(1);
  int cnt =0;
  std::string strMsg="";
  while(g_bThreadFlag)
  {
    printf("\033[1;36m[%s][%d] :x: chk \033[m\n",__FUNCTION__,__LINE__);
    strMsg = "Message : ["+ std::to_string(cnt)+"]";

    pDlg->Send_msg(strMsg);
    cnt++;
   
    sleep (1);
  }
  return 0;
}
int main(int argc, char *argv[]) {
  std::string strDialogTitle(
      "QT_button_ex ver 121212.343456");
  QApplication app(argc, argv);

  QRect rec = QApplication::desktop()->screenGeometry();
  int iWidth = rec.width()/6;
  int iHeight = rec.height()/4;
  printf("\033[1;36m[%s][%d] :x: width ; %d height ; %d \033[m\n",
      __FUNCTION__,__LINE__,iWidth,iHeight);

  QSize winSize( iWidth, iHeight );
  CMainDialog mainDialog;
  mainDialog.setWindowTitle(strDialogTitle.c_str());
  mainDialog.resize(winSize);
  mainDialog.showNormal();
  std::thread thr(Loop,&mainDialog);
  app.exec();
  g_bThreadFlag = false;
  thr.join();
  return 0;
}
