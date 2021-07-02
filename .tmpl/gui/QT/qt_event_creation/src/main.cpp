#include "CMainDialog.h"
#include <QApplication>
#include <QDesktopWidget>

#include <unistd.h>
void Test_PostEvt(QObject* pObj) {

  std::vector<QInputEvent*> vecInputEvt = {
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Enter,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Tab,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Enter,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Tab,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Enter,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Tab,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Enter,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Tab,Qt::NoModifier),
    new QKeyEvent( QEvent::Type::KeyPress,Qt::Key_Enter,Qt::NoModifier)
  };
  int iCnt = 0;
  for (auto evt : vecInputEvt) {
    iCnt++;
    printf("\033[1;33m[%s][%d] :x: chk [%d]\033[m\n",__FUNCTION__,__LINE__,iCnt);
    sleep(1);
    QCoreApplication::postEvent(pObj,evt);
  }
}

int main(int argc, char *argv[]) {
  int ret;
  std::string strDialogTitle(
      "WW_ProjectName_WW ver WW_MajorVer_WW.WW_MinorVer_WW");
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

  std::thread pThrTest(Test_PostEvt,(QObject*)&mainDialog);
  
  ret = app.exec();

  pThrTest.join();
  return ret;
}
