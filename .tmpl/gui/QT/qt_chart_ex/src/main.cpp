#include "CMainDialog.h"
#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[]) {
  std::string strDialogTitle(
      "WW_ProjectName_WW ver WW_MajorVer_WW.WW_MinorVer_WW");
  QApplication app(argc, argv);

  QRect rec = QApplication::desktop()->screenGeometry();
  int iWidth = rec.width()/2;
  int iHeight = rec.height();

  QSize winSize( iWidth, iHeight );
  CMainDialog mainDialog;
  mainDialog.setWindowTitle(strDialogTitle.c_str());
  mainDialog.resize(winSize);
  mainDialog.showNormal();

  return app.exec();
}
