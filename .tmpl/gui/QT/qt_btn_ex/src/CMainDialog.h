#ifndef _CMAINDIALOG_H_
#define _CMAINDIALOG_H_

#include <QtWidgets>
#include <QPushButton>
#include <QGridLayout>

class CMainDialog : public QDialog
{
  Q_OBJECT
public:
  CMainDialog(QWidget *parent = 0);
  ~CMainDialog();
private slots:
  // :x: button handlers should be in the 'slots'
  void handleBtnA();
  void handleBtnB();
  void handleBtnC();
  void handleBtnD();
  void handleBtnExit();
private:

  // :x: Grid Layout
  QGridLayout *m_pLayoutGrid;
  // :x: Buttons
  QPushButton *m_pBtnA;
  QPushButton *m_pBtnB;
  QPushButton *m_pBtnC;
  QPushButton *m_pBtnD;

  QPushButton *m_pBtnExit;
  QLabel      *m_pLabel00;

};

#endif // _CMAINDIALOG_H_
