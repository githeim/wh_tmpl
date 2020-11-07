#include "CMainDialog.h"

#include <QCoreApplication>

CMainDialog::CMainDialog(QWidget *parent)
{
  // :x: Create Grid Layout and put the buttons on the grid
  m_pLayoutGrid = new QGridLayout;

  m_pBtnA    = new QPushButton("Button A",this);
  connect(m_pBtnA,    SIGNAL (released()), this, SLOT (handleBtnA()));
  m_pBtnB    = new QPushButton("Button B",this);
  connect(m_pBtnB,    SIGNAL (released()), this, SLOT (handleBtnB()));
  m_pBtnC    = new QPushButton("Button C",this);
  connect(m_pBtnC,    SIGNAL (released()), this, SLOT (handleBtnC()));
  m_pBtnD    = new QPushButton("Button D",this);
  connect(m_pBtnD,    SIGNAL (released()), this, SLOT (handleBtnD()));
  m_pBtnExit = new QPushButton("Exit",this);
  connect(m_pBtnExit, SIGNAL (released()), this, SLOT (handleBtnExit()));

  m_pTxtEdit = new QTextEdit("blabla",this);

  qRegisterMetaType<std::string>("std::string");
  connect(this,SIGNAL(valueCh(std::string)), this, SLOT(setValue(std::string)));

  m_pLabel00 = new QLabel("Start",this);
  m_pLabel00->setFont(QFont("",20));
  
  m_pLayoutGrid->addWidget(m_pBtnA    ,0,0);
  m_pLayoutGrid->addWidget(m_pBtnB    ,0,1);
  m_pLayoutGrid->addWidget(m_pLabel00 ,0,2);
  m_pLayoutGrid->addWidget(m_pBtnC    ,1,0);
  m_pLayoutGrid->addWidget(m_pBtnD    ,1,1);
  m_pLayoutGrid->addWidget(m_pBtnExit ,2,0);
  m_pLayoutGrid->addWidget(m_pTxtEdit ,2,1);

  m_pLayoutGrid->setColumnStretch(1, 10);
  m_pLayoutGrid->setColumnStretch(2, 20);

  setLayout(m_pLayoutGrid);
}

CMainDialog::~CMainDialog(){
  delete m_pLayoutGrid;
  delete m_pBtnA;
  delete m_pBtnB;
  delete m_pBtnC;
  delete m_pBtnD;
  delete m_pBtnExit;
  delete m_pLabel00;
  printf("\033[1;33m[%s][%d] :x: Clean out the pointers \033[m\n",
      __FUNCTION__,__LINE__);

}

void CMainDialog::handleBtnA(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button A");
}
void CMainDialog::handleBtnB(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button B");
}
void CMainDialog::handleBtnC(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button C");
}
void CMainDialog::handleBtnD(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button D");
}
void CMainDialog::handleBtnExit(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  QWidget::close();
}
void CMainDialog::setValue(std::string strVal) {
  m_pTxtEdit->setText(strVal.c_str());
  printf("\033[1;33m[%s][%d] :x: Value[%s] \033[m\n",
      __FUNCTION__,__LINE__,strVal.c_str());
}

void CMainDialog::Send_msg(std::string strMsg) {
  emit valueCh(strMsg);
}
