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

  m_pGraphicsView = new QGraphicsView(this);
  m_pGraphicsScene = new QGraphicsScene(this);
  m_pGraphicsView->setScene(m_pGraphicsScene);
  m_pGraphicsView->setFixedSize(100,100);
  QBrush blueBrush(Qt::blue);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);

  m_pEllipse = m_pGraphicsScene->addEllipse(10, 10, 50, 50, outlinePen, blueBrush);

  m_pLabel00 = new QLabel("Start",this);
  m_pLabel00->setFont(QFont("",20));
  
  m_pLayoutGrid->addWidget(m_pBtnA    ,0,0);
  m_pLayoutGrid->addWidget(m_pBtnB    ,0,1);
  m_pLayoutGrid->addWidget(m_pLabel00 ,0,2);
  m_pLayoutGrid->addWidget(m_pBtnC    ,1,0);
  m_pLayoutGrid->addWidget(m_pBtnD    ,1,1);
  m_pLayoutGrid->addWidget(m_pGraphicsView,1,2);
  m_pLayoutGrid->addWidget(m_pBtnExit ,2,0);

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

  delete m_pGraphicsView;
  delete m_pGraphicsScene;

  printf("\033[1;33m[%s][%d] :x: Clean out the pointers \033[m\n",
      __FUNCTION__,__LINE__);

}

void CMainDialog::handleBtnA(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button A");
  QBrush grayBrush(Qt::gray);
  m_pEllipse->setBrush(grayBrush);
}
void CMainDialog::handleBtnB(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button B");
  QBrush blueBrush(Qt::blue);
  m_pEllipse->setBrush(blueBrush);
}
void CMainDialog::handleBtnC(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button C");

  QBrush greenBrush(Qt::green);
  m_pEllipse->setBrush(greenBrush);
}
void CMainDialog::handleBtnD(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button D");
  QBrush redBrush(Qt::red);
  m_pEllipse->setBrush(redBrush);
}
void CMainDialog::handleBtnExit(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  QWidget::close();
}
