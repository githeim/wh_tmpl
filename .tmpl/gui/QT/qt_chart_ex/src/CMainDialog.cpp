#include "CMainDialog.h"

#include <QCoreApplication>

#include <QtCharts/QAreaSeries>

CMainDialog::CMainDialog(QWidget *parent)
{
  QPalette pal = window()->palette();
  // :x:  Set Palette to dark gray
  pal.setColor(QPalette::Window, QRgb(0x40434a));
  pal.setColor(QPalette::WindowText, QRgb(0xd6d6d6));
  window()->setPalette(pal);

  // :x: Create Grid Layout and put the buttons on the grid
  m_pLayoutGrid = new QGridLayout;

  QChartView *pChartView;
  // :x: create line chart
  pChartView = new QChartView(CreateLineChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  0, 0);
  
  // :x: create spline chart
  pChartView = new QChartView(CreateSplineChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  0, 1);

  // :x: create scatter chart
  pChartView = new QChartView(CreateScatterChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  1, 0);

  // :x: create bar chart
  pChartView = new QChartView(CreateBarChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  1, 1);

  // :x: create pie chart
  pChartView = new QChartView(CreatePieChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  2, 0);

  // :x: create polar chart
  pChartView = new QChartView(CreatePolarChart());
  pChartView->setRenderHint(QPainter::Antialiasing);
  pChartView->chart()->setTheme(QChart::ChartThemeBlueCerulean);
  m_pLayoutGrid->addWidget(pChartView,  2, 1);



  m_pBtnA    = new QPushButton("Button A",this);
  connect(m_pBtnA,    SIGNAL (released()), this, SLOT (handleBtnA()));
  m_pBtnExit = new QPushButton("Exit",this);
  connect(m_pBtnExit, SIGNAL (released()), this, SLOT (handleBtnExit()));

  m_pLabel00 = new QLabel("Start",this);
  m_pLabel00->setFont(QFont("",20));
  
  m_pLayoutGrid->addWidget(m_pLabel00 ,3,0);
  m_pLayoutGrid->addWidget(m_pBtnA    ,4,0);
  m_pLayoutGrid->addWidget(m_pBtnExit ,4,1);


  this->setLayout(m_pLayoutGrid);
}

QChart *CMainDialog::CreateLineChart() const {
  int iValueCount =10;
  int iValueMax =5;
  QChart *pChart = new QChart();
  pChart->setTitle("Line chart");
  QLineSeries* pSeries00 = new QLineSeries();
  pSeries00->append(0,4);
  pSeries00->append(4,2);
  pSeries00->append(9,5);

  QLineSeries* pSeries01 = new QLineSeries();
  pSeries01->append(0,2);
  pSeries01->append(6,4);
  pSeries01->append(9,1);

  pChart->addSeries(pSeries00);
  pChart->addSeries(pSeries01);

  pChart->createDefaultAxes();
  pChart->axes(Qt::Horizontal).first()->setRange(0, iValueCount);
  pChart->axes(Qt::Vertical).first()->setRange(0, iValueMax);

  return pChart;
}

QChart *CMainDialog::CreateSplineChart() const {
  int iValueCount =10;
  int iValueMax =5;
  QChart *pChart = new QChart();
  pChart->setTitle("Spline chart");
  QSplineSeries* pSeries00 = new QSplineSeries();
  pSeries00->append(0,4);
  pSeries00->append(4,2);
  pSeries00->append(9,5);

  QSplineSeries* pSeries01 = new QSplineSeries();
  pSeries01->append(0,2);
  pSeries01->append(6,4);
  pSeries01->append(9,1);

  pChart->addSeries(pSeries00);
  pChart->addSeries(pSeries01);

  pChart->createDefaultAxes();
  pChart->axes(Qt::Horizontal).first()->setRange(0, iValueCount);
  pChart->axes(Qt::Vertical).first()->setRange(0, iValueMax);

  return pChart;
}

QChart *CMainDialog::CreateScatterChart() const {
  int iValueCount =10;
  int iValueMax =5;
  QChart *pChart = new QChart();
  pChart->setTitle("Scatter chart");
  QScatterSeries* pSeries00 = new QScatterSeries();
  pSeries00->append(0.5, 4.7);
  pSeries00->append(  4, 4);
  pSeries00->append(  9, 3);

  QScatterSeries* pSeries01 = new QScatterSeries();
  pSeries01->append(0.5, 1);
  pSeries01->append(  6, 2);
  pSeries01->append(  9, 2.5);

  pChart->addSeries(pSeries00);
  pChart->addSeries(pSeries01);

  pChart->createDefaultAxes();
  pChart->axes(Qt::Horizontal).first()->setRange(0, iValueCount);
  pChart->axes(Qt::Vertical).first()->setRange(0, iValueMax);

  return pChart;
}

QChart *CMainDialog::CreateBarChart() const {
  int iValueCount =10;
  int iValueMax =5;
  QChart *pChart = new QChart();
  pChart->setTitle("Bar chart");
  QBarSeries* pSeries00 = new QBarSeries();
  QBarSet * pSet00 = new QBarSet("AA");
  pSet00->append(4.7);
  pSet00->append(4);
  pSet00->append(3);
  pSeries00->append(pSet00);

  QBarSet * pSet01 = new QBarSet("BB");
  pSet01->append(1);
  pSet01->append(2);
  pSet01->append(2.5);
  pSeries00->append(pSet01);

  QBarSet * pSet02 = new QBarSet("C1");
  pSet02->append(3);
  pSet02->append(4);
  pSet02->append(4.5);
  pSeries00->append(pSet02);

  pChart->addSeries(pSeries00);

  pChart->createDefaultAxes();
  pChart->axes(Qt::Horizontal).first()->setRange(0, iValueCount);
  pChart->axes(Qt::Vertical).first()->setRange(0, iValueMax);

  return pChart;
}

QChart *CMainDialog::CreatePieChart() const {
  QChart *pChart = new QChart();
  pChart->setTitle("Pie(Donut) chart");
  QPieSeries *pSeries = new QPieSeries();
  pSeries->setHoleSize(0.35);
  pSeries->append("Protein 4.2%", 4.2);
  QPieSlice *slice = pSeries->append("Fat 15.6%", 15.6);
  slice->setExploded();
  slice->setLabelVisible();
  pSeries->append("Other 23.8%", 23.8);
  pSeries->append("Carbs 56.4%", 56.4);

  pChart->addSeries(pSeries);

  pChart->createDefaultAxes();

  return pChart;
}

QChart* CMainDialog::CreatePolarChart() const {
  QChart *pPolarChart = new QPolarChart();

  float  fAngularMin = -100;
  float  fAngularMax =  100;
  float  fRadialMax  =  100;

  QScatterSeries *pSeries00 = new QScatterSeries();
  pSeries00->setName("scatter");
  for (int i = fAngularMin; i <= fAngularMax; i += 10) {
    pSeries00->append(i, (i / fRadialMax) * fRadialMax + 8.0);
  }

  QSplineSeries *pSeries01 = new QSplineSeries();
  pSeries01->setName("spline");
  for (int i = fAngularMin; i <= fAngularMax; i += 10) {
    pSeries01->append(i, (i / fRadialMax) * fRadialMax);
  }

  QValueAxis *pAngularAxis = new QValueAxis();
  // First and last ticks are co-located on 0/360 angle.
  pAngularAxis->setTickCount(9); 
  pAngularAxis->setLabelFormat("%.1f");
  pAngularAxis->setShadesVisible(true);
  pAngularAxis->setShadesBrush(QBrush(QColor(249, 249, 255)));
  ((QPolarChart*)(pPolarChart))->addAxis(pAngularAxis, 
                                         QPolarChart::PolarOrientationAngular);

  QValueAxis *pRadialAxis = new QValueAxis();
  pRadialAxis->setTickCount(9);
  pRadialAxis->setLabelFormat("%d");
  ((QPolarChart*)(pPolarChart))->addAxis(pRadialAxis, 
                                         QPolarChart::PolarOrientationRadial);

  pPolarChart->addSeries(pSeries00);
  pPolarChart->addSeries(pSeries01);

  pSeries00->attachAxis(pAngularAxis);
  pSeries00->attachAxis(pRadialAxis);

  pSeries01->attachAxis(pAngularAxis);
  pSeries01->attachAxis(pRadialAxis);

  return pPolarChart;
}
CMainDialog::~CMainDialog(){
  delete m_pLayoutGrid;
  delete m_pBtnA;
  delete m_pBtnExit;
  delete m_pLabel00;
  printf("\033[1;33m[%s][%d] :x: Clean out the pointers \033[m\n",
      __FUNCTION__,__LINE__);

}

void CMainDialog::handleBtnA(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  m_pLabel00->setText("Button A");
}

void CMainDialog::handleBtnExit(){
  printf("\033[1;33m[%s][%d] :x: Btn Event \033[m\n",__FUNCTION__,__LINE__);
  QWidget::close();
}
