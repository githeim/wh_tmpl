#ifndef _CMAINDIALOG_H_
#define _CMAINDIALOG_H_

#include <QtWidgets>
#include <QPushButton>
#include <QGridLayout>


#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QPieSeries>
#include <QtCharts/QPieSlice>
#include <QtCharts/QScatterSeries>

// :x: for bar chart
#include <QtCharts/QStackedBarSeries>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>


#include <QtCharts/QPolarChart>
#include <QtCharts/QValueAxis>

#include <QtCore/QDebug>
QT_CHARTS_USE_NAMESPACE




class CMainDialog : public QDialog
{
  Q_OBJECT
public:
  CMainDialog(QWidget *parent = 0);
  ~CMainDialog();
private slots:
  // :x: button handlers should be in the 'slots'
  void handleBtnA();
  void handleBtnExit();
private:
  QChart*      CreateLineChart() const;
  QChart*      CreateSplineChart() const;
  QChart*      CreateScatterChart() const;
  QChart*      CreateBarChart() const;
  QChart*      CreatePieChart() const;
  QChart*      CreatePolarChart() const;

  // :x: Grid Layout
  QGridLayout *m_pLayoutGrid;
  // :x: Buttons
  QPushButton *m_pBtnA;

  QPushButton *m_pBtnExit;
  QLabel      *m_pLabel00;

};

#endif // _CMAINDIALOG_H_
