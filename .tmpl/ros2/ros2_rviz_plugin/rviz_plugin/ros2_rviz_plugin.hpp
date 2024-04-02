#ifndef _ROS2_RVIZ_PLUGIN_HPP_
#define _ROS2_RVIZ_PLUGIN_HPP_ 


// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
#include <QRadioButton>

#include "rviz_common/panel.hpp"

namespace ros2_rviz_plugin {
class CROS2_rviz_plugin : public rviz_common::Panel {
  
  Q_OBJECT
  public:
  explicit CROS2_rviz_plugin(QWidget * parent = 0);
  virtual ~CROS2_rviz_plugin();

  QGridLayout *m_pLayoutGrid= nullptr;
  QPushButton * m_pButton00 = nullptr;
};

} // end namespace ros2_rviz_plugin
#endif /* ifndef _ROS2_RVIZ_PLUGIN_HPP_ */
