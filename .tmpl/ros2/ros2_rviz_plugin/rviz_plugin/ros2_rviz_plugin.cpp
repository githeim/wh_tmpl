#include "ros2_rviz_plugin.hpp"

namespace ros2_rviz_plugin {

CROS2_rviz_plugin::CROS2_rviz_plugin(QWidget * parent) :
Panel(parent)
{
  m_pLayoutGrid = new QGridLayout;

  m_pButton00 = new QPushButton("Button A",this);

  m_pLayoutGrid->addWidget(m_pButton00);
  setLayout(m_pLayoutGrid);
}



CROS2_rviz_plugin::~CROS2_rviz_plugin()
{
  if (m_pButton00) 
    delete m_pButton00;
}



} // end namespace ros2_rviz_plugin
  
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_rviz_plugin::CROS2_rviz_plugin, rviz_common::Panel)
