#include "thermal_nodelet.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace thermal {

void IRNodelet::onInit() {

  NODELET_INFO("Initializing nodelet");
  ros::NodeHandle nh = getNodeHandle();
  inst_.reset(new IRCapture(nh, getPrivateNodeHandle()));
  monitor_ = nh.createTimer(ros::Duration(0.5), boost::bind(&IRCapture::Monitor, inst_, _1));
  pubThread_.reset(new boost::thread(boost::bind(&IRCapture::Run, inst_)));
}

} // namespace thermal
PLUGINLIB_EXPORT_CLASS(thermal::IRNodelet, nodelet::Nodelet)
