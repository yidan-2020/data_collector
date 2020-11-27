#include "flir_nodelet.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(flir::FlirNodelet, nodelet::Nodelet)

namespace flir {

void FlirNodelet::onInit() {

  NODELET_INFO("Initializing nodelet");

  inst_.reset(new FlirCapture(getNodeHandle(), getPrivateNodeHandle()));
  inst_->Init();
  pubThread_.reset(new boost::thread(boost::bind(&FlirCapture::Run, inst_)));
}

} // namespace flir


