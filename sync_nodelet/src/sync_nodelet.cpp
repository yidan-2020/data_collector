#include "sync_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <string>

namespace syncfilter {

void SyncNodelet::callback(const ImageConstPtr &image_left,
                           const ImageConstPtr &image_right) {
  NODELET_INFO("left right callback");
}
void SyncNodelet::onInit() {
  NODELET_INFO("Initializing sync nodelet");
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle private_nh = getPrivateNodeHandle();
  std::string left_topic, right_topic;
  ROS_ASSERT_MSG(private_nh.getParam("left_name", left_topic),
                 "left topic name is required!");
  ROS_ASSERT_MSG(private_nh.getParam("right_name", right_topic),
                 "right topic name is required!");
  message_filters::Subscriber<Image> image_left_sub(
      nh, "/camera/left/image_raw", 1);
  message_filters::Subscriber<Image> image_right_sub(
      nh, "/camera/right/image_raw", 1);
  TimeSynchronizer<Image, Image> sync(image_left_sub, image_right_sub,
                                      10); // exact time
  sync.registerCallback(boost::bind(&SyncNodelet::callback, this, _1, _2));
  ros::spin();
}

} // namespace syncfilter
PLUGINLIB_EXPORT_CLASS(syncfilter::SyncNodelet, nodelet::Nodelet)