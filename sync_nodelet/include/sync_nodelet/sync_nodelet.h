#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

namespace syncfilter {
class SyncNodelet : public nodelet::Nodelet {

public:
  SyncNodelet() {}
  ~SyncNodelet() {}
  virtual void onInit();
  void callback(const ImageConstPtr &image_left,
               const ImageConstPtr &image_right);

private:
  ros::Publisher pub;
};
} // namespace syncfilter
