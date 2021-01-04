#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr &image_left, const ImageConstPtr &image_right,
              const ImageConstPtr &thermal_left,
              const ImageConstPtr &thermal_right,
              const PointCloud2ConstPtr &pt) {
  ROS_INFO("message filter callback");
  ROS_INFO("image left: %d", image_left->header.stamp.nsec);
  ROS_INFO("image right: %d", image_right->header.stamp.nsec);
  ROS_INFO("thermal left: %d", thermal_left->header.stamp.nsec);
  ROS_INFO("thermal right: %d", thermal_right->header.stamp.nsec);
  ROS_INFO("pointcloud: %d", pt->header.stamp.nsec);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_left_sub(
      nh, "/camera/left/image_raw", 1);
  message_filters::Subscriber<Image> image_right_sub(
      nh, "/camera/right/image_raw", 1);
  message_filters::Subscriber<Image> thermal_left_sub(
      nh, "/thermal/left/image_raw", 1);
  message_filters::Subscriber<Image> thermal_right_sub(
      nh, "/thermal/right/image_raw", 1);
  message_filters::Subscriber<PointCloud2> lidar_sub(nh, "/rslidar_points", 1);
  typedef sync_policies::ApproximateTime<Image, Image, Image, Image,
                                         PointCloud2>
      MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_left_sub,
                                  image_right_sub, thermal_left_sub,
                                  thermal_right_sub, lidar_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();

  return 0;
}