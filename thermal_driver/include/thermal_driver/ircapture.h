#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
// ROS
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "IRSDK.h"

class IRCapture {

public:
  IRCapture(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~IRCapture();
  void Run();
  void Monitor(const ros::TimerEvent &event);

  friend int FrameCallback(void *lData, void *lParam);
  friend int FrameCallback_r(void *lData, void *lParam);

private:
  void GetTempretureImage();

private:
  // cam variables
  bool created_;
  bool publish_raw = false;
  std::string left_ip, right_ip;
  T_IPADDR ip_info[2];
  Frame frame_left;
  Frame frame_right;
  unsigned short pGray_left[640 * 480];
  unsigned short pGray_right[640 * 480];
  // ros variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_pvt_;
  image_transport::ImageTransport it_;

  image_transport::Publisher thermal_right_pub =
      it_.advertise("thermal/right/image_raw", 10);
  image_transport::Publisher thermal_left_pub =
      it_.advertise("thermal/left/image_raw", 10);
  image_transport::Publisher raw_right_pub =
      it_.advertise("raw/right/image_raw", 10);
  image_transport::Publisher raw_left_pub =
      it_.advertise("raw/left/image_raw", 10);
};
