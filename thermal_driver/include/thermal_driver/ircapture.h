#include <boost/thread.hpp>
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


class IRCamera {
  public:
  IRCamera();
  ~IRCamera();
  friend int FrameCallback0(void* lData, void* lParam);
  friend int FrameCallback1(void* lData, void* lParam);
  int FrameRecv0(Frame *tmp, void *lParam);
  int FrameRecv1(Frame *tmp, void *lParam);
  CBF_IR cbf_frame0;
  CBF_IR cbf_frame1;
  Frame* frame0;
  Frame* frame1;
};

class IRCapture {

public:
  IRCapture(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~IRCapture();
  void Run();
  void Monitor(const ros::TimerEvent &event);
  
  CBF_IR callback_frame0;
  CBF_IR callback_frame1;

private: 
  void GetTempretureImage(unsigned short *buffer, float x, cv::Mat &img);
  void ExporttoROS();

private:
  // cam variables
  std::vector<cv::Mat> frames_;
  double min_t, max_t;
  std::string left_ip, right_ip;
  T_IPADDR ip_info[2];
  bool created_ = false;
  // ros variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_pvt_;
  image_transport::ImageTransport it_;
  image_transport::Publisher camera_image_pubs_left;
  image_transport::Publisher camera_image_pubs_right;
};
