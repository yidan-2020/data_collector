
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// OpenCV
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;
namespace flir {
class FlirCapture {

public:
  ~FlirCapture();
  FlirCapture(ros::NodeHandle node, ros::NodeHandle private_nh);
  void Init();

  void Run();

private:
  Mat ConverttoMat(ImagePtr pImage);
  void PublishImage(cv::Mat &img, string id);

private:
  SystemPtr system_;
  CameraList cam_list;
  vector<string>cam_ids;
  string left_id, right_id;
  size_t camera_nums;
  // ros variables
  ros::NodeHandle nh_;
  ros::NodeHandle nh_pvt_;
  // image_transport::ImageTransport it_;
  image_transport::ImageTransport it_;
  image_transport::Publisher camera_image_left_pub;
  image_transport::Publisher camera_image_right_pub;
};

} // namespace flir
