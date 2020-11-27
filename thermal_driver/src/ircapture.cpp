#include "ircapture.h"

Frame *const frame0 = new Frame;
Frame *const frame1 = new Frame;

int FrameCallback0(void *lData, void *lParam) {
  Frame *tmp = (Frame *)lData;
  if (tmp == NULL) {
    return -1;
  }
  memcpy(frame0, tmp, sizeof(Frame));
  return 0;
}

int FrameCallback1(void *lData, void *lParam) {
  Frame *tmp = (Frame *)lData;
  if (tmp == NULL) {
    return -1;
  }
  memcpy(frame1, tmp, sizeof(Frame));
  return 0;
}

IRCapture::IRCapture(ros::NodeHandle node, ros::NodeHandle private_nh)
    : nh_(node), it_(nh_), nh_pvt_(private_nh) {
  ROS_ASSERT_MSG(nh_pvt_.getParam("left_ip", left_ip), "left ip is required");
  ROS_ASSERT_MSG(nh_pvt_.getParam("right_ip", right_ip),
                 "right ip is required");
  ROS_ASSERT_MSG(nh_pvt_.getParam("min_tempreture", min_t),
                 "min tempreture is required");
  ROS_ASSERT_MSG(nh_pvt_.getParam("max_tempreture", max_t),
                 "max tempreture is required");
  IRSDK_Init();
  memset(ip_info, 0, sizeof(ip_info)); // clear

  callback_frame0 = &FrameCallback0;
  callback_frame1 = &FrameCallback0;

  for (int i = 0; i < 2; i++) {
    frames_.push_back(cv::Mat::zeros(512, 640, CV_8UC1));
  }
  camera_image_pubs_left = it_.advertise("thermal/left/image_raw", 1);
  camera_image_pubs_right = it_.advertise("thermal/right/image_raw", 1);
}

IRCapture::~IRCapture() {

  IRSDK_Quit();
  ros::shutdown();
}

void IRCapture::Monitor(const ros::TimerEvent &event) {
  if (created_) {
    if (IRSDK_IsConnected(0) == 0) {
      ROS_ERROR("not connected 0");
      IRSDK_Connect(0);
    }
    if (IRSDK_IsConnected(1) == 0) {
      ROS_ERROR("not connected 1");
      IRSDK_Connect(1);
    }
  }
  IRSDK_InqureIP(ip_info, 500);
  ROS_ERROR("monitoring");
}

void IRCapture::Run() {
  ros::Rate ros_rate(10);
  try {
    while (ros::ok()) {
      if (!created_) {
        if (ip_info[0].totalOnline < 2) {
          continue;
        }
        std::cout << "0: " << ip_info[0].IPAddr << std::endl;
        std::cout << "1: " << ip_info[1].IPAddr << std::endl;
        IRSDK_Create(0, ip_info[0], callback_frame0, NULL, NULL,
                     (void *)(this));
        IRSDK_Create(1, ip_info[1], callback_frame1, NULL, NULL,
                     (void *)(this));
        IRSDK_Connect(0);
        IRSDK_Connect(1);
        created_ = true;
      } else {
        if (float(frame0->u8TempDiv) > 0 && float(frame1->u8TempDiv) > 0) {
          GetTempretureImage(frame0->buffer, float(frame0->u8TempDiv),
                             frames_[0]);
          GetTempretureImage(frame1->buffer, float(frame1->u8TempDiv),
                             frames_[1]);
        }
      }
      ExporttoROS();
      ros_rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Excption: " << e.what());
  } catch (...) {
    ROS_FATAL_STREAM("Some unknown exception occured.");
  }
}

void IRCapture::GetTempretureImage(unsigned short *buffer, float x,
                                   cv::Mat &img) {
  if (x < 1) {
    ROS_ERROR("div x is too small, use 100 instead");
    x = 100;
  }
  for (int i = 0; i < img.rows; i++) {
    uchar *data = img.ptr(i);
    for (int j = 0; j < img.cols; j++) {
      double temperature = (buffer[i * 512 + j] - 10000) / x;
      // std::cout<<"tep: " << temperature << std::endl;
      double value = (temperature - min_t) / (max_t - min_t);
      data[j] = static_cast<uchar>(std::max(std::min(1.0, value), 0.0) * 255);
    }
  }
}

void IRCapture::ExporttoROS() {
  std_msgs::Header img_msg_header;
  img_msg_header.stamp = ros::Time::now();
  for (int i = 0; i < 2; i++) {
    img_msg_header.frame_id = "thermal_frame_" + std::to_string(i);
    sensor_msgs::ImagePtr img_msgs =
        cv_bridge::CvImage(img_msg_header, "mono8", frames_[i]).toImageMsg();
    if (i < 1) {
      camera_image_pubs_left.publish(img_msgs);
    } else {
      camera_image_pubs_right.publish(img_msgs);
    }
  }
  ROS_INFO("published");
}