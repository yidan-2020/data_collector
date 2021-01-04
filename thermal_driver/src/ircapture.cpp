#include "ircapture.h"

int FrameCallback(void *lData, void *lParam) {
  Frame *tmp = (Frame *)lData;
  if (tmp == NULL) {
    return -1;
  }
  IRCapture *ir = (IRCapture *)lParam;
  ir->frame_left = *tmp;
  IRSDK_FrameConvertDDE(tmp, ir->pGray_left, 50, 50, NULL, NULL, 0, 50, 5);
  return 0;
}

int FrameCallback_r(void *lData, void *lParam) {
  Frame *tmp = (Frame *)lData;
  if (tmp == NULL) {
    return -1;
  }
  IRCapture *ir = (IRCapture *)lParam;
  ir->frame_right = *tmp;
  IRSDK_FrameConvertDDE(tmp, ir->pGray_right, 50, 50, NULL, NULL, 0, 50, 5);
  return 0;
}

IRCapture::IRCapture(ros::NodeHandle node, ros::NodeHandle private_nh)
    : nh_(node), it_(nh_), nh_pvt_(private_nh) {
  ROS_ASSERT_MSG(nh_pvt_.getParam("left_ip", left_ip), "left ip is required");
  ROS_ASSERT_MSG(nh_pvt_.getParam("right_ip", right_ip),
                 "right ip is required");
  nh_pvt_.getParam("publish_raw", publish_raw);
  IRSDK_Init();
  memset(ip_info, 0, sizeof(ip_info)); // clear
  IRSDK_InqureIP(ip_info, 500);
  created_ = false;
}

IRCapture::~IRCapture() {
  IRSDK_Destroy(0);
  IRSDK_Destroy(1);
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
}

void IRCapture::Run() {
  ros::Rate ros_rate(10);
  try {
    while (ros::ok()) {
      if (!created_) {
        {
          if (ip_info[0].totalOnline < 2) {
            ros_rate.sleep();
            continue;
          }
          std::cout << "0: " << ip_info[0].IPAddr << std::endl;
          std::cout << "1: " << ip_info[1].IPAddr << std::endl;
          T_IPADDR left_ipinfo = ip_info[0];
          T_IPADDR right_ipinfo = ip_info[1];
          if (std::string(left_ipinfo.IPAddr).compare(left_ip)) {
            ROS_ERROR("switching");
            left_ipinfo = ip_info[1];
            right_ipinfo = ip_info[0];
          }
          if (std::string(left_ipinfo.IPAddr).compare(left_ip) ||
              std::string(right_ipinfo.IPAddr).compare(right_ip)) {
            ROS_ERROR("unmatched");
            ros_rate.sleep();
            continue;
          }
          ROS_INFO("creating0===");
          IRSDK_Create(0, left_ipinfo, &FrameCallback, NULL, NULL,
                       (void *)this);
          IRSDK_Connect(0);
          ROS_INFO("creating1===");
          IRSDK_Create(1, right_ipinfo, &FrameCallback_r, NULL, NULL,
                       (void *)this);
          IRSDK_Connect(1);
          created_ = true;
          ROS_INFO("created success!");
        }

      } else {
        GetTempretureImage();
      }
      ros_rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Excption: " << e.what());
  } catch (...) {
    ROS_FATAL_STREAM("Some unknown exception occured.");
  }
  IRSDK_Destroy(0);
  IRSDK_Destroy(1);
}

void IRCapture::GetTempretureImage() {
  cv::Mat img_left = cv::Mat::zeros(480, 640, CV_8UC1);
  cv::Mat img_right = cv::Mat::zeros(480, 640, CV_8UC1);

  if (pGray_left == NULL || pGray_right == NULL) {
    return;
  }

  for (int i = 0; i < img_left.rows; i++) {
    uchar *data_l = img_left.ptr(i);
    uchar *data_r = img_right.ptr(i);
    for (int j = 0; j < img_left.cols; j++) {
      uchar gray_l = static_cast<uchar>(pGray_left[640 * i + j]);
      data_l[j] = gray_l;
      uchar gray_r = static_cast<uchar>(pGray_right[640 * i + j]);
      data_r[j] = gray_r;
    }
  }
  std_msgs::Header img_msg_header;
  img_msg_header.stamp = ros::Time::now();
  img_msg_header.frame_id = "left";
  sensor_msgs::ImagePtr img_left_msgs =
      cv_bridge::CvImage(img_msg_header, "mono8", img_left).toImageMsg();
  img_msg_header.frame_id = "right";
  sensor_msgs::ImagePtr img_right_msgs =
      cv_bridge::CvImage(img_msg_header, "mono8", img_right).toImageMsg();
  if (publish_raw) {
    cv::Mat raw_left = cv::Mat::zeros(480, 640, CV_16UC1);
    cv::Mat raw_right = cv::Mat::zeros(480, 640, CV_16UC1);
    for (int i = 0; i < raw_left.rows; i++) {
      ushort *data_l = raw_left.ptr<ushort>(i);
      ushort *data_r = raw_right.ptr<ushort>(i);
      for (int j = 0; j < raw_left.cols; j++) {
        ushort gray_l = static_cast<ushort>(frame_left.buffer[640 * i + j]);
        data_l[j] = gray_l;
        ushort gray_r = static_cast<ushort>(frame_right.buffer[640 * i + j]);
        data_r[j] = gray_r;
      }
    }
    sensor_msgs::ImagePtr raw_left_msgs =
        cv_bridge::CvImage(img_msg_header, "mono16", raw_left).toImageMsg();
    sensor_msgs::ImagePtr raw_right_msgs =
        cv_bridge::CvImage(img_msg_header, "mono16", raw_right).toImageMsg();
    raw_left_pub.publish(raw_left_msgs);
    raw_right_pub.publish(raw_right_msgs);
  }

  thermal_left_pub.publish(img_left_msgs);
  ROS_INFO("left");
  thermal_right_pub.publish(img_right_msgs);
  ROS_INFO("right");
}