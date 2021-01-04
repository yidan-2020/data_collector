#include "flir_driver/flir_capture.h"

namespace flir {
FlirCapture::~FlirCapture() {
  // Clear camera list before releasing system_
  cam_list.Clear();
  // Release system_
  system_->ReleaseInstance();
}

FlirCapture::FlirCapture(ros::NodeHandle nodehandl, ros::NodeHandle private_nh)
    : nh_(nodehandl), it_(nh_), nh_pvt_(private_nh) {
  ROS_INFO_STREAM("Creating system_ instance...");
  system_ = System::GetInstance();
  camera_image_left_pub = it_.advertise("camera/left/image_raw", 1);
  camera_image_right_pub = it_.advertise("camera/right/image_raw", 1);

  cam_list = system_->GetCameras();

  camera_nums = cam_list.GetSize();

  cout << "Number of cameras detected: " << camera_nums << endl << endl;
  for (unsigned int i = 0; i < camera_nums; i++) {
    string id = string(cam_list.GetByIndex(i)->GetUniqueID());
    cam_ids.push_back(id);
    cout << "ID: " << id << endl;
  }
  ROS_ASSERT_MSG(nh_pvt_.getParam("left_cam", left_id), "left id is required!");
  ROS_ASSERT_MSG(nh_pvt_.getParam("right_cam", right_id),
                 "right id is required!");
}

void FlirCapture::Init() {
  ROS_INFO("*** INITIALIZING ***");
  try {
    for (unsigned int i = 0; i < camera_nums; i++) {
      cam_list[i]->Init();
    }
  } catch (Spinnaker::Exception &e) {
    string error_msg = e.what();
    ROS_FATAL_STREAM("Error: " << error_msg);
    ros::shutdown();
  }
}

void FlirCapture::Run() {
  // Finish if there are no cameras
  if (camera_nums == 0) {
    // Clear camera list before releasing system_
    cam_list.Clear();
    // Release system_
    system_->ReleaseInstance();

    cout << "Not enough cameras!" << endl;
    cout << "Done! Press Enter to exit..." << endl;
    getchar();
    ros::shutdown();
  }

  ROS_INFO("*** ACQUISITION ***");
  try {
    for (unsigned int i = 0; i < camera_nums; i++) {
      cam_list[i]->BeginAcquisition();
    }
  } catch (Spinnaker::Exception &e) {
    string error_msg = e.what();
    ROS_FATAL_STREAM("Error: " << error_msg);
    ros::shutdown();
  }

  int count = 0;

  ros::Rate ros_rate(10);
  try {
    while (ros::ok()) {

      double t = ros::Time::now().toSec();
      CameraPtr pCam = nullptr;
      if (camera_nums != 2) {
        ROS_FATAL_STREAM("Error: "
                         << "must have 2 cameras");
        ros::shutdown();
      }
      pCam = cam_list.GetByIndex(0);
      ImagePtr image_0 = pCam->GetNextImage(200);
      pCam = cam_list.GetByIndex(1);
      ImagePtr image_1 = pCam->GetNextImage(200);
      if (image_0->IsIncomplete()) {

        ROS_WARN_STREAM("Image0 incomplete with image status "
                        << image_0->GetImageStatus() << "!");
      }
      if (image_1->IsIncomplete()) {

        ROS_WARN_STREAM("Image1 incomplete with image status "
                        << image_1->GetImageStatus() << "!");
      }
      Mat mat_img0 = ConverttoMat(image_0);
      Mat mat_img1 = ConverttoMat(image_1);
      PublishBinoImage(mat_img0, mat_img1);
      ros_rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Excption: " << e.what());
  } catch (...) {
    ROS_FATAL_STREAM("Some unknown exception occured. \v Exiting gracefully, "
                     "\n  possible reason could be Camera Disconnection...");
  }

  for (unsigned int i = 0; i < camera_nums; i++) {
    cam_list[i]->EndAcquisition();
    cam_list[i]->DeInit();
  }
}

Mat FlirCapture::ConverttoMat(ImagePtr pImage) {
  ImagePtr convertedImage;
  convertedImage = pImage->Convert(PixelFormat_BGR8); //, NEAREST_NEIGHBOR);
  unsigned int XPadding = convertedImage->GetXPadding();
  unsigned int YPadding = convertedImage->GetYPadding();
  unsigned int rowsize = convertedImage->GetWidth();
  unsigned int colsize = convertedImage->GetHeight();
  // image data contains padding. When allocating Mat container size, you need
  // to account for the X,Y image data padding.
  cv::Mat img;
  img = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3,
                convertedImage->GetData(), convertedImage->GetStride());
  return img.clone();
}

void FlirCapture::PublishImage(cv::Mat &img, string id) {
  std_msgs::Header img_msg_header;
  img_msg_header.stamp = ros::Time::now();
  img_msg_header.frame_id = "cam_" + id;
  sensor_msgs::ImagePtr img_msgs =
      cv_bridge::CvImage(img_msg_header, "bgr8", img).toImageMsg();
  if (!id.compare(left_id)) {
    camera_image_left_pub.publish(img_msgs);
  } else if (!id.compare(right_id)) {
    camera_image_right_pub.publish(img_msgs);
  }
}

void FlirCapture::PublishBinoImage(cv::Mat &img0, cv::Mat &img1) {
  bino_msgs::bino_camera msg;
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  sensor_msgs::ImagePtr img_left_msgs;
  sensor_msgs::ImagePtr img_right_msgs;
  for (int i = 0; i < 2; i++) {
    string id = cam_ids[i];
    if (!id.compare(left_id)) {
      msg_header.frame_id = "cam_left";
      img_left_msgs = cv_bridge::CvImage(msg_header, "bgr8", img0).toImageMsg();
      camera_image_left_pub.publish(img_left_msgs);
    } else if (!id.compare(right_id)) {
      msg_header.frame_id = "cam_right";
      img_right_msgs = cv_bridge::CvImage(msg_header, "bgr8", img1).toImageMsg();
      camera_image_right_pub.publish(img_right_msgs);
    }
  }
}

} // namespace flir
