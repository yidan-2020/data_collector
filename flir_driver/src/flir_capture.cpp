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
      for (unsigned int i = 0; i < camera_nums; i++) {
        // Select camera
        pCam = cam_list.GetByIndex(i);
        ImagePtr image = pCam->GetNextImage(100);
        if (image->IsIncomplete()) {

          ROS_WARN_STREAM("Image incomplete with image status "
                          << image->GetImageStatus() << "!");

        } else {
          Mat mat_img = ConverttoMat(image);
          PublishImage(mat_img, cam_ids[i]);
        }
      }
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

  for (unsigned int i = 0; i < camera_nums; i++) {
    img_msg_header.frame_id = "cam_" + to_string(i);

    sensor_msgs::ImagePtr img_msgs =
        cv_bridge::CvImage(img_msg_header, "bgr8", img).toImageMsg();
    if (id.compare(left_id)) {
      camera_image_left_pub.publish(img_msgs);
    } else if (id.compare(right_id)) {
      camera_image_right_pub.publish(img_msgs);
    }
  }
}

} // namespace flir
