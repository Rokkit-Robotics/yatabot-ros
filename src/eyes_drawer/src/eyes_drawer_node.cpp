#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "eyes_drawer");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left = it.advertise("eyes/left", 1);
  image_transport::Publisher pub_right = it.advertise("eyes/right", 1);

  std::string img_filename;
  if (!nh.getParam("test_image", img_filename)) {
    ROS_FATAL("can't get test image filename param");
    return 1;
  }

  int img_width, img_height;
  if (!nh.getParam("img_width", img_width) || !nh.getParam("img_height", img_height)) {
    ROS_FATAL("can't get image size parameters");
    return 1;
  }

  cv::Mat demo_img = cv::imread(img_filename.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::Mat demo_img_left;

  cv::resize(demo_img, demo_img_left, cv::Size(img_width, img_height));

  cv::Mat demo_img_right;
  cv::flip(demo_img_left, demo_img_right, 1);

  sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", demo_img_left).toImageMsg();
  sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", demo_img_right).toImageMsg();

  ros::Rate loop_rate(20);

  while (nh.ok()) {
    pub_left.publish(msg_left);
    pub_right.publish(msg_right);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
