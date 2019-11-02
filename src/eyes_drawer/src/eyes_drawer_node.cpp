#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <algorithm>

tf::Vector3 GetPointInDisplayCoords(const tf::Vector3& display_pos, const tf::Vector3& display_norm,
                                    const tf::Vector3& eye_origin, const tf::Vector3& point_of_interest) {
  const tf::Vector3 or_minus_pi = eye_origin - point_of_interest;
  const tfScalar d_nom = (eye_origin - display_pos).dot(display_norm);
  const tfScalar d_den = or_minus_pi.dot(display_norm);

  // if observed object is out of sight, just drop back to normal position
  if (std::abs(d_den) < 1e-6) {
    return display_pos;
  }

  return eye_origin - or_minus_pi * (d_nom / d_den);
}


class EyeDrawer {
 public:
  EyeDrawer(ros::NodeHandle& nh,
            image_transport::ImageTransport& it,
            const std::string& eye_name,
            const std::string& eye_origin_frame,
            const std::string& display_origin_frame,
            const std::string& poi_frame,
            double meters_per_pixel,
            int img_w, int img_h)
  : nh_(nh)
  , it_(it)
  , eye_origin_frame_(eye_origin_frame)
  , display_origin_frame_(display_origin_frame)
  , poi_frame_(poi_frame)
  , meters_per_pixel_(meters_per_pixel)
  , img_w_(img_w)
  , img_h_(img_h) {

    dot_pos_ = nh_.advertise<geometry_msgs::Vector3>(std::string("eyes_pos/") + eye_name, 10);
    img_pub_ = it_.advertise(std::string("eyes/") + eye_name, 1);
  }

  void redraw() {
    tf::StampedTransform poi_to_display;

    // we have bunch of static transforms, so we can deside to make display origin as
    // origin for all our translations, so final point will be in display coordinates
    listener_.lookupTransform(display_origin_frame_, eye_origin_frame_, ros::Time(0), eye_to_display_);

    listener_.lookupTransform(display_origin_frame_, poi_frame_, ros::Time(0), poi_to_display);

    tf::Vector3 result = GetPointInDisplayCoords(tf::Vector3(), tf::Vector3(0, 0, 1),
                          eye_to_display_.getOrigin(), poi_to_display.getOrigin());

    geometry_msgs::Vector3 msg;
    msg.x = result.x();
    msg.y = result.y();
    msg.z = result.z();

    dot_pos_.publish(msg);

    draw_send_point(result.x(), result.y());
  }

  void draw_send_point(double x_orig, double y_orig) {
    cv::Mat img = cv::Mat::zeros(img_h_, img_w_, CV_8UC3);

    int x = img_w_ / 2 + static_cast<int>(x_orig / meters_per_pixel_);
    int y = img_h_ / 2 + static_cast<int>(y_orig / meters_per_pixel_);

    if (x < 0) {
      x = 0;
    }

    if (x >= img_w_) {
      x = img_w_ - 1;
    }

    if (y < 0) {
      y = 0;
    }

    if (y >= img_h_) {
      y = img_h_ - 1;
    }

    cv::circle(img, cv::Point(x, y), 20, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    img_pub_.publish(msg);
  }

 private:
  ros::NodeHandle& nh_;
  image_transport::ImageTransport& it_;

  ros::Publisher dot_pos_;
  image_transport::Publisher img_pub_;

  tf::StampedTransform eye_to_display_;
  tf::TransformListener listener_;

  std::string eye_origin_frame_;
  std::string display_origin_frame_;
  std::string poi_frame_;

  double meters_per_pixel_;
  int img_w_, img_h_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "eyes_drawer");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  int img_width, img_height;
  if (!nh.getParam("img_width", img_width) || !nh.getParam("img_height", img_height)) {
    ROS_FATAL("can't get image size parameters");
    return 1;
  }

  EyeDrawer left_drawer(nh, it, "left", "/left_origin", "/left_display", "/point_of_interest", 0.0002, img_width, img_height);
  EyeDrawer right_drawer(nh, it, "right", "/right_origin", "/right_display", "/point_of_interest", 0.0002, img_width, img_height);

  ros::Rate loop_rate(20);

  while (nh.ok()) {
    try {
      left_drawer.redraw();
      right_drawer.redraw();
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    /* pub_left.publish(msg_left); */
    /* pub_right.publish(msg_right); */

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
