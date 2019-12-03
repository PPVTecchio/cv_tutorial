/* COPYRIGHT http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages 2019
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW_B = "Image window_B";
static const std::string OPENCV_WINDOW_G = "Image window_G";
static const std::string OPENCV_WINDOW_R = "Image window_R";
static const std::string OPENCV_WINDOW = "Image window";

class ImageManipulator {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_b_;
  image_transport::Publisher image_pub_g_;
  image_transport::Publisher image_pub_r_;
  cv::Mat image_src_;
  cv::Mat image_rgb_[3];

 public:
  ImageManipulator()
    : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageManipulator::imageCb, this);

    image_pub_b_ = it_.advertise("/image_manipulator/output_b", 1);
    image_pub_g_ = it_.advertise("/image_manipulator/output_g", 1);
    image_pub_r_ = it_.advertise("/image_manipulator/output_r", 1);

    cv::namedWindow(OPENCV_WINDOW_B, cv::WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW_G, cv::WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW_R, cv::WINDOW_NORMAL);
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  }

  ~ImageManipulator() {
    cv::destroyWindow(OPENCV_WINDOW_B);
    cv::destroyWindow(OPENCV_WINDOW_G);
    cv::destroyWindow(OPENCV_WINDOW_R);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    image_src_ = cv_ptr->image;
    cv::split(image_src_, image_rgb_);

    // for(int y = 0; y < image_src_.rows; y++) {
    //   for(int x = 0; x < image_src_.cols; x++) {
    //     cv::Vec3b color = image_src_.at<cv::Vec3b>(cv::Point(x, y));
    //     //   if (color[0] > 100 && color[2] < 200)
    //     //   color[0] = 255;
    //     // if (color[0] < 255)
    //     //   color[0] = 0;
    //     // negative
    //     // for (int i = 0; i < 3; i++)
    //     //   color[i] = 255 - color[i];

    //     image_src_.at<cv::Vec3b>(cv::Point(x, y)) = color;
    //   }
    // }


    // from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
    cv::Mat gray;
    cv::cvtColor(image_src_, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16, 100, 60, 1, 300);
    for (size_t i = 0; i < circles.size(); i++) {
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      cv::circle(image_src_, center, 1, cv::Scalar(0, 100, 100),
                  3, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle(image_src_, center, radius, cv::Scalar(255, 0, 255),
                  3, cv::LINE_AA);
      ROS_INFO("Circle %d at (%d, %d)", i, c[0], c[1]);
    }

    cv::Mat edges;
    cv::Canny(gray, edges, 30, 150, 3, false);


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, image_src_);

    cv::imshow(OPENCV_WINDOW_B, edges);
    // cv::imshow(OPENCV_WINDOW_G, image_rgb_[1]);
    // cv::imshow(OPENCV_WINDOW_R, image_rgb_[2]);
    cv::waitKey(3);

    // Output modified video stream NOT WORKING

    // cv_ptr = cv_bridge::toCvCopy()
    // cv_ptr->image = image_rgb_[0];
    // image_pub_b_.publish(cv_ptr->toImageMsg());
    // cv_ptr->image = image_rgb_[1];
    // image_pub_g_.publish(cv_ptr->toImageMsg());
    // cv_ptr->image = image_rgb_[2];
    // image_pub_r_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_manipulator");
  ImageManipulator im;
  ros::spin();
  return 0;
}