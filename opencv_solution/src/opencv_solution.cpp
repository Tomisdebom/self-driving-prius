#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"

#include <memory>
#include <string>
#include <vector>

#include <iostream>
#include <iomanip>

class ImageConverter
{
  // Initialize all subscribers and publishers
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detection_pub_;
public:
  // Pass nh_ to the constructor of it_.
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/prius/front_camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/opencv_solution_node/visual", 1);
    detection_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/opencv_solution_node/detections", 1);
  }
  // Image callback function
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat mat_image = cv_ptr->image;

    // Initialize HOG detection
    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    // Initialize humans vector and Detection2Darray
    std::vector<cv::Rect> humans;
    vision_msgs::Detection2DArray detections;

    // Use HOG to detect humans a
    hog.detectMultiScale(mat_image, humans, 0 , cv::Size(8,8), cv::Size(0,0), 1.05, 2);
    
    // Put a rectangle around each detection
    for (int i = 0; i<humans.size(); i++){
      cv::rectangle(mat_image, humans[i].tl(), humans[i].br(), cv::Scalar(0, 255, 0), 2);
      
      // Construct a new detection
      vision_msgs::Detection2D detection;

      // Set the detection position
      detection.bbox.center.x = humans[i].x;
      detection.bbox.center.y = humans[i].y;

      // Put the detection in the list of detections
      detections.detections.push_back(detection);
    }

    // Publish the image message
    image_pub_.publish(cv_ptr->toImageMsg());

    // Publish detections
    detection_pub_.publish(detections);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}