#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <numeric>
#include <cmath>
#include <std_msgs/Float32.h>

 
static const char WINDOW[] = "Image window";

cv::Point2f lastPosition;
ros::Time lastTime;
std::vector<float> periods;
int counter = 0;

void detectAndDrawRed(cv::Mat& frame);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
 
  //Reading an image from the file
  cv::VideoCapture cap;

  cap.open("/dev/video0",cv::CAP_ANY);

  if (!cap.isOpened()) {
    ROS_INFO("Open camera failed!");
    return -1;
  }
  
  cv::Mat cv_image;
  //Show the image
  cv::namedWindow(WINDOW);
  //Convert OpenCV image to ROS message
  ros::NodeHandle node;
  ros::Publisher accel_pub=node.advertise<std_msgs::Float32>("accel1",10);
  image_transport::ImageTransport transport(node);
  image_transport::Publisher image_pub; 
  image_pub=transport.advertise("OutImage", 1);
  sensor_msgs::Image im;
  std_msgs::Float32 accel_msg;

  ros::Rate loop_rate(24);
  while(ros::ok()){
    cap.read(cv_image);
    if(cv_image.empty() )
    {
        ROS_ERROR("Read the picture failed!");
        return -1;
    }
 
    detectAndDrawRed(cv_image);

    ros::Time time=ros::Time::now(); 
 
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = cv_image;
 
    
    cvi.toImageMsg(im);
    image_pub.publish(im);
    
    accel_pub.publish(accel_msg);
    //ROS_INFO("Converted Successfully!");
 
    
    cv::imshow(WINDOW,cv_image);
    loop_rate.sleep();
  }
  cv::waitKey(0);
  cap.release(); // 释放VideoCapture对象
  cv::destroyAllWindows(); // 关闭所有打开的窗口

  return 0;
}

void detectAndDrawRed(cv::Mat& frame) {
    // Convert the image from BGR to HSV color space
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // Define the color range for red color
    cv::Mat red;
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), red);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw bounding boxes for each contour
    for (const auto& contour : contours) {
        // Calculate the area of the contour
        double area = cv::contourArea(contour);
        
        // If the area is larger than 200, draw the bounding box
        if (area > 200) {
            cv::Rect box = cv::boundingRect(contour);
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);

            // Calculate the center position of this contour
            cv::Point2f position(box.x + box.width / 2.0f, box.y + box.height / 2.0f);

            // Detect the swing period
            if (lastPosition.x < position.x && counter % 2 == 0) {  // The laser pen is moving to the right
                if (lastTime != ros::Time()) {  // Ignore the first detection
                    periods.push_back((ros::Time::now() - lastTime).toSec());
                    if (periods.size() == 3) {  // If we have recorded three periods, calculate the average
                        float avgPeriod = std::accumulate(periods.begin(), periods.end(), 0.0f) / 3.0f;
                        float length = pow(avgPeriod * 2 / (2 * M_PI), 2) * 9.8;  // Calculate the length
                        std::cout << "Average period: " << avgPeriod << " seconds" << std::endl;
                        std::cout << "Length: " << length << " meters" << std::endl;
                        periods.clear();
                    }
                }
                lastTime = ros::Time::now();
                counter++;
            } else if (lastPosition.x > position.x && counter % 2 == 1) {  // The laser pen is moving to the left
                lastTime = ros::Time::now();
                counter++;
            }
            lastPosition = position;
        }
    }
}


