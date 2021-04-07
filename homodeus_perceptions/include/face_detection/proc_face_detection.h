#ifndef FACEDETECTION_H
#define FACEDETECTION_H


// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom headers
#include <custom_msgs/FacePosition.h>
#include <custom_msgs/FacePositions.h>

// Boost headers
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>
#include <stdexcept>

/* ArmInterface
Description:    This class uses the Haar Cascades algorithm in OpenCV to allow the detection
                of faces in an image.
Attributes:     _nh (type, ros::NodeHandle):
                    Necessary for the proper handling of a ROS node

                _pub (type, ros::Publisher):
                    The topic to publish the faces on
                
                _frontClassifier (type, cv::CascadeClassifier):
                    The dataset used to recognize front faces

                _profileClassifier (type, cv::CascadeClassifier):
                    The dataset used to recognize profile faces

                _imageSub (type, image_transport::Subscriber):
                    The topic containing the image)

                _imDebugPub (type, image_transport::Publisher):
                    The topic to publish the image on (for debugging purposes)
                
                _imgProcessingSize (type, cv::Size):
                    The size of the image we use face detection on
                
                _minFaceSize, _maxFaceSize (type, cv::Size):
                    Possible dimensions of the faces detected
*/
class FaceDetector
{
public:

  FaceDetector(ros::NodeHandle& nh);
  virtual ~FaceDetector();

protected:

  ros::NodeHandle _nh;
  ros::Publisher _pub;
  ros::Publisher observer_pub;

  cv::CascadeClassifier _frontClassifier;
  cv::CascadeClassifier _profileClassifier;
  cv::Size _imgProcessingSize;
  cv::Size _minFaceSize, _maxFaceSize;

  image_transport::Publisher _imDebugPub;
  image_transport::Subscriber _imageSub;
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  std::vector<cv::Rect> detectFaces(const cv::Mat& img,
                    cv::CascadeClassifier classifier);

  void publishDetections(const std::vector<cv::Rect>& faces);

  void publishDebugImage(const cv::Mat& img,
                         const std::vector<cv::Rect>& faces);
};

#endif