// PAL headers
#include <pal_detection_msgs/FaceDetections.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>
#include <stdexcept>

class FaceDetector
{
public:

  FaceDetector(ros::NodeHandle& nh);
  virtual ~FaceDetector();

protected:

  ros::NodeHandle _nh;
  bool _verbosePublishing;
  ros::Time _imgTimeStamp;
  std::string _cameraFrameId;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  std::vector<cv::Rect> detectFaces(const cv::Mat& img,
                    cv::CascadeClassifier classifier);

  void publishDetections(const std::vector<cv::Rect>& faces);
  void publishDebugImage(const cv::Mat& img,
                         const std::vector<cv::Rect>& faces);

  cv::CascadeClassifier _frontClassifier;
  cv::CascadeClassifier _profileClassifier;

  image_transport::ImageTransport _imageTransport;
  image_transport::Subscriber _imageSub;

  ros::Publisher _pub;
  image_transport::Publisher _imDebugPub;
  cv_bridge::CvImage _cvImg;

  cv::Size _imgProcessingSize, _originalImageSize;
  double _minRelEyeDist, _maxRelEyeDist;
  cv::Size _minFaceSize, _maxFaceSize;

};