#include <face_detection/face_detector.h>


int main(int argc, char **argv)
{
  ros::init(argc,argv,"face_detection_node");
  ros::NodeHandle nh("~");

  double frequency = 5;

  ROS_INFO("Creating face detector");

  FaceDetector detector(nh);

  ROS_INFO("Spinning to serve callbacks ...");

  ros::Rate rate(frequency);
  while ( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
