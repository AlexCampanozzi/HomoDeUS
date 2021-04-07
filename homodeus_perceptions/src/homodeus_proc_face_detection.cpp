#include <face_detection/homodeus_proc_face_detection.h>

#define MIN_FACE_SIZE_RATIO 0.00001
/* FaceDetector: Constructor

Description:    subscribes to the the image topic and initializes variables

Inputs:         _nh (type, ros::NodeHandle): Manages things related to the node
          
Outputs:        None
*/
FaceDetector::FaceDetector(ros::NodeHandle& nh, std::string mode):
  _nh(nh)
{
  // Image topics
  std::string imageTopic = "proc_input_camera_feed";

  image_transport::ImageTransport imageTransport(nh);

  std::string pathToFrontClassifier = ros::package::getPath("homodeus_external") +
                                 "/face_detection/config/haarcascade_frontalface_alt.xml";
  std::string pathToProfileClassifier = ros::package::getPath("homodeus_external") +
                                 "/face_detection/config/haarcascade_profileface.xml";

  std::string camera_mode = mode;
  // Checks if the classifier files are there
  if ( !_frontClassifier.load(pathToFrontClassifier.c_str())  or 
      ! _profileClassifier.load(pathToProfileClassifier.c_str()) )
    throw std::runtime_error("Error loading classifier");

  //Initializing subscribers and publishers
  image_transport::TransportHints transportHint("raw");
  _imageSub = imageTransport.subscribe(imageTopic, 1, &FaceDetector::imageCallback, this, transportHint);

  _pub = _nh.advertise<custom_msgs::FacePositions>("/proc_output_face_positions", 1);
  _imDebugPub = imageTransport.advertise("debug", 1);

  sensor_msgs::CameraInfo camera_info;

  //Dimensions of the image
  if (camera_mode == "remote")
  {
    camera_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/usb_cam/camera_info"));
  }
  else
  {
    camera_info = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/xtion/rgb/camera_info"));
  }
  _imgProcessingSize.height = camera_info.height;
  _imgProcessingSize.width = camera_info.width;
}


/* FaceDetector: Destructor

Description:    Closes the image window during shutdown

Inputs:         None
          
Outputs:        None
*/
FaceDetector::~FaceDetector()
{
  cv::destroyWindow("face detections");
}


/* FaceDetector: publishDetections

Description:    Publishes the detected faces on a topic

Inputs:         faces (type, std::vector<cv::Rect>): Contains the face locations
          
Outputs:        None (publishes on /pal_face/faces)
*/
void FaceDetector::publishDetections(const std::vector<cv::Rect>& faces)
{
  custom_msgs::FacePositions msg;
  custom_msgs::FacePosition  detection;

  BOOST_FOREACH(const cv::Rect& face, faces)
  {
    //Information to publish
    detection.x           = static_cast<int>(face.x);
    detection.y           = static_cast<int>(face.y);
    detection.width       = static_cast<int>(face.width);
    detection.height      = static_cast<int>(face.height);
    msg.faces.push_back(detection);
  }
  _pub.publish(msg);
}


/* FaceDetector: publishDebugImage

Description:    Puts recognized faces on image to help with debugging

Inputs:         img (type, cv::Mat): Current image
                faces (std::vector<cv::Rect>): Contains the face locations
          
Outputs:        None (publishes on topic /pal_face/debug)
*/
void FaceDetector::publishDebugImage(const cv::Mat& img,
                                     const std::vector<cv::Rect>& faces)
{
  cv::Mat imgDebug = img.clone();
  cv_bridge::CvImage cvImg;

  BOOST_FOREACH(const cv::Rect& face, faces)
  {
    cv::rectangle(imgDebug, face, CV_RGB(0,255,0), 1);
  }

  if ( imgDebug.channels() == 3 && imgDebug.depth() == CV_8U )
    cvImg.encoding = sensor_msgs::image_encodings::RGB8;

  else if ( imgDebug.channels() == 1 && imgDebug.depth() == CV_8U )
    cvImg.encoding = sensor_msgs::image_encodings::MONO8;
  else
    throw std::runtime_error("Error in FaceDetector::publishDebugImage: only 24-bit BGR or 8-bit MONO images are currently supported");

  cvImg.image = imgDebug;
  sensor_msgs::Image imgMsg;
  cvImg.toImageMsg(imgMsg); //copy image data to ROS message

  _imDebugPub.publish(imgMsg);
}


/* FaceDetector: imageCallback

Description:    Checks for faces for each new image

Inputs:         msg (type, sensor_msgs::ImageConstPtr): Image message that will be converted to an OpenCV image
          
Outputs:        None
*/
void FaceDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if ( _pub.getNumSubscribers() > 0 || _imDebugPub.getNumSubscribers() > 0 )
  {
    cv::Mat img;
    cv::Rect r;

    std::vector<cv::Rect> faces;
    std::vector<cv::Rect> leftProfileFaces;
    std::vector<cv::Rect> rightProfileFaces;

    cv::Mat flipped;
    cv::Mat imgScaled;

    cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvShare(msg);
    cvImgPtr->image.copyTo(img);
  
    // Minimum and maximum sizes of the faces that can be detected
    _minFaceSize.width = static_cast<int>(MIN_FACE_SIZE_RATIO * _imgProcessingSize.width);
    _maxFaceSize.width = static_cast<int>(_imgProcessingSize.width);
    cv::resize(img, imgScaled, _imgProcessingSize);

    _minFaceSize.height = _minFaceSize.width;
    _maxFaceSize.height = _maxFaceSize.width;

    faces = detectFaces(imgScaled, _frontClassifier); // Front faces

    leftProfileFaces = detectFaces(imgScaled, _profileClassifier); // Left profile
    cv::flip(imgScaled, flipped, 1);
    rightProfileFaces = detectFaces(flipped, _profileClassifier); // Right Profile

    // Flipping the rectangles detected for the right profile
    for(int i=0;i<rightProfileFaces.size();i++)
    {
        r=rightProfileFaces[i];
        r.x=flipped.cols-r.x-r.width;
        rightProfileFaces[i]=r;
    }

    faces.insert(faces.end(), leftProfileFaces.begin(), leftProfileFaces.end());
    faces.insert(faces.end(), rightProfileFaces.begin(), rightProfileFaces.end());

    cv::groupRectangles(faces,  // Merges rectangles that are close to each other
                        1, // Min number of rectangles
                        0.5 // Distance between rectangles to merge
                        );

    if ( _pub.getNumSubscribers() > 0 && !faces.empty())
      publishDetections(faces);

    if ( _imDebugPub.getNumSubscribers() > 0 )
      publishDebugImage(imgScaled, faces);
  }
}


/* FaceDetector: detectFaces

Description:    Detects faces in image

Inputs:         img (type, cv::Mat): Current image
                classifier (type, cv::CascadeClassifier): labeled data containing faces
          
Outputs:        detections (type, std::vector<cv::Rect>): list of faces detected
*/
std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& img,
                               cv::CascadeClassifier classifier)
{
  cv::Mat imgGray;
  cv::cvtColor(img, imgGray, CV_BGR2GRAY);
  std::vector<cv::Rect> detections;

  classifier.detectMultiScale(imgGray,
                                   detections,
                                   1.1,  // Scale Factor
                                   2, // minNeighbors
                                   0, // flags
                                   _minFaceSize,
                                   _maxFaceSize);
  return detections;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"homodeus_proc_face_detection_node");
  std::string mode;

  ros::NodeHandle nh;

  double frequency = 5;

  ROS_INFO("Creating face detector");
  nh.getParam("camera_mode", mode);

  ROS_INFO("Face detection camera mode: %s", mode.c_str());
  FaceDetector detector(nh, mode);
  ROS_INFO("Spinning to serve callbacks ...");

  ros::Rate rate(frequency);
  while ( ros::ok() )
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
