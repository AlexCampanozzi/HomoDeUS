#include "face_detector.h"
/**
 * @brief The FaceDetector class publishes face detection info on a ROS topic
 */

/**
 * @brief Constructor of the class, subscribes to the the image topic and initializes
 * variables
 */
FaceDetector::FaceDetector(ros::NodeHandle& nh):
  _nh(nh),
  _imageTransport(nh),
  _imgProcessingSize(-1,-1),
  _minRelEyeDist(0.0375),
  _maxRelEyeDist(0.1)
{
  std::string pathToFrontClassifier = ros::package::getPath("image_recognition") +
                                 "/config/haarcascade_frontalface_alt.xml";
  std::string pathToProfileClassifier = ros::package::getPath("image_recognition") +
                                 "/config/haarcascade_profileface.xml";
  if ( !_frontClassifier.load(pathToFrontClassifier.c_str())  or 
      ! _profileClassifier.load(pathToProfileClassifier.c_str()) )
    throw std::runtime_error("Error loading classifier");

  image_transport::TransportHints transportHint("raw");

  _imageSub = _imageTransport.subscribe("image", 1, &FaceDetector::imageCallback, this, transportHint);

  ROS_INFO_STREAM("Subscribing to image topic: " << _imageSub.getTopic());

  _pub = _nh.advertise<pal_detection_msgs::FaceDetections>("faces", 1);
  _imDebugPub = _imageTransport.advertise("debug", 1);

  _nh.param<int>("processing_img_width", _imgProcessingSize.width, _imgProcessingSize.height);
  _nh.param<int>("processing_img_height", _imgProcessingSize.height, _imgProcessingSize.height);
  _nh.param<double>("rel_min_eye_dist", _minRelEyeDist, _minRelEyeDist);
  _nh.param<double>("rel_max_eye_dist", _maxRelEyeDist, _maxRelEyeDist);
}

/**
 * @brief Destructor of the class
 */
FaceDetector::~FaceDetector()
{
  cv::destroyWindow("face detections");
}

/**
 * @brief Publishes the detected faces on a topic
 * @params faces (vector of Rect): Contains the face locations
 */
void FaceDetector::publishDetections(const std::vector<cv::Rect>& faces)
{
  pal_detection_msgs::FaceDetections msg;
  pal_detection_msgs::FaceDetection  detection;

  msg.header.stamp = _imgTimeStamp;
  msg.header.frame_id = _cameraFrameId;

  BOOST_FOREACH(const cv::Rect& face, faces)
  {
    //publish the detection according to the original image size
    detection.x           = static_cast<int>(face.x      * _originalImageSize.width/_imgProcessingSize.width);
    detection.y           = static_cast<int>(face.y      * _originalImageSize.height/_imgProcessingSize.height);
    detection.width       = static_cast<int>(face.width  * _originalImageSize.width/_imgProcessingSize.width);
    detection.height      = static_cast<int>(face.height * _originalImageSize.height/_imgProcessingSize.height);
    detection.eyesLocated = false;
    detection.leftEyeX    = 0;
    detection.leftEyeY    = 0;
    detection.rightEyeX   = 0;
    detection.rightEyeY   = 0;

    detection.name        = "";
    detection.confidence  = 0;
    msg.faces.push_back(detection);
  }

  _pub.publish(msg);
}

/**
 * @brief Puts recognized faces on image to help with debugging
 * @params: img (Mat): Current image
 *          faces (vector of Rect): Contains the face locations
 */
void FaceDetector::publishDebugImage(const cv::Mat& img,
                                     const std::vector<cv::Rect>& faces)
{
  cv::Mat imgDebug = img.clone();
  BOOST_FOREACH(const cv::Rect& face, faces)
  {
    cv::rectangle(imgDebug, face, CV_RGB(0,255,0), 1);
  }

  if ( imgDebug.channels() == 3 && imgDebug.depth() == CV_8U )
    _cvImg.encoding = sensor_msgs::image_encodings::RGB8;

  else if ( imgDebug.channels() == 1 && imgDebug.depth() == CV_8U )
    _cvImg.encoding = sensor_msgs::image_encodings::MONO8;
  else
    throw std::runtime_error("Error in FaceDetector::publishDebugImage: only 24-bit BGR or 8-bit MONO images are currently supported");

  _cvImg.image = imgDebug;
  sensor_msgs::Image imgMsg;
  _cvImg.toImageMsg(imgMsg); //copy image data to ROS message

  _imDebugPub.publish(imgMsg);
}

/**
 * @brief Checks for faces for each new image
 * @params msg: Image message that will be converted to an OpenCV image
 */
void FaceDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if ( _pub.getNumSubscribers() > 0 || _imDebugPub.getNumSubscribers() > 0 )
  {
    _imgTimeStamp = msg->header.stamp;
    _cameraFrameId = msg->header.frame_id;

    cv::Mat img;
    cv::Rect r;

    std::vector<cv::Rect> faces;
    std::vector<cv::Rect> leftProfileFaces;
    std::vector<cv::Rect> rightProfileFaces;

    cv::Mat flipped;

    cv_bridge::CvImageConstPtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvShare(msg);
    cvImgPtr->image.copyTo(img);

    _originalImageSize = img.size();

    cv::Mat imgScaled;

    if ( _imgProcessingSize.width != -1 )
    {
      _minFaceSize.width = static_cast<int>(_minRelEyeDist * _imgProcessingSize.width / 0.4);
      _maxFaceSize.width = static_cast<int>(_maxRelEyeDist * _imgProcessingSize.width / 0.4);
      cv::resize(img, imgScaled, _imgProcessingSize);
    }
    else
    {
      _minFaceSize.width = static_cast<int>(_minRelEyeDist * _originalImageSize.width / 0.4);
      _maxFaceSize.width = static_cast<int>(_maxRelEyeDist * _originalImageSize.width / 0.4);
      imgScaled = img;
    }

    _minFaceSize.height = _minFaceSize.width;
    _maxFaceSize.height = _maxFaceSize.width;

    faces = detectFaces(imgScaled, _frontClassifier); // Front faces

    leftProfileFaces = detectFaces(imgScaled, _profileClassifier); // Left profile

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

    cv::groupRectangles(faces, 0, 2);
    if ( _pub.getNumSubscribers() > 0 && !faces.empty())
      publishDetections(faces);


    if ( _imDebugPub.getNumSubscribers() > 0 )
      publishDebugImage(imgScaled, faces);
  }
}

/**
 * @brief detects faces in image
 * @params img: Current image
           classifier: labeled data containing faces
 */
std::vector<cv::Rect> FaceDetector::detectFaces(const cv::Mat& img,
                               cv::CascadeClassifier classifier)
{
  cv::Mat imgGray;

  cv::cvtColor(img, imgGray, CV_BGR2GRAY);

  std::vector<cv::Rect> detections;

  classifier.detectMultiScale(imgGray,
                                   detections,
                                   1.1,  
                                   2,
                                   0, //CV_HAAR_DO_CANNY_PRUNING,
                                   _minFaceSize,
                                   _maxFaceSize);
  return detections;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"face_detector"); // Create and name the Node
  ros::NodeHandle nh("~");

  double frequency = 5;
  if ( argc > 2 )
  {
    frequency = atof(argv[2]);
  }

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

