

image_recognition
========================

ROS package wrapping OpenCV's face detection cascade. It provides a ROS node that subscribes to an image topic and publishes the following topics:

   ```
   /pal_face/faces
   /pal_face/debug
   ```

The `/pal_face/faces` topic contains a `pal_detection_msgs/FaceDetections` message with the regions of interest classified as faces.
The `/pal_face/debug` is an image topic with the faces detected painted on it.


### How to test

roslaunch image_recognition detector.launch

roslaunch usb_cam usb_cam-test.launch 
rosrun usb_cam usb_cam_node

rosrun image_view image_view image:=/pal_face/debug


