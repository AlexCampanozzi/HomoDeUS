For now it is only a copy of the OpenCV tutorials turned into its own package.

For testing purposes, I changed track_sequential.cpp to use the webcam image. This will be useful for image recognition later on.

To use the webcam, download usb_cam

sudo apt-get install ros-melodic-usb-cam

then you can launch Gazebo following the tutorials

Launch usb_cam with:
rosrun usb_cam usb_cam_node

then
rosrun image_recognition track_sequential
