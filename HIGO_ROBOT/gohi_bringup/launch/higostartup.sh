
#!/bin/bash

roslaunch gohi_hw gohi_hw.launch  &
sleep 1
echo "higo base starting success!"

roslaunch rplidar_ros rplidar.launch  &
echo "rplidar starting success!"
sleep 0.1
roslaunch openni_launch openni.launch &
echo "kinect starting success!"
sleep 0.1
roslaunch simple_voice nav_speaker.launch &
echo "voice recognition starting success!"
sleep 0.1
roslaunch rbx1_vision usb_cam.launch video_device:=/dev/video0
sleep 0.1
echo "left camera starting success!"
roslaunch rbx1_vision usb_cam_left.launch video_device:=/dev/video1
sleep 0.1   
echo "right camera starting success!"

sleep 0.1

wait
exit 0
