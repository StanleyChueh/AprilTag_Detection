# AprilTag_Detection
💡 In this repository, I provide a way to visualize the coordinates of AprilTags in RVIZ. It'll be useful for localizing the robot in the map frame. 

## Install AprilTag Library
```
pip install apriltag
```

## Install OpenCV,CV_Bridge
``` 
sudo apt-get install ros-foxy-cv-bridge
pip install opencv-python
pip install numpy
```

## Build
``` 
cd ~
mkdir apriltag_ws
cd ~/apriltag_ws
mkdir src
cd src
git clone https://github.com/StanleyChueh/AprilTag_Detection.git
cd ..
colcon build
source install/setup.bash
ros2 launch apriltag_detection detection.launch.py 
```
DEMO:https://youtu.be/O2OGl4VvAsE?si=Pe6WszfrXSu8xyVe
![Screenshot from 2024-06-01 09-56-04](https://github.com/StanleyChueh/AprilTag_Detection/assets/153347369/fa684032-54d7-4b8e-8239-609624e7593d)

## Test On ROS2 Foxy
ROS2 Foxy:https://docs.ros.org/en/foxy/Tutorials.html

## Support
🤝 Please create an issue on this GitHub for any questions. This allows other people with the same question to find your answer. 🤝

## Reference📌
AprilTag:https://github.com/AprilRobotics/apriltag
