# AprilTag_Detection
``` 
cd ~
mkdir apriltag_ws
cd ~/apriltag_ws
mkdir src
cd src
git clone https://github.com/StanleyChueh/AprilTag_Detection.git
colcon build
cd ..
source install/setup.bash
ros2 launch apriltag_detection detection.launch.py 
```
