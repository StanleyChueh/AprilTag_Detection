# AprilTag_Detection
```
pip install apriltag
``` 
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
