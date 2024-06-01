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
![Screenshot from 2024-06-01 09-56-04](https://github.com/StanleyChueh/AprilTag_Detection/assets/153347369/fa684032-54d7-4b8e-8239-609624e7593d)

