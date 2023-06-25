
# ROS2 Driver for Cerulean Sonar S500

This package implements a ROS driver for the Cerulean Sonar S500 device. It uses the  [ping-cpp](https://github.com/bluerobotics/ping-cpp) package to interface with the device. 




## Installing

To install the package simply clone and build like any ROS2 package. 

```bash
cd <sonar_ws>/src
git clone https://github.com/jackarivera/s500_ros2
cd ../
colcon build
```
    
## Usage

Source your workspace and use the launch file provided.

Launch with provided sonar_config.yaml (s500_sonar/configs/sonar_config.yaml)
```bash
ros2 launch s500_sonar s500.launch.py
```

Launch with overrided parameters
```bash
ros2 launch s500_sonar s500.launch.py device:=/dev/ttyUSB0 baudrate:=115200
```
## License

[MIT](https://choosealicense.com/licenses/mit/)

