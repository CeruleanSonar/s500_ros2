
# ROS2 Driver for Cerulean Sonar S500

This package implements a ROS driver for the Cerulean Sonar S500 device. It uses the  [ping-python](https://github.com/bluerobotics/ping-python) package to interface with the device. 




## Installing

To install the package simply clone and build like any ROS2 package. 

```bash
cd <sonar_ws>/src
git clone https://github.com/jackarivera/s500_ros2
cd ../
rosdep install --from-paths src --ignore-src
colcon build
```
    
## Usage

Source your workspace and run the sonar node via:

Run the sonar node
```bash
ros2 run s500_sonar sonar_node
```
## License

[MIT](https://choosealicense.com/licenses/mit/)

