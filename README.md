# Rainbow Dash - ROS on ARDUINO

### Dependencies

[Setup Arduino IDE to use ROS](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

On linux computer with Arduino IDE:
```
sudo apt install ros-kinetic-rosserial-arduino
sudo apt install ros-kinetic-rosserial
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

On HOST communicating with Arduino:
```
cd <workspace>/src
git clone https://github.com/ros-drivers/rosserial.git
cd <workspace>
catkin_make
```


[MEGA 2560 PINOUT](https://docs.google.com/spreadsheets/d/1PkfsWlCeW8HTPtbFbr59YXDUESc_GEy8GViLvgwnDSE/edit?usp=sharing)


