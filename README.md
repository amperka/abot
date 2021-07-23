![Only_ROS_Noetic](https://img.shields.io/badge/ROS-Noetic-informational)

Amperka ROS robot "Abot" source code.

# Documentaion

Detailed instructions for assembling the robot are available in Russian:

- [Как сделать робота на ROS своими руками. Часть 1: шасси и бортовая электроника](https://amperka.ru/blogs/projects/abot-robot-part-1)
- [Как сделать робота на ROS своими руками. Часть 2: дистанционное управление и навигация](https://amperka.ru/blogs/projects/abot-robot-part-2)


# Software requirements

Robot running on Ubuntu 20.04.2.0 Server arm64 and ROS Noetic.

Necessary software to run the project:

- [WiringPi](https://github.com/WiringPi/WiringPi)
- [ds4drv](https://github.com/naoki-mizuno/ds4drv)

Necessary ROS packages to run the project:

```bash
ros-noetic-gmapping
ros-noetic-joy
ros-noetic-pid
ros-noetic-map-server
ros-noetic-navigation
```
