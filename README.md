![Only_ROS_Noetic](https://img.shields.io/badge/ROS-Noetic-informational)

Amperka ROS robot "Abot" source code.

# Documentation

Detailed instructions for assembling the robot are available in Russian:

- [Как сделать робота на ROS своими руками. Часть 1: шасси и бортовая электроника](https://amperka.ru/blogs/projects/abot-robot-part-1)
- [Как сделать робота на ROS своими руками. Часть 2: дистанционное управление и навигация](https://amperka.ru/blogs/projects/abot-robot-part-2)

# Robot packages

The basis of the robot:

- `abot_description` — URDF robot description and main bringup launch files.
- `abot_driver` — Low-level drivers for working with motors and encoders on Raspberry Pi.
- `abot_control` — Robot controllers, differential drive controller.
- `abot_base` — Robot base controller.
- `abot_teleop` — Remote control of the base robot controller via the DualShock 4 joystick.
- `abot_slam` — Building 2D maps using the SLAM method and the `gmapping` package.
- `abot_navigation` — Navigation of the robot on a prepared 2D map.

Working with sound and speech on the robot:

- `abot_speech_to_text` — STT `pocketsphinx` ROS wrapper.
- `abot_text_to_speech` — TTS ROS wrappers: `RHVoice`, `festival`, AWS Polly.
- `abot_text_to_speech` — Voice control of the robot. Voice control of the navigation stack.

# Software requirements

Robot running on Ubuntu 20.04.2.0 Server arm64 and ROS Noetic and Raspberry Pi 4 B 4 Gb.

## Basis part requirements

Necessary software to run the main part of the project:

- [WiringPi](https://github.com/WiringPi/WiringPi)
- [ds4drv](https://github.com/naoki-mizuno/ds4drv)

ROS packages:

```bash
ros-noetic-gmapping
ros-noetic-joy
ros-noetic-pid
ros-noetic-map-server
ros-noetic-navigation
```

## Sound part requirements

Necessary software to run the sound part of the project:

- [GStreamer](https://en.wikipedia.org/wiki/GStreamer)
- [pocketsphinx](https://github.com/cmusphinx/pocketsphinx)
- [ru4sphinx](https://github.com/zamiron/ru4sphinx)
- [WiringPi](https://github.com/WiringPi/WiringPi)
- [TroykaHatCpp](https://github.com/amperka/TroykaHatCpp)
- [festival](https://github.com/festvox/festival)
- [RHVoice](https://github.com/RHVoice/RHVoice)
- [boto3](https://github.com/boto/boto3)
- [sox](https://github.com/chirlu/sox)

ROS packages:

```bash
ros-noetic-audio-common
ros-noetic-navigation
```
