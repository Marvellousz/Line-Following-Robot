# Line Following Robot Project

## Overview

This project implements an intelligent line following robot using Arduino. The robot has been developed in three progressive versions, each adding new capabilities to enhance its functionality.

## Project Files

The project consists of three main Arduino sketch files:

1. **linefollowing.ino** - Basic line following functionality
2. **linefollowingwithtcs3200.ino** - Line following with color detection
3. **linefollowingwithtcsanddet...** - Line following with color detection and ultrasonic obstacle detection

## Features

### Basic Line Following (linefollowing.ino)

- Line detection and tracking using IR sensors
- PID control for smooth navigation
- Adjustable motor speeds for precise control


### Color Detection (linefollowingwithtcs3200.ino)

- All features from the basic version
- TCS3200 color sensor integration
- Variable speed control based on detected surface colors:
    - Red: Slow speed
    - Green: Medium speed
    - Blue: Fast speed


### Obstacle Detection (linefollowingwithtcsanddetection)

- All features from previous versions
- Ultrasonic sensor integration for obstacle detection
- Obstacle avoidance capabilities
- Real-time decision making based on both line position and obstacles


## Hardware Requirements

- Arduino Uno
- L298N Motor Driver
- 2x IR sensors for line detection
- TCS3200 Color Sensor
- Ultrasonic Distance Sensor (HC-SR04)
- DC Motors
- Chassis and wheels
- Power supply
- Jumper wires


## Setup Instructions

1. Connect all components according to the pin configurations in each sketch
2. Position sensors appropriately:
    - IR sensors pointing downward at the front to detect the line
    - TCS3200 color sensor approximately 1cm from the surface
    - Ultrasonic sensor at the front for obstacle detection
3. Upload the desired sketch to your Arduino
4. Place the robot on a line track and power on

## Customization

Each version of the code includes configurable parameters:

- PID constants for line following
- Speed settings for different colors
- Distance thresholds for obstacle detection
- Motor speed adjustments


## Future Improvements

- Bluetooth control capabilities
- Data logging and analysis
- Multiple obstacle avoidance strategies
- Integration with other sensors


## License

This project is open source and available under the MIT License.



