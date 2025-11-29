Arduino Mega Vehicle Controller Firmware
----------------------------------------

This sketch runs on the Arduino Mega used as the vehicle controller for the scale autonomous vehicle.

- Reads serial commands from ROS 2 nodes (F/S/L/R or steering/speed values).
- Drives the rear motor and steering motor through the H-bridge drivers.
- Implements "hard stop" behavior when the LiDAR stop command is received.
