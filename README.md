# trackball_arm
Trackball Controlled Robotic Arm

This program will work for a Lynx motion AL5D Robotic Arm and a Pololu Micro Maestro servo control board, along with a trackball with scroll ring (such as the Kensington Orbit Trackball USB Mouse with Scroll Ring).  It has been developed on a Raspberry Pi with Raspbian Linux.

The trackball is read on /dev/input/event0 and the Pololu Micro Maestro is written to at /dev/ttyACM0 with a baud rate of 115200.

The trackball controls X/Y, the scroll wheel controls Z, and the two buttons are gripper open/close.

For a demo, see https://www.youtube.com/watch?v=7MxWVD4g58I

The inverse kinematics come from somehwere I forget, but perhaps https://bldlive.wordpress.com/2010/07/15/robot-arm/
