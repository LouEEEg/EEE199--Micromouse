# EEE199-Micromouse

![MicroMouse](https://github.com/Lou-ee/EEE199--Micromouse/blob/2d07c3c2da80336d45141a2d836fa3eb12fdac01/Images/Mouse_Main.jpeg "EEE199-Micromouse")


## Overview
EEE199-Micromouse is an ongoing independent project featuring an autonomous differential drive robot. Begining as a standard IEEE competition Micromouse, the project has evolved into its current state as a learning tool for robot navigation using machine vision. The robot currently relies on a remote mounted overhead camera to provide cartesian coordinates and trajectory generation. Images are captured with a Raspberry Pi & Pi-Cam and processed in MATLAB. The  trajectory data is sent to the robot using BLE. 

The current state of the project can be grouped into four main categories:   
   1. Robot design & Kinematic Controller
   2. Absolute Coordinate System using Machine Vision/ Hough Transform
   3. Localization & path planning
   4. Path Following

Future goals I am working toward include:
   1. Map generation using images
   2. Map generation using robot perceptors
   3. SLAM running native on robot controller

## Robot Design

#### Modeling

   The robot is a four wheel, dual motor, differential drive robot. Modeling and design was completed in Fusion 360 and 3D printed with PETG on an Ender 3 pro running Klipper/ Octoprint. 

<img src="https://github.com/Lou-ee/EEE199--Micromouse/blob/e7db04d1f170ffef34a7609a15892aa0f72debba/Images/3D_Model.png" width="625" height="500">


#### Main Components
   |  Part |  Use | Qty  |
   | :---: | :---:|:---: |
   | GP2Y0A51SK0F infrared sensor | Range perception for mapping, object detection & avoidance | 5 |
   | ESP 32 | Robot Controller: PID controller, BLE, I/O data processing | 1 |
   | BNO055 IMU | Robot orientation | 1 |
   | AS5048B Encoders | Wheel position feedback, odometry | 2 | 
   | Raspberry Pi 3B+ | Image Capture & BLE | 1 |

#### Data Flow

<img src="https://github.com/Lou-ee/EEE199--Micromouse/blob/2483e82bbe608f3d427465cd6741989928ad0a47/Images/Data_Path.png">
   
#### Navigation Algorithm


<img src="https://github.com/Lou-ee/EEE199--Micromouse/blob/2483e82bbe608f3d427465cd6741989928ad0a47/Images/Algo_Main.png">

#### Machine Vision 

#### Path Generation

#### Trajectory Following

   
## Video Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/qSRAssMJCGc/0.jpg)](https://youtu.be/qSRAssMJCGc)

https://youtu.be/qSRAssMJCGc


## Conclusion 
Thanks for reading. 
