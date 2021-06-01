# Mahindra_ADAS_System
An implementation of Forward warning collision system and lane assist in MATLAB for the vehicle controlled by ROS

<b><h1>Abstract</h1></b>

The following thesis discusses a section of the parent project, focusing on the development of an Autonomous Controller for a driverless car. The primary aim is to design an Automated Driving System with lane assist and forward-collision warning system which could be run successfully on IIT Delhi roads. Designed algorithms, used preexisting libraries on MATLAB, and then also integrated it with ROS for live communication. Gazebo simulation systems, video sources, and even live video feed was used for testing some of the algorithms and the qualitative comparison of the designed models. The main target is to exploit the capabilities of MATLAB and use its inbuilt libraries, models, and algorithms to achieve the desired output and control signals.

<b><h1>Overview</h1></b>

For this undergraduate thesis, the aim is to design a vision based assist system capable of controlling the e2o car autonomously, keeping the stereo camera as the global sensor. The idea is to use the technology which is developed by Mathworks and mold it according to the required needs. Initially, the focus was on understanding and working on the MATLAB vision toolbox, but later, the approach was diversified. The algorithms were tested on simulation and real-world videos. ROS-MATLAB framework was also explored to integrate the results and outputs to control the car


<b><h1> Implementation : Forward Collision Warning System </h1></b>
The forward warning collision system will take in live video from the monocular camera, and give the world coordinates of the vehicle as well as the pedestrian wrt the Ego vehicle. In case a vehicle or pedestrian is very close to the Ego vehicle brakes will be deployed by sending brake msgs over the MATLAB-ROS bridge 

1. Using the MATLAB Camera callibration toolbox, calibrate the camera to get the intrinsic and extrinsic properties 
2. Feed the found properties to the MATLAB code(Final_car.m) 
3. Give the correct Video path on which the code is to be run and run the code 
 
MATLAB 2019b was used for the generation of the result below

<h1><p align="left">Results</p></h1>

<p align="center"><img src="https://github.com/dhruvtalwar18/Mahindra_ADAS_System/blob/main/images_BTTPII/FCWS_GIF.gif" title="Result 1"></p>
<p align="center">Fig.1 Live Vechicle detection</p><br />
<img src="https://github.com/dhruvtalwar18/Mahindra_ADAS_System/blob/main/images_BTTPII/car_ped.png" title="Result 2">
<p align="center">Fig.2 Vechicle and Pedestrian Detection</p><br />

In a pedestrian or a vehicle comes closer to the Ego vehicle than the safe distance set, the algorithm would send braking commands to the ROS master on the ROS topic /E2OCtrl
And the detection would appear in red bounding boxes
Eg:

<img src="https://github.com/dhruvtalwar18/Mahindra_ADAS_System/blob/main/images_BTTPII/close_ped.png" title="Result 3">
<p align="center"> Fig.3 Unsafe distance of the Pedestrian from the car</p>

<b><h1> Implementation : Lane Assist System </h1></b>





