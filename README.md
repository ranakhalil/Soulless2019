# Intro

Team Soulless have competed in the [Self Racing Cars event](http://selfracingcars.com/), and the code in the repository is the autonomy stack which ran on a Kia Niro with a drive by wire [DriveKit](https://polysync.io/drivekit) from Polysync.

# Overview 

The code is split into different rosnodes with separate functionalities:

## 1. Drive by wire / Joysticking the vehicle:

Our team forked Polysync's [ROSCCO](https://github.com/PolySync/roscco) package which has a simple joystick controller that we have tested with a PS4 controller, and compataible with an XBOX controller. We have only added an additional node to test getting reported values from the vehicle to make sure our canbus is setup properly.

The node we have added is in the [roscco_report.cpp](https://github.com/ranakhalil/roscco/blob/3d467604ad93c5f56085d3b6194c30efaaf72f7e/example/roscco_report.cpp) file.

## 2. Perception:

SegNet was used for semantic segmentation, however with the twist of using YOLOv2 for feature extraction. Thunderhill's Semantic segmentation is availble at: https://github.com/jendrikjoe/SegNet/tree/thunderhill2019

## 3. Trajector Generation:

Using the semantic segmentation processed image, we have looked at the driveable regions of the road using lane markings and cones as boundaries, our algorithm drew trajector lines as the example shown below:

![Painted Trajectory Example](traject_thunderhill/0_traject.jpg "Trajectories")
 
 More examples of painted trajectories are under the traject_thunderhill/ repository.

 Details of the algorithm have been implemented in iterations, and a python notebook is availble to view [here](https://github.com/ranakhalil/Soulless2019/blob/master/curves.ipynb)

## 4. Velocity PID controller and OBD reader

TBD more information with code cleanup.


 # Running the code:

 In order to get all the dependencies there is an `install.sh` file providing you with all you need to install (Might need some additions, feel free to open an issue if you get stuck).

 To check all related code for the submodules, run `gitall.sh`, and to make the project run `make.sh`


 # Launching the code:

 There is a `launch.sh` file to provide a great entry point to the project. However all launch files are also availble under: https://github.com/ranakhalil/Soulless2019/tree/master/src/launch 



