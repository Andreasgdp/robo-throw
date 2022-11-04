# robo-throw  
<!-- ABOUT THE PROJECT -->
## About The Project
<p align="center"> 
<img src="https://user-images.githubusercontent.com/39928082/200022665-8c2d359a-6334-4ccb-8d87-b789a3a441dd.gif" alt="SDU" title="SDU" width="80%" height="80%"/> 
</p>


### Abstract
In this project, we have worked with creating a robot system that can identify an object with
the use of machine vision, then pick it up and throw it at any spot in a given target area. Both
the object and the target is being spotted by an RGB camera and then being processed with
machine vision in C++ with the library OpenCV to find their positions. We have then made
translations between the camera’s coordinate system and the table and then between the table and
the robot’s coordinate system. We have made a C++ program that controls the robot and the
gripper with use of the libraries UR-RTDE and Robotics library [3]. The program calculates an
object trajectory with input from the camera and the program runs a simulation in URSim where
every movement is tested before they are send to the robot. All this is being logged in a database
in MySQL through out the program.

<p align="center"> 
<img src="https://user-images.githubusercontent.com/39928082/200008203-6aefe037-1afe-4fd9-b082-2203290b54da.png" alt="Throw" title="Throw" width="15%" height="15%"/> 
</p>



### Built With

* [C++]()
* [Python]()
* [MatLab]()
* [Pure manpower]()

