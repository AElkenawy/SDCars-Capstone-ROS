# **System Integration Capstone Project**

## Project goal
Integration of Autonomous vehicle subsystems of perception, localization, planning and control into one system using ROS (Robot Operating System) middleware. The vehicle should be able to drive around the test track using waypoints navigation while obeying traffic rules (and avoiding obstacles).

## Project overview
The aim of _drive-by-wire (dbw)_ system configuration is to generate acceleration, braking and steering commands based on sensor measurement. The core elements of the autonomous vehicle system are perception, localization, planning and control subsystems.

<img src="./imgs/1_Architecture overview.png" alt="System Subsystems" width="800">


The system architecture is based on open-source software for self-driving vehicles of _Autoware_. ROS nodes and topics used to implement core functionality of the autonomous vehicle system of waypoint following, traffic light detection and control are illustrated as

<img src="./imgs/2_ROS Tpcs Nds.png" alt="ROS topics and nodes" width="800"> 


Nodes of interest in this project are:
1. Waypoint Updater Node **(Planning subsystem)** `./ros/src/waypoint_updater/waypoint_updater.py`
<img src="./imgs/3_WPUpdate node.png" alt="Wayppoint Updater Node"> 

2. DBW/drive-by-wire Node  **(Control subsystem)** `./ros/src/twist_controller/dbw_node.py`
<img src="./imgs/4_DBW node.png" alt="DBW Node"> 

3. Traffic Light Detection Node **(Perception subsystem)** `./ros/src/tl_detector/tl_detector.py`. 
Traffic Light Classifier is a SSD (Single Shot MultiBox Detector) CNN model. `./ros/src/tl_detector/light_classification/tl_classifier.py`
<img src="./imgs/5_TLDetect node.png" alt="Traffic Light Detection Node"> 

### Basic Build Instructions
Please use **one** of the two installation options, either native **or** docker installation.

#### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

#### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

#### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

#### Usage

1. Clone the project repository
```bash
git clone https://github.com/AElkenawy/SDCars-Capstone-ROS.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator
