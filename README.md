# IRL with dynamical system

Includes mouse_perturbation ROS package, the marker package, and the IRL matlab code.

## Getting Started
remember the ros program in this folder is not the updated one, the updated one is in 


### Prerequisites
https://github.com/yias/motion_example.git


```
Give examples
```

### Files

#### FOLDER: mouse\_perturbation_robot 			

It includes c++ code, experiment protocol, modulation dynamical system.

##### MotionGenerator

the constant to be specified: numObstacle (1 or 2). 

##### MouseInterface
Declare mouseMessage and publish it to the motionGenerator (in my PC). If we use the code in KUKA robot-arm PC, then the mouse data is published by another node "spacenav".



#### FOLDER: obstacle\_with\_sample\_from_dynamics 		
matlab code, experiment IRL node.

### Usage of ROS code




### Usage of MATLAB code

launch MATLAB  
`$cd /usr/local/MATLAB/R20XXx/bin/`  
`$./matlab`  
addpaths.m

## Running the codes

`$cd catkin_ws/`  
`$catkin_make clean`  
`$catkin_make`  
`$source devel/setup.bash`

`$roscore`  

`$roslaunch lwr_simple_example sim.launch` OR
`$roslaunch lwr_simple_example real.launch`

`$roslaunch lwr_fri lwr_fri_console` typye control inside and starting working with the robotarm

`$roslaunch spacenav_node classic.launch`
`$rostopic echo -c /spacenav/joy`

`$rosrun using_markers basic_shapes`
`$rosrun using_markers arrow`

`$rosrun motion_example moveToDesiredJoints 42 45 0 -75 60 -45 -45` - this is the horizontal end effector orientation.

`$rosrun motion_example moveToDesiredJoints 42 45 0 -75 0 55 0`
downward end effector 
`$rosrun motion_example moveToDesiredJoints 42 45 0 -75 0 55 20` to solve the shaking problem  
`$rosrun motion_example moveToDesiredJoints 43.1 53 0.15 -76 -0.1 53.5 20`  
Four points  
`$rosrun motion_example moveToDesiredJoints 50 34.7 1.73 -92.2 -1.94 52.8 42.2`    
`$rosrun motion_example moveToDesiredJoints 44.5 34.7 1.73 -92.2 -1.94 52.8 42.2`    
`$rosrun motion_example moveToDesiredJoints 55 53.8 5.3 -57.8 -3.1 69.6 40`  
`$rosrun motion_example moveToDesiredJoints 45.6 39.2 5.38 -88.98 0.986 52.8167 45`  

damping eigen - 90/1000  
damping eigen - 90/500  
rot stiff - 15/50  
rot damping - 3/50  

two obstacle : 45 37 0 -75 0 65 0  

`$rosrun rqt_reconfigure rqt_reconfigure`

`$rosrun mouse_perturbation_robot mouseInterface /dev/input/event#`

`$rosrun mouse_perturbation_robot motionGenerator`

Check the 3D mouse   
`$ls /dev/input`
`$sudo su`

Check messages
`$rostopic echo -c /lwr/ee_pose`

`$rosrun rqt_graph rqt_graph`

Parametere setting in the robotarm PC:  
damping eigen - 90/1000  (80 - walid)
damping eigen - 90/500  (80 - walid)
rot stiff - 11/50    (20 - walid)
rot damping - 2/50  

while in my PC, it should be 850 and 850 for damping eigen.
### code procedure

start the ROS core



start the mouse interface node

start the motion generator node


## Deployment

Add additional notes about how to deploy this on a live system


## parameters
tailEffect should be off  
the obstacle definition in matlab should be larger?  


in the header file "MotionGenerator.h", flag PROTOCAL_DEBUG is for debugging the EEG decoding, which means the end effector will return to the origin position if the user release the 3D mouse.  



## multiple machines
/etc/hosts  
add ip address and name.

Something like that: 
128.179.131.239 fumi-ThinkPad
128.179.160.1 DESKTOP-TLGS8JE


dialout
'ls /dev/ttyACM*'
'python python_client.py'
'ent group dialout'
'sudo adduser shupeng dialout'


reference links: 
[The matlab page](https://www.mathworks.com/help/robotics/examples/connect-to-a-ros-network.html)
[The list of checking](https://www.mathworks.com/matlabcentral/answers/196911-use-matlab-robotics-system-toolbox-to-receive-ros-message)
[hosts](ftp://ftp.iitb.ac.in/LDP/en/solrhe/chap9sec95.html)



## another computer running the IRL algorithm

rosinit('128.178.145.250','Nodename','IRL_wsp')

matlab
setenv('ROS_MASTER_URI', 'http://')
getenv()
setenv('ROS_IP','')


## Gripper
rosrun robotiq_c_model_control CModelTcpNode.py 192.168.1.11
https://github.com/epfl-lasa/lasa-wiki/wiki/Robotiq-gripper


rosrun gripper_c Gripper_c


## Some links

'https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/examples/index.htm'

'https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf'



## message

$ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

rostopic pub /topic package/msg_type '{x: 1, y: 2, c: a, type: string}'

rostopic pub /eeg_weight std_msgs/String 'data: "0.5"'