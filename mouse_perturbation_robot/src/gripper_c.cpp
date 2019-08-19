#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <grasp_interface/rs_gripper_interface.h>


void chatterCallBack(const std_msgs::Int8::ConstPtr& msg)
{
	std_msgs::Int8 msgint = msg->data;
	if (msgint == 1)
	{
		ROS_INFO("[gripper] close the gripper");
		gripper_->setPosition(250); // to close
		ros::Duration(0.8).sleep(); // wait
	}
	else if (msgint == 0)
	{
		ROS_INFO("[gripper] open the gripper");
		gripper_->setPosition(0); // to open
		ros::Duration(0.8).sleep(); // wait
	}
}


int main()
{
	ros::init(argc, argv, "Gripper_c");

	std::unique_ptr<RSGripperInterface> gripper_;
	ros::NodeHandle n;

	gripper_.reset(new RSGripperInterface(false));
	ROS_INFO("[node-name] activating, wait 4 sec ...");
	gripper_->activate();
	ros::Duration(3.0).sleep();
	gripper_->setSpeed(250);
	gripper_->setPosition(128);
	ros::Duration(1.0).sleep();

	ros::Subscriber sub = n.subscribe("/gripper/out", 1000, chatterCallback);

	ros::spin();

	while (ros::ok())
	{
	
	}

	return 0
}