#ifndef __GRIPPER_C_H__
#define __GRIPPER_C_H__

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <grasp_interface/rs_gripper_interface.h>
#include <iostream>
#include <memory>


class GripperC
{
private:
	int _msgint;

	std::unique_ptr<RSGripperInterface> gripper_;

	ros::NodeHandle _n;

	ros::Subscriber _sub;

	ros::Publisher _pub;

	static GripperC* me;

public:
	GripperC(ros::NodeHandle &n, double frequency);

	bool init();

	void run();

private:
	void gripperCallBack(const std_msgs::Int8::ConstPtr& msg);

	void sendGripperCommand();

};

#endif