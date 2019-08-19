#include <gripper_c.h>

GripperC::GripperC(ros::NodeHandle &n, double frequency)
{
	// me = this;
	ROS_INFO_STREAM("The gripper c node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}

bool GripperC::init()
{
	_msgint= 0;

	gripper_.reset(new RSGripperInterface(false));
	ROS_INFO("[node-name] activating, wait 4 sec ...");
	gripper_->activate();
	ros::Duration(3.0).sleep();
	gripper_->setSpeed(250);
	gripper_->setPosition(128);
	ros::Duration(1.0).sleep();

	_sub = _n.subscribe("/gripper/out", 1, &GripperC::gripperCallBack, this, ros::TransportHints().reliable().tcpNoDelay());
	// _sub = _n.subscribe("/gripper/out", 1, &GripperC::gripperCallback);	

	_pub = _n.advertise<std_msgs::Int8>("/gripper/in", 1);

	if (_n.ok()) 
	{ 
		ros::spinOnce();
		ROS_INFO("The gripper c is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void GripperC::run()
{
	while (_n.ok()) 
	{

		sendGripperCommand();

		ros::spinOnce();
	}

	ros::shutdown();
}


void GripperC::gripperCallBack(const std_msgs::Int8::ConstPtr& msg)
{
	_msgint = msg->data;
	std::cout << "[gripper] recieve the command. " << std::endl;
}


void GripperC::sendGripperCommand()
{
	if (_msgint == 1)
	{
		ROS_INFO("[gripper] close the gripper");
		gripper_->setPosition(250); // to close
		ros::Duration(0.8).sleep(); // wait
	}
	else if (_msgint == 0)
	{
		ROS_INFO("[gripper] open the gripper");
		gripper_->setPosition(0); // to open
		ros::Duration(0.8).sleep(); // wait
	}
}