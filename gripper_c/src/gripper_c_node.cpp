#include <gripper_c.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Gripper_c");
	ros::NodeHandle n;
	float frequency = 100.0f;

	GripperC gripperC(n, frequency);

	if (!gripperC.init()) 
	{
		return -1;
	}
	else
	{
		gripperC.run();
	}

	return 0;
  
}
