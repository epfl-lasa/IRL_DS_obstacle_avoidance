#include "MotionGenerator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MotionGenerator");
  ros::NodeHandle n;
  float frequency = 500.0f;

  MotionGenerator motionGenerator(n,frequency);
 
  if (!motionGenerator.init()) 
  {
    return -1;
  }
  else
  {
    motionGenerator.run();
  }

  return 0;
  
}

