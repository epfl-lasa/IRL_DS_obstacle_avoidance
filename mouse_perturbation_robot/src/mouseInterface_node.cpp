#include "MouseInterface.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "MouseInterface");

  ros::NodeHandle n;
  float frequency = 500.0f;

  // Read
  std::string eventPath;

  if (argc == 2)
  {
      /* code */
    eventPath = argv[1];
  }
  else
  {
    std::cerr << "Please specify event path: /dev/input/event.. " << std::endl;
    return 0;
  }

  MouseInterface mouseInterface(n,frequency);
 
  if (!mouseInterface.init(eventPath)) 
  {
    return -1;
  }
  else
  {
    mouseInterface.run();
  }

  return 0;

}