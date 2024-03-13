
#include "moveit/move_group/move_group.h"
#include "log_helper/log_helper.h"
#include <cstring>
#include <iostream>

int main(int argc, char** argv)
{
  bool debug = false;
  for (int i = 1; i < argc; ++i)
    if (strncmp(argv[i], "--debug", 7) == 0)
    {
      debug = true;
      break;
    }
  if (debug)
	  //ROS_INFO("MoveGroup debug mode is ON");
	  std::cout << ("MoveGroup debug mode is ON") << std::endl;
  else
	  //ROS_INFO("MoveGroup debug mode is OFF");
	  std::cout << "MoveGroup debug mode is OFF" << std::endl;

  move_group::MoveGroup moveGroup;
  moveGroup.init(debug);

  std::cout << "Move Group Is running. Press Any Key to Exit." << std::endl;
  getchar();

  return 0;
}