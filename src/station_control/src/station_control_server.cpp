#include "station_control/StationControl.h"


int main(int argc, char** argv)
{
  ROS_ERROR_STREAM("is tool 1 installed? (y/n) ...");

  while(true)
  {
    char input;
    std::cin >> input;
    if(input == 'n')
    {
      ROS_INFO_STREAM("quitting ...");
      return 1;
    }
    else if (input == 'y')
    {
      ros::init(argc, argv, "station_control_server");
      StationControl stationControl(argc, argv);

      while(ros::ok())
      {
        stationControl.rosSpinOnce();
      }

      return 0;
    }
    else
    {
      ROS_INFO_STREAM("invalid answer try again: (y/n) ...");
    }
  }
}