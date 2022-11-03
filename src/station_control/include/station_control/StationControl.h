#include "control_lib/TaskInterface.h"

#include "ros/ros.h"
#include "station_control/changeTool.h"
#include "station_control/gotoTP.h"


class StationControl
{
private:
ros::NodeHandle nh_station_control_;

int current_tool_ = 1;

public:
TaskInterface* taskInterface;

ros::ServiceServer service1;
ros::ServiceServer service2;

bool serviceGotoTP(station_control::gotoTP::Request  &req, station_control::gotoTP::Response &res);

bool serviceChangeTool(station_control::changeTool::Request  &req, station_control::changeTool::Response &res);

void rosSpinOnce();

StationControl(int argc, char** argv);

~StationControl();
};