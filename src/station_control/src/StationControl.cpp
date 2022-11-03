#include "station_control/StationControl.h"


bool StationControl::serviceGotoTP(station_control::gotoTP::Request  &req, station_control::gotoTP::Response &res)
{
  std::vector<double> goalPose = req.goalPose.poseVector;
  bool useMoveit = req.useMoveit;
  double velocity = req.velocity;
  double acceleration = req.acceleration;
  if (req.definedLocation == "HOME")
  {
    ROS_INFO_STREAM("executing serviceGotoTP(HOME)");
    goalPose = taskInterface->Robot_Task->home_position_;
    taskInterface->Robot_Task->gotoTP(HOME, useMoveit, velocity, acceleration);
  }
  else if(req.definedLocation == "CLAMPDEVICE")
  {
    ROS_INFO_STREAM("executing serviceGotoTP(CLAMPDEVICE)");
    goalPose = taskInterface->Robot_Task->clampdevice_position_;
    taskInterface->Robot_Task->gotoTP(CLAMPDEVICE, useMoveit, velocity, acceleration);
  }
  else if(req.definedLocation == "TOOLCHANGER")
  {
    ROS_INFO_STREAM("executing serviceGotoTP(TOOLCHANGER)");
    goalPose = taskInterface->Robot_Task->toolchanger_position_;
    taskInterface->Robot_Task->gotoTP(TOOLCHANGER, useMoveit, velocity, acceleration);
  }
  else
  {
    ROS_INFO_STREAM("executing serviceGotoTP(goalPose)");
    taskInterface->Robot_Task->gotoTP(goalPose, useMoveit, velocity, acceleration);
  }
  
  // check if goal reached with boundries (20 centimeter because of inaccuracy of moveit)
  std::vector<double> currentPose = taskInterface->Robot_Task->rtde_receive->getTargetTCPPose();
  std::string outputAnswer;
  for(int i = 0; i < 6; i++)
  {
    //ROS_INFO_STREAM(goalPose.at(i) << " / " << currentPose.at(i));
    double bound;
    if(useMoveit == true)
    {
      bound = 0.20;
    }
    else
    {
      bound = 0.001;
    }
    double deviation = abs(goalPose.at(i)) - abs(currentPose.at(i)); // ??? (problem for later)
    if(abs(deviation) < bound)
    {
      outputAnswer = "goal reached";
    }
    else
    {
      outputAnswer = "goal not reached";
      break;
    }
  }
  res.answer = outputAnswer;
  return true;
}

bool StationControl::serviceChangeTool(station_control::changeTool::Request  &req, station_control::changeTool::Response &res)
{
  if(current_tool_ == req.getFromSlot)
  {
    ROS_INFO_STREAM("tool already in use");
    return false;
  }
  
  int fromSlotInt = current_tool_ - 1;
  int toSlotInt = req.getFromSlot - 1;
  Slot fromSlotEnum = static_cast<Slot>(fromSlotInt);
  Slot toSlotEnum = static_cast<Slot>(toSlotInt);

  taskInterface->Robot_Task->changeTool(fromSlotEnum, toSlotEnum);

  ROS_INFO_STREAM("finished serviceChangeTool()");

  res.currentTool = req.getFromSlot;
  current_tool_ = req.getFromSlot;
  return true;
}

void StationControl::rosSpinOnce()
{
  ros::spinOnce();
}

StationControl::StationControl(int argc, char** argv)
{ 
  taskInterface = new TaskInterface(argc, argv);
  service1 = nh_station_control_.advertiseService("gotoTP", &StationControl::serviceGotoTP, this);
  service2 = nh_station_control_.advertiseService("changeTool", &StationControl::serviceChangeTool, this);
}
StationControl::~StationControl()
{
  delete taskInterface;
  taskInterface = nullptr;
}