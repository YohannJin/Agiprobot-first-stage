#pragma once

#include <iostream>

//ROS
#include <ros/ros.h>
#include "std_msgs/Bool.h"


enum DrillMode {FASTEN, LOSEN};

class DrillControl
{
private:
	ros::NodeHandle nh_DrillControl;
	ros::Subscriber sub_Ready_;
	ros::AsyncSpinner *spinner_Drillcontrol;

	bool isNotReady_ = false; //indicates if Drill is ready

	// updates isNotReady_ variable
	void setIsNotReady(const std_msgs::Bool::ConstPtr& msg);
	

public:
	DrillControl(int argc, char** argv);
	~DrillControl();

	/**
   * @brief Selects a programm on the drill controller.
   * @param mode FASTEN for programm 1 and LOSEN for programm 2.
   */
	void triggerProg(DrillMode mode);
	
	/**
   * @brief Start the currently selected programm.
   */
	void triggerStart();

	/**
   * @brief Checks the current status of the drill.
   * @returns a bool indicating if drill is ready or not.
   */
	bool isDrillReady();
};
