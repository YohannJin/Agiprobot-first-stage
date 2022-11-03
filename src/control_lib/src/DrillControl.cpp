#include "control_lib/DrillControl.h"

/*
Describtion on whats going on:
DrillControl uses ROS to publish messages and listens to topics
*/

// simple ROS callback funktion
void DrillControl::setIsNotReady(const std_msgs::Bool::ConstPtr& msg)
{
	// assign value to isNotReady_
	isNotReady_ = msg->data;
}

// Constructor of DrillControl
DrillControl::DrillControl(int argc, char** argv)
{
	ROS_INFO_STREAM("Starting DrillControl");

	// initiate ROS and create subscribers and an Async Spinner to update massages
	ros::init(argc, argv, "NucPC");
    sub_Ready_ = nh_DrillControl.subscribe("digital_input04", 3, &DrillControl::setIsNotReady, this, ros::TransportHints().tcpNoDelay());
	spinner_Drillcontrol = new ros::AsyncSpinner(0);
	spinner_Drillcontrol->start();
}

// Destructor of DrillControl
DrillControl::~DrillControl()
{	
	// delete pointes and set storage free
	delete spinner_Drillcontrol;
	spinner_Drillcontrol = nullptr;
}

// method to publish massages on different topics
void DrillControl::triggerProg(DrillMode mode)
{	
	// create publisher
  	ros::Publisher pubPG0 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output02", 1000);
  	ros::Publisher pubPG1 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output06", 1000);
  	ros::Publisher pubPG2 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output03", 1000);
  	ros::Publisher pubPG3 = nh_DrillControl.advertise<std_msgs::Bool>("digital_output07", 1000);

	// create massages
  	std_msgs::Bool outpPG0;
  	std_msgs::Bool outpPG1;
  	std_msgs::Bool outpPG2;
  	std_msgs::Bool outpPG3;
	
	// give massages values
	if(mode == FASTEN)
	{
  		outpPG0.data = true;
  		outpPG1.data = false;
  		outpPG2.data = false;
  		outpPG3.data = false;
	}
	if(mode == LOSEN)
	{
  		outpPG0.data = false;
  		outpPG1.data = true;
  		outpPG2.data = false;
  		outpPG3.data = false;
	}

	// publish massages with 10Hz 5 times for each value to make sure they are getting received
  	ros::Rate loop_rate(10);

  	int count0 = 0;
  	while (count0 < 5)
  	{
    	pubPG0.publish(outpPG0);

    	ros::spinOnce();

    	loop_rate.sleep();
    	count0++;
  	}
  
  	int count1 = 0;
  	while (count1 < 5)
  	{
    	pubPG1.publish(outpPG1);

    	ros::spinOnce();

    	loop_rate.sleep();
    	count1++;
  	}

  	int count2 = 0;
  	while (count2 < 5)
  	{
    	pubPG2.publish(outpPG2);

    	ros::spinOnce();

    	loop_rate.sleep();
    	count2++;
  	}

  	int count3 = 0;
  	while (count3 < 5)
  	{
    	pubPG3.publish(outpPG3);

    	ros::spinOnce();

    	loop_rate.sleep();
    	count3++;
  	}
}

// another method to publish a massage see method, triggerProg() for details
void DrillControl::triggerStart()
{
	ros::Publisher pubStart = nh_DrillControl.advertise<std_msgs::Bool>("digital_output04", 1000);
  	std_msgs::Bool outpStart;
  	outpStart.data = true;
  	ros::Rate loopRate(10);

  	int countStart = 0;
  	while (countStart < 5)
  	{
    	pubStart.publish(outpStart);

    	ros::spinOnce();

    	loopRate.sleep();
    	countStart++;
  	}

  	outpStart.data = false;

  	countStart = 0;
  	while (countStart < 5)
  	{
    	pubStart.publish(outpStart);

    	ros::spinOnce();

    	loopRate.sleep();
    	countStart++;
  	}
}

// because drill ready equals false and drill not ready equals true we have to revert the received status massage 
bool DrillControl::isDrillReady()
{
	if(isNotReady_ == true)
		return false;
	else
		return true;
}
