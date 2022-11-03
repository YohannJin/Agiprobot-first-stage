#include "ros/ros.h"
#include "station_control/changeTool.h"


void deleteParam(ros::NodeHandle* nh)
{
    nh->deleteParam("/changeTool_client/fromSlot");
    nh->deleteParam("/changeTool_client/toSlot");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "changeTool_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<station_control::changeTool>("changeTool");
    station_control::changeTool srv;

    // check if all params are given
    if(!nh.hasParam("/changeTool_client/getFromSlot"))
    {
        ROS_ERROR_STREAM("param getfromSlot missing");
        deleteParam(&nh);
        return 1;
    }

    // assign params to values
    int inputGetFromSlot;
    nh.getParam("/changeTool_client/getFromSlot", inputGetFromSlot);

    // check if params are valid options, if true: assign request
    if(inputGetFromSlot >= 1 && inputGetFromSlot <= 3)
    {
        srv.request.getFromSlot = inputGetFromSlot;
    }
    else
    {
        ROS_ERROR_STREAM("inputGetFromSlot value invalid: must be between 1 and 3 but is " << inputGetFromSlot);
        deleteParam(&nh);
        return 1;
    }

    // call service and wait for response
    if (client.call(srv))
    {
        ROS_INFO_STREAM("Service changeTool called: Tool " << srv.response.currentTool << " installed");   
    }
    else
    {
        ROS_ERROR("Failed to call sevice changeTool");
        deleteParam(&nh);
        return 1;
    }

    deleteParam(&nh);
    return 0;
}