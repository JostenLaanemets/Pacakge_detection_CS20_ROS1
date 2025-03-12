#include <iostream>

#include <ros/ros.h>
#include "synexens_ros1/SYRosDevice.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synexens_bridge");

    std::shared_ptr<SYRosDevice> device(new SYRosDevice);

    SYErrorCode errorCode = device->startCameras();

    if (errorCode != Synexens::SYERRORCODE_SUCCESS)
    {
        ROS_ERROR_STREAM("Failed to start cameras");
        return -1;
    }

    ROS_INFO("ROS Exit Started");
    device.reset();
    ROS_INFO("ROS Exit");
    ros::shutdown();
    ROS_INFO("ROS Shutdown complete");

    return 0;
}