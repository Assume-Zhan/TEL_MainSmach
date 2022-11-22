#pragma once

#include <ros/ros.h>
#include <map>

#include "robot_arm_control/GetObject.h"
#include "geometry_msgs/Point.h"

class Arm_State{

public:

    Arm_State(){};

    void Init(ros::NodeHandle nh);

    void MoveArmCatching(std::map<char, geometry_msgs::Point> BlockPositions);

private:

    ros::ServiceClient arm_client;
    ros::ServiceServer arm_server;

    bool CatchSuccessfully = false;

    std::map<char, bool> CatchBlock;

};