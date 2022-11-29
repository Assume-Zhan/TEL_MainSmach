#pragma once

#include <ros/ros.h>
#include <map>

#include "robot_arm_control/GetObject.h"
#include "robot_arm_control/ServiceFinish.h"
#include "geometry_msgs/Point.h"

typedef enum CatchType{

    Basic = 0,
    CapturePicture,
    Back

} CatchType;

class Arm_State{

public:

    Arm_State(){};

    void Init(ros::NodeHandle nh);

    void MoveArmCatching(geometry_msgs::Point BlockPosition, CatchType type);

private:

    // Callback function
    bool arm_callback(robot_arm_control::ServiceFinishRequest& req, robot_arm_control::ServiceFinishResponse& res);

    ros::ServiceClient arm_client;
    ros::ServiceServer arm_server;

    bool CatchSuccessfully = false;

    std::map<char, bool> CatchBlock;

    double callTimeout = 3;
    double callTimeoutReload = 3;
    double waitingRate_ = 50;

    double timeout = 20;
    double timeoutReload = 20;

};