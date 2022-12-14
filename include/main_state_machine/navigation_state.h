#pragma once

#include <queue>
#include <ros/ros.h>

#include "nav_mec/navMec_srv.h"
#include "nav_mec/navMec_fsrv.h"
#include "geometry_msgs/Point.h"

class Navigation_State{

public:

    Navigation_State(){}

    void Init(ros::NodeHandle* nh, double timeout, double sleepRate);

    bool MoveTo(std::queue<std::pair<geometry_msgs::Point, char>> pathWithMode);

private:

    bool navigation_callback(nav_mec::navMec_fsrvRequest& req, nav_mec::navMec_fsrvResponse& res);
    bool navigationFinished = false;
    bool running = false;

    double timeout, timeoutReload;
    double callTimeout = 3, callTimeoutReload = 3;
    double sleepRate = 100;

    ros::NodeHandle* nh_;
    ros::ServiceClient navigation_client;
    ros::ServiceServer navigation_server;

};