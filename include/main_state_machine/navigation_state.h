#pragma once

#include <queue>
#include <ros/ros.h>

#include "nav_mec/navMec_srv.h"
#include "nav_mec/navMec_fsrv.h"
#include "geometry_msgs/Point.h"

class Navigation_State{

public:

    Navigation_State(ros::NodeHandle nh, double timeout, double sleepRate);

    bool MoveTo(std::queue<std::pair<geometry_msgs::Point, char>> pathWithMode);

private:

    bool navigation_callback(nav_mec::navMec_fsrvRequest& req, nav_mec::navMec_fsrvResponse& res);
    bool navigationFinished = false;
    bool running = false;

    double timeout, timeoutReload;
    double sleepRate = 50;

    ros::ServiceClient navigation_client;
    ros::ServiceServer navigation_server;

};