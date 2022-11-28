#include "navigation_state.h"

void Navigation_State::Init(ros::NodeHandle nh, double timeout, double sleepRate){
    this->navigation_client = nh.serviceClient<nav_mec::navMec_srv>("/navMec_trigger");
    this->navigation_server = nh.advertiseService("/navMec_resp", &Navigation_State::navigation_callback, this);

    this->timeout = timeout;
    this->timeoutReload = timeout;

    this->navigationFinished = false;

    this->sleepRate = sleepRate;
}

bool Navigation_State::MoveTo(std::queue<std::pair<geometry_msgs::Point, char>> pathWithMode){

    this->running = true;
    ROS_INFO_STREAM("NAVIGATION : running");

    // First trigger the server on navigation node
    nav_mec::navMec_srv req;
    while(!pathWithMode.empty()){
        req.request.next.push_back(pathWithMode.front().first);
        req.request.mode.push_back(pathWithMode.front().second);

        pathWithMode.pop();
    }

    // Call the navigation client
    ros::Rate(this->sleepRate);
    this->callTimeoutReload = this->callTimeout;
    while(!this->navigation_client.call(req)){
        this->callTimeoutReload -= (this->sleepRate != 0) ? 1. / this->sleepRate : 0.01;

        if(this->callTimeoutReload <= 0){
            this->callTimeoutReload = this->callTimeout;

            ROS_INFO_STREAM("Fail to call nav service");

            return false;
        }
    }

    // Wait for service call back to set the trigger off or timeout
    ros::Rate rate(this->sleepRate);
    this->timeoutReload = this->timeout;
    while(this->navigationFinished == false){
        this->running = true;
        timeoutReload -= (this->sleepRate != 0) ? 1. / this->sleepRate : 0.01;

        if(timeoutReload <= 0){
            nav_mec::navMec_srv req_false;
            req_false.request.next.clear();
            req_false.request.mode.clear();

            this->navigation_client.call(req_false);

            ROS_INFO_STREAM("Failed to navigation to goal point");

            timeoutReload = timeout;

            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    this->running = false;
    ROS_INFO_STREAM("NAVIGATION : stop running");
    this->navigationFinished = false;

    return true;
}

bool Navigation_State::navigation_callback(nav_mec::navMec_fsrvRequest& req, nav_mec::navMec_fsrvResponse& res){
    if(req.finished == true && this->running){
        this->navigationFinished = true;
        this->running = false;
    }

    res.reset = true;

    return true;
}