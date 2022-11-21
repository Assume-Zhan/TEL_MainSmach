#include "navigation_state.h"

Navigation_State::Navigation_State(ros::NodeHandle nh, double timeout, double sleepRate){
    this->navigation_client = nh.serviceClient<nav_mec::navMec_srv>("navMec_trigger");
    this->navigation_server = nh.advertiseService("navMec_resp", Navigation_State::navigation_callback, this);

    this->timeout = timeout;
    this->timeoutReload = timeoutReload;

    this->navigationFinished = false;

    this->sleepRate = sleepRate;
}

bool Navigation_State::MoveTo(std::queue<std::pair<geometry_msgs::Point, char>> pathWithMode){

    this->running = true;

    // First trigger the server on navigation node
    nav_mec::navMec_srv::Request req;
    while(!pathWithMode.empty()){
        req.next.push_back(pathWithMode.front().first);
        req.mode.push_back(pathWithMode.front().second);

        pathWithMode.pop();
    }

    if(!this->navigation_client.call(req)){
        return false;
    }


    // Wait for service call back to set the trigger off or timeout
    ros::Rate rate(this->sleepRate);
    while(this->navigationFinished == false){
        timeoutReload -= (this->sleepRate != 0) ? 1 / this->sleepRate : 0;

        if(timeoutReload <= 0){
            nav_mec::navMec_srv::Request req_false;
            req_false.next.clear();
            req_false.mode.clear();

            this->navigation_client.call(req_false);

            break;
        }

        rate.sleep();
    }

    this->running = false;

    // Reset this state
    timeoutReload = timeout;

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