#pragma once

#include <ros/ros.h>
#include "navigation_state.h"
#include "path_trace.h"
#include "localization/Reset.h"

/**
 * @brief Main state machine object
 *
 */
class MainSmach{

public:

    MainSmach(ros::NodeHandle& nh);

    void execute();

private:

    void firstStage();

    void secondStage();

    void thirdStage();


    ros::NodeHandle nh_;
    ros::ServiceClient ResetLocal_cli;

    // Navigation state
    Navigation_State navigation;

    // Path generator
    PathTrace* pathTrace;

};