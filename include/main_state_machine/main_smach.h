#pragma once

#include <ros/ros.h>
#include "navigation_state.h"
#include "camera_state.h"
#include "calibration_state.h"
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

    void ResetLocalization(geometry_msgs::Point resetPoint);


    ros::NodeHandle nh_;
    ros::ServiceClient ResetLocal_cli;

    // Navigation state
    Navigation_State navigation;

    // Path generator
    PathTrace* pathTrace;

    // Block detector
    Camera_State camera;

    // Docking 
    Calibration_State calibrate;

    // Start at which stage
    bool StartAtSecond_ = false;
    bool StartAtThird_ = false;

    // Navigation service parameter
    double NavigationTimeout_ = 100;
    double NavigationWaitRate_ = 50;

    // Path prefix
    std::string PathPrefix_ = "/home/ubuntu/catkin_ws/src/main_state_machine/path/";
};