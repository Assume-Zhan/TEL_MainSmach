#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "geometry_msgs/Point.h"

class Calibration_State{

public:

    Calibration_State(){}

    void Init(ros::NodeHandle nh);

    void InitDockingPoint(std::string fileName);

    void StartCalibration(std::string type);

    geometry_msgs::Point GetCalibrationPoint(std::string type);

    const std::string DockingName[5] = {
        "DOCKING_POINT_1_1",
        "DOCKING_POINT_1_2",
        "DOCKING_POINT1_3",
        "DOCKING_BEFORE_2",
        "DOCKING_STAGE_3"
    };

private:

    std::vector<std::pair<geometry_msgs::Point, char>> DockingPoints;

    ros::ServiceClient DockingClient;
    ros::ServiceServer DockingFinServer;
};