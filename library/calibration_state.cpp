#include "calibration_state.h"

void Calibration_State::Init(ros::NodeHandle nh){

}

void Calibration_State::InitDockingPoint(std::string fileName){
    YAML::Node pathConfig = YAML::LoadFile(fileName);

    this->DockingPoints.resize(this->DockingName->size() + 5);

    int i = 0;
    for(auto path : pathConfig){

        if(i >= this->DockingName->size()) break;

        auto pathName = path[this->DockingName[i]];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["xyz"][0].as<double>();
            point.y = xyz["xyz"][1].as<double>();
            point.z = xyz["xyz"][2].as<double>();

            this->DockingPoints[i] = {point, xyz["mode"].as<char>()};
        }

        i++;
    }
}

void Calibration_State::StartCalibration(std::string type){

}

geometry_msgs::Point Calibration_State::GetCalibrationPoint(std::string type){
    geometry_msgs::Point hello;
    return hello;
}
