#include "calibration_state.h"

void Calibration_State::Init(ros::NodeHandle nh, std::string PathPrefix){
    this->DockingClient = nh.serviceClient<distance_to_wall::DockingStart>("/DockingStart");
    this->DockingFinServer = nh.advertiseService("/DockingFinish", &Calibration_State::FinishedCallback, this);

    this->InitDockingPoint(PathPrefix + "calibration_point.yaml");
}

void Calibration_State::InitDockingPoint(std::string fileName){
    YAML::Node pathConfig = YAML::LoadFile(fileName);

    this->DockingPoints.resize(this->DockingName->size() + 5);

    int i = 0;
    for(auto path : pathConfig){

        if(i >= this->DockingName->size()) break;

        auto pathName = path[this->DockingName[i]];
        ROS_INFO_STREAM("Now : " << pathName);
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

bool Calibration_State::FinishedCallback(distance_to_wall::DockingFinish::Request& req, distance_to_wall::DockingFinish::Response& res){
    this->FinishedCalibration = true;
    return true;
}

void Calibration_State::StartCalibration(std::string type, double requestDistance){

    this->FinishedCalibration = false;
    distance_to_wall::DockingStart req;

    req.request.Distance = requestDistance;

    ros::Rate wait(waitingRate);
    callTimeoutReload = callTimeout;
    while(!this->DockingClient.call(req)){
        this->callTimeoutReload -= (this->waitingRate != 0) ? 1. / this->waitingRate : 0.01;

        if(callTimeoutReload <= 0){

            ROS_ERROR_STREAM("SMACH : Fail to call the service");

            return;
        }
    }

    timeoutReload = timeout;
    while(!this->FinishedCalibration){
        this->timeoutReload -= (this->waitingRate != 0) ? 1. / this->waitingRate : 0.01;

        if(timeoutReload <= 0){

            ROS_ERROR_STREAM("SMACH : Fail to docking");

            return;
        }

        ros::spinOnce();
    }

    ROS_INFO_STREAM("Finished the calibration");
}

geometry_msgs::Point Calibration_State::GetCalibrationPoint(std::string type){
    for(int i = 0; i < this->DockingName->size(); i++){
        if(type == this->DockingName[i])
            return this->DockingPoints[i].first;
    }

    return this->DockingPoints[0].first;
}
