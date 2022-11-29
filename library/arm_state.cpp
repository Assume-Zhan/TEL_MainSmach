#include "arm_state.h"

void Arm_State::Init(ros::NodeHandle nh){
    // TODO : Init ros service and client in this function
}

void Arm_State::MoveArmCatching(geometry_msgs::Point BlockPosition, CatchType type){

    robot_arm_control::GetObject ObjectSrv;

    switch(type){
        case Basic:
        default:
            ObjectSrv.request.x = BlockPosition.x;
            ObjectSrv.request.y = BlockPosition.y;
            ObjectSrv.request.z = BlockPosition.z;
            break;
        case CapturePicture:
            ObjectSrv.request.x = -1;
            ObjectSrv.request.y = -1;
            ObjectSrv.request.z = -1;
            break;
        case Back:
            ObjectSrv.request.x = 0;
            ObjectSrv.request.y = 0;
            ObjectSrv.request.z = 0;
            break;
    }

    // Init before catching
    this->CatchSuccessfully = false;

    // Send request with timeout
    ros::Rate waitingRate(this->waitingRate_);
    callTimeoutReload = callTimeout;
    while(!this->arm_client.call(ObjectSrv)){
        this->callTimeoutReload -= (this->waitingRate_ != 0) ? 1. / this->waitingRate_ : 0.01;

        if(callTimeoutReload <= 0){

            ROS_INFO_STREAM("Fail to call the arm service");

            return;
        }
    }

    // Wait for successful catch message
    timeoutReload = timeout;
    while(this->CatchSuccessfully == false){
        this->timeoutReload -= (this->waitingRate_ != 0) ? 1. / this->waitingRate_ : 0.01;

        if(callTimeoutReload <= 0){

            ROS_INFO_STREAM("Fail to catch blocks");

            return;
        }

        ros::spinOnce();
    }

    return;
}

bool Arm_State::arm_callback(robot_arm_control::GetObjectRequest& req, robot_arm_control::GetObjectResponse& res){
    this->CatchSuccessfully = true;
}

