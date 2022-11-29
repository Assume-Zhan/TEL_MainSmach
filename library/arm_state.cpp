#include "arm_state.h"

void Arm_State::Init(ros::NodeHandle nh){
    this->arm_client = nh.serviceClient<robot_arm_control::GetObject>("/GetObject_service");
    this->arm_server = nh.advertiseService("/RobotArm_ServiceFinish", &Arm_State::arm_callback, this);

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

            ROS_ERROR_STREAM("SMACH : Fail to call the arm service");

            return;
        }

        waitingRate.sleep();
    }

    ROS_INFO_STREAM("SMACH : arm going to (" << ObjectSrv.request.x << ", " << ObjectSrv.request.y << ")");

    // Wait for successful catch message
    timeoutReload = timeout;
    while(this->CatchSuccessfully == false){
        this->timeoutReload -= (this->waitingRate_ != 0) ? 1. / this->waitingRate_ : 0.01;

        if(timeoutReload <= 0){

            ROS_ERROR_STREAM("SMACH : Fail to catch blocks");

            return;
        }

        waitingRate.sleep();

        ros::spinOnce();
    }

    return;
}

bool Arm_State::arm_callback(robot_arm_control::ServiceFinishRequest& req, robot_arm_control::ServiceFinishResponse& res){
    this->CatchSuccessfully = true; // TODO
    return true;
}

