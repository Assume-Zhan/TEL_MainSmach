#include "camera_state.h"

void Camera_State::Init(ros::NodeHandle nh, double timeout, double waitingRate){

    // Advertise client
    this->camera_client = nh.serviceClient<block_detector::ObjectPt_srv>("/block_detector_node/getObjectPts");

    this->InitBlocks();

    this->timeout = this->timeoutReload = timeout;
    this->waitingRate = waitingRate;

}

void Camera_State::InitBlocks(){
    std::string BlockNumber = "TEL";

    geometry_msgs::Point InitPoint;
    InitPoint.x = InitPoint.y = InitPoint.z = -1.0;

    for(char Block : BlockNumber){
        BlockPositions[Block] = InitPoint;
    }
}


std::map<char, geometry_msgs::Point> Camera_State::GetBlockPositions(){
    return this->BlockPositions;
}

void Camera_State::CatchBlocks(){

    this->InitBlocks();

    block_detector::ObjectPt_srv BlockInformation;

    BlockInformation.request.catchBlockPts = true;


    ros::Rate waiting(this->waitingRate);
    timeoutReload = timeout;
    while(!this->camera_client.call(BlockInformation)){

        timeoutReload -= (this->waitingRate != 0) ? 1. / this->waitingRate : 0.01;

        if(timeoutReload <= 0){
            ROS_ERROR_STREAM("Fail to connect to camera service");

            return;
        }

    }

    int idx = 0;
    for(char Block : BlockInformation.response.blockName){
        BlockPositions[Block] = BlockInformation.response.BlockPts[idx++];
    }

}


