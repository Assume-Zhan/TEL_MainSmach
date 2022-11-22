#include "camera_state.h"

void Camera_State::Init(ros::NodeHandle nh){

    // Advertise client
    this->camera_client = nh.serviceClient<block_detector::ObjectPt_srv>("/block_detector_node/getObjectPts");

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

    block_detector::ObjectPt_srv BlockInformation;

    BlockInformation.request.catchBlockPts = true;

    int idx = 0;
    if(this->camera_client.call(BlockInformation)){  // TODO : timeout

        for(char Block : BlockInformation.response.blockName){
            BlockPositions[Block] = BlockInformation.response.BlockPts[idx++];
        }

    }
    else{
        ROS_ERROR_STREAM("Fail to connect to camera service");
    }
}


