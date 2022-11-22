#include "arm_state.h"

void Arm_State::Init(ros::NodeHandle nh){
    // TODO : Init ros service and client in this function
}

void Arm_State::MoveArmCatching(std::map<char, geometry_msgs::Point> BlockPositions){

    robot_arm_control::GetObject ObjectSrv;

    for(auto block : this->CatchBlock){
        if(block.second == true || BlockPositions.count(block.first) <= 0) continue;

        ObjectSrv.request.x = BlockPositions[block.first].x;
        ObjectSrv.request.y = BlockPositions[block.first].y;
        ObjectSrv.request.z = BlockPositions[block.first].z;

        while(!this->arm_client.call(ObjectSrv)); // Client call // TODO : timeout

        if(ObjectSrv.response.isLegal == false) continue;

        this->CatchSuccessfully = false;
        while(this->CatchSuccessfully == false){
            // Arm is catching, waiting for callback
            // TODO : Set timeout
        }
        this->CatchBlock[block.first] = true;
    }

}
