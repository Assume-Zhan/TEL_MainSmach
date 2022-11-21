#include "main_smach.h"

MainSmach::MainSmach(ros::NodeHandle& nh){
    this->ResetLocal_cli = nh.serviceClient<localization::Reset>("/Localization_node/Localization_Reset");


    navigation = new Navigation_State(nh, 25 /* Timeout */, 50 /* Service waiting rate */);
    pathTrace = new PathTrace();
}

void MainSmach::execute(){
    this->firstStage();
}

void MainSmach::firstStage(){

    // Reset the first position
    localization::ResetRequest resetReq;
    resetReq.x = 0;
    resetReq.y = 0;
    resetReq.theta = 0;
    while(!this->ResetLocal_cli.call(resetReq));

    // Move to first catch stage
    // Get path -> by PathTracker
    // Navigate to first catch point
    this->navigation->MoveTo(this->pathTrace->getPath(FIRST_CATCH));

    // Catch Block
    // Using camera state to record the block position in camera state

    // Robot arm state
    // Using robot arm state to catch the block
    // Positions are recorded in camera state



    // Move to second catch stage
    // Get path -> by PathTracker
    // Navigate to second catch point
    this->navigation->MoveTo(this->pathTrace->getPath(SECOND_CATCH));

    // Catch Block
    // Using camera state to record the block position in camera state

    // Robot arm state
    // Using robot arm state to catch the block
    // Positions are recorded in camera state


    // Move to putting position
    this->navigation->MoveTo(this->pathTrace->getPath(PUT_BLOCK));

    // Put thing
    // Using putting state

    // SECOND stage READY !
}

void MainSmach::secondStage(){

    localization::ResetRequest resetReq;

    // Ready for second stage
    this->navigation->MoveTo(this->pathTrace->getPath(SECOND_READY));

    // Go inside 20
    this->navigation->MoveTo(this->pathTrace->getPath(INSIDE_TWENTY));

    // Reset the location
    geometry_msgs::Point twentyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_TWENTY);
    resetReq.x = twentyCalibrationPt.x;
    resetReq.y = twentyCalibrationPt.y;
    resetReq.theta = twentyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));

    // Go inside 40
    this->navigation->MoveTo(this->pathTrace->getPath(INSIDE_FORTY));

    // Reset the location
    geometry_msgs::Point twentyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_FORTY);
    resetReq.x = twentyCalibrationPt.x;
    resetReq.y = twentyCalibrationPt.y;
    resetReq.theta = twentyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));

    // Go inside 60
    this->navigation->MoveTo(this->pathTrace->getPath(INSIDE_SIXTY));

    // Reset the location
    geometry_msgs::Point twentyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_SIXTY);
    resetReq.x = twentyCalibrationPt.x;
    resetReq.y = twentyCalibrationPt.y;
    resetReq.theta = twentyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));

    // Go outside 60
    this->navigation->MoveTo(this->pathTrace->getPath(OUT_SIXTY));
}

void MainSmach::thirdStage(){

    this->navigation->MoveTo(this->pathTrace->getPath(THIRD_READY));

    this->navigation->MoveTo(this->pathTrace->getPath(TURBO_UP));
    this->navigation->MoveTo(this->pathTrace->getPath(FLAT_SLOWDOWN));

    this->navigation->MoveTo(this->pathTrace->getPath(MOVE_OUT));
    this->navigation->MoveTo(this->pathTrace->getPath(FINISHED));

}