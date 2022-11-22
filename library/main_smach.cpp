#include "main_smach.h"

MainSmach::MainSmach(ros::NodeHandle& nh){
    this->ResetLocal_cli = nh.serviceClient<localization::Reset>("/Localization_node/Localization_Reset");


    navigation.Init(nh, 25 /* Timeout */, 50 /* Service waiting rate */);
    camera.Init(nh);
    pathTrace = new PathTrace();

    pathTrace->readPath("/home/assume/Desktop/TLE_Navigation/src/main_state_machine/path/path.yaml");
}

void MainSmach::execute(){
    this->firstStage();
    this->secondStage();
    this->thirdStage();
}

void MainSmach::firstStage(){

    ROS_INFO_STREAM("STAGE 1 : INIT");

    // Reset the first position
    localization::Reset resetReq;
    resetReq.request.x = 0;
    resetReq.request.y = -0.5;
    resetReq.request.theta = 0;
    while(!this->ResetLocal_cli.call(resetReq)); // TODO timeout
    ROS_INFO_STREAM("STAGE 1 : RESET localization successfully");

    // Move to first catch stage
    // Get path -> by PathTracker
    // Navigate to first catch point
    this->navigation.MoveTo(this->pathTrace->getPath(FIRST_CATCH));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to first catch point");

    // Catch Block
    // Using camera state to record the block position in camera state
    ROS_INFO_STREAM("STAGE 1 : start to capture camera");
    this->camera.CatchBlocks();

    // Robot arm state
    // Using robot arm state to catch the block
    // Positions are recorded in camera state



    // Move to second catch stage
    // Get path -> by PathTracker
    // Navigate to second catch point
    this->navigation.MoveTo(this->pathTrace->getPath(SECOND_CATCH));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to second catch point");
    this->camera.CatchBlocks();

    // Catch Block
    // Using camera state to record the block position in camera state

    // Robot arm state
    // Using robot arm state to catch the block
    // Positions are recorded in camera state


    // Move to putting position
    this->navigation.MoveTo(this->pathTrace->getPath(PUT_BLOCK));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to put block point");

    // Put thing
    // Using putting state

    // SECOND stage READY !
}

void MainSmach::secondStage(){

    ROS_INFO_STREAM("STAGE 2 : GO in stage two");

    localization::Reset resetReq;

    // Ready for second stage
    this->navigation.MoveTo(this->pathTrace->getPath(SECOND_READY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION to second ready point");

    // Go inside 20
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_TWENTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside twenty");

    // Reset the location
    geometry_msgs::Point twentyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_TWENTY);
    resetReq.request.x = twentyCalibrationPt.x;
    resetReq.request.y = twentyCalibrationPt.y;
    resetReq.request.theta = twentyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));  // TODO : timeout
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside twenty");

    // Go inside 40
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_FORTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside forty");

    // Reset the location
    geometry_msgs::Point fortyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_FORTY);
    resetReq.request.x = fortyCalibrationPt.x;
    resetReq.request.y = fortyCalibrationPt.y;
    resetReq.request.theta = fortyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));  // TODO : timeout
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside forty");

    // Go inside 60
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_SIXTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside sixty");

    // Reset the location
    geometry_msgs::Point sixtyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_SIXTY);
    resetReq.request.x = sixtyCalibrationPt.x;
    resetReq.request.y = sixtyCalibrationPt.y;
    resetReq.request.theta = sixtyCalibrationPt.z;
    while(!this->ResetLocal_cli.call(resetReq));  // TODO :timeout
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside sixty");

    // Go outside 60
    this->navigation.MoveTo(this->pathTrace->getPath(OUT_SIXTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION outside sixty");
}

void MainSmach::thirdStage(){

    this->navigation.MoveTo(this->pathTrace->getPath(THIRD_READY));

    this->navigation.MoveTo(this->pathTrace->getPath(TURBO_UP));
    this->navigation.MoveTo(this->pathTrace->getPath(FLAT_SLOWDOWN));

    this->navigation.MoveTo(this->pathTrace->getPath(MOVE_OUT));
    this->navigation.MoveTo(this->pathTrace->getPath(FINISHED));

}