#include "main_smach.h"

MainSmach::MainSmach(ros::NodeHandle& nh){
    this->ResetLocal_cli = nh.serviceClient<localization::Reset>("/Localization_Reset");


    navigation.Init(nh, 210 /* Timeout */, 50 /* Service waiting rate */);
    camera.Init(nh);
    calibrate.Init(nh);
    pathTrace = new PathTrace();

    pathTrace->readPath("/home/ubuntu/catkin_ws/src/main_state_machine/path/path.yaml");
}

void MainSmach::execute(){
    geometry_msgs::Point pt;
    pt.x = 3.5;
    pt.y = -0.5;
    pt.z = 0;
    this->ResetLocalization(pt);
    this->firstStage();
    this->secondStage();
    // geometry_msgs::Point pt;
    // pt.x = 6.985;
    // pt.y = -0.5;
    // pt.z = 0;
    // this->ResetLocalization(pt);
    this->thirdStage();
}

void MainSmach::ResetLocalization(geometry_msgs::Point resetPoint){

    localization::Reset resetMes;

    resetMes.request.x = resetPoint.x;
    resetMes.request.y = resetPoint.y;
    resetMes.request.theta = resetPoint.z;
    while(!this->ResetLocal_cli.call(resetMes)); // TODO timeout

}

void MainSmach::firstStage(){

    ROS_INFO_STREAM("STAGE 1 : INIT");

    /* Reset the first position */
    geometry_msgs::Point StartPoint;
    StartPoint.x = 0;
    StartPoint.y = -0.5;
    StartPoint.z = 0;
    this->ResetLocalization(StartPoint);
    ROS_INFO_STREAM("STAGE 1 : RESET localization successfully");

    /* Move to first catch stage
       Get path -> by PathTracker
       Navigate to first catch point */
    this->navigation.MoveTo(this->pathTrace->getPath(FIRST_CATCH));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to first catch point");

    /* Docking in first point */
    // this->calibrate.StartCalibration(this->calibrate.DockingName[0]);
    // geometry_msgs::Point AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[0]);
    // this->ResetLocalization(AfterDocking);

    /* Catch Block */
    /* Using camera state to record the block position in camera state */
    ROS_INFO_STREAM("STAGE 1 : start to capture camera");
    this->camera.CatchBlocks();
    ROS_INFO_STREAM("STAGE 1: finished capture the image");

    /* Robot arm state
       Using robot arm state to catch the block
       Positions are recorded in camera state */



    /* Move to second catch stage
       Get path -> by PathTracker
       Navigate to second catch point */
    this->navigation.MoveTo(this->pathTrace->getPath(SECOND_CATCH));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to second catch point");

    /* Docking in second point */
    // this->calibrate.StartCalibration(this->calibrate.DockingName[1]);
    // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[1]);
    // this->ResetLocalization(AfterDocking);

    /* Catch Block
       Using camera state to record the block position in camera state */
    ROS_INFO_STREAM("STAGE 1 : start to capture camera");
    this->camera.CatchBlocks();
    ROS_INFO_STREAM("STAGE 1: finished capture the image");

    /* Robot arm state
       Using robot arm state to catch the block
       Positions are recorded in camera state */

    /* Docking */
    this->navigation.MoveTo(this->pathTrace->getPath(PUT_BLOCK_BEFORE));
    geometry_msgs::Point putBlockBefore = this->pathTrace->getCalibrationPoint(PUT_BLOCK_BEFORE);
    this->ResetLocalization(putBlockBefore);
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to put block point before");
    // this->calibrate.StartCalibration(this->calibrate.DockingName[2]);
    // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[2]);
    // this->ResetLocalization(AfterDocking);

    /* Move to putting position */
    this->navigation.MoveTo(this->pathTrace->getPath(PUT_BLOCK));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to put block point");

    /* Put thing
      Using putting state */

    /* Docking */
    this->navigation.MoveTo(this->pathTrace->getPath(BEFORE_SECOND));
    ROS_INFO_STREAM("STAGE 1 : before second");
    // this->calibrate.StartCalibration(this->calibrate.DockingName[3]);
    // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[3]);
    // this->ResetLocalization(AfterDocking);
}

void MainSmach::secondStage(){

    ROS_INFO_STREAM("STAGE 2 : GO in stage two");

    /* Ready for second stage */
    this->navigation.MoveTo(this->pathTrace->getPath(SECOND_READY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION to second ready point");

    /* Go inside 20 */
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_TWENTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside twenty");

    /* Reset the location */
    geometry_msgs::Point twentyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_TWENTY);
    this->ResetLocalization(twentyCalibrationPt);
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside twenty");

    /* Go inside 40 */
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_FORTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside forty");

    /* Reset the location */
    geometry_msgs::Point fortyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_FORTY);
    this->ResetLocalization(fortyCalibrationPt);
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside forty");

    /* Go inside 60 */
    this->navigation.MoveTo(this->pathTrace->getPath(INSIDE_SIXTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION inside sixty");

    /* Reset the location */
    geometry_msgs::Point sixtyCalibrationPt = this->pathTrace->getCalibrationPoint(INSIDE_SIXTY);
    this->ResetLocalization(sixtyCalibrationPt);
    ROS_INFO_STREAM("STAGE 2  : RESET at -> (x, y, z) -> (" << sixtyCalibrationPt.x << ", "
        << sixtyCalibrationPt.y << ", " << sixtyCalibrationPt.z << ")");
    ROS_INFO_STREAM("STAGE 2 : RESET localization inside sixty");

    /* Go outside 60 */
    this->navigation.MoveTo(this->pathTrace->getPath(OUT_SIXTY));
    ROS_INFO_STREAM("STAGE 2 : NAVIGATION outside sixty");
}

void MainSmach::thirdStage(){

    this->navigation.MoveTo(this->pathTrace->getPath(THIRD_READY));
    ROS_INFO_STREAM("STAGE 3 : Move to third ready point");

    this->navigation.MoveTo(this->pathTrace->getPath(TURBO_UP));
    ROS_INFO_STREAM("STAGE 3 : Turbo up mode for moving upward");

    this->navigation.MoveTo(this->pathTrace->getPath(FLAT_SLOWDOWN));
    ROS_INFO_STREAM("STAGE 3 : Slowdown for slow down");

    ros::Rate WaitForCalib(2);
    WaitForCalib.sleep();

    ROS_INFO_STREAM("Start calibration");
    this->calibrate.StartCalibration("DOCKING_STAGE_3");
    geometry_msgs::Point CalibPoint = this->calibrate.GetCalibrationPoint("DOCKING_STAGE_3");
    this->ResetLocalization(CalibPoint);
    ROS_INFO_STREAM("End calibration");

    this->navigation.MoveTo(this->pathTrace->getPath(MOVE_OUT));
    ROS_INFO_STREAM("STAGE 3 : Move out the downward part");

    this->navigation.MoveTo(this->pathTrace->getPath(FINISHED));
    ROS_INFO_STREAM("STAGE 3 : Finished the race");
}