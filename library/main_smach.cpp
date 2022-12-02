#include "main_smach.h"

MainSmach::MainSmach(ros::NodeHandle* nh){
    this->nh_ = nh;
    this->ResetLocal_cli = this->nh_->serviceClient<localization::Reset>("/Localization_Reset");

    // Stage control
    ros::param::get("~StartAtSecond", StartAtSecond_);
    ros::param::get("~StartAtThird", StartAtThird_);

    // Path prefix
    ros::param::get("~PathPrefix", PathPrefix_);

    // Navigation timeout and waiting rate
    ros::param::get("~NavigationTimeout", NavigationTimeout_);
    ros::param::get("~NavigationWaitRate", NavigationWaitRate_);

    // Calibration parameter
    ros::param::get("~CalibrationDistance", CalibrationDistance_);

    // Block navigation parameter
    ros::param::get("~ArmLength", ArmLength_);
    ros::param::get("~ArmCentralLength", ArmCentralLength_);
    ros::param::get("~HalfCarLength", HalfCarLength_);

    ros::param::get("~arm_y_offset", arm_y_offset);

    // Omega
    ros::param::get("~Omega", Omega);


    navigation.Init(nh, NavigationTimeout_, NavigationWaitRate_);
    camera.Init(nh, CameraTimeout_, CameraWaitRate_);
    arm.Init(nh);
    calibrate.Init(nh, this->PathPrefix_);
    pathTrace = new PathTrace();

    pathTrace->readPath(PathPrefix_ + "path.yaml");
}

void MainSmach::execute(){

    if(!StartAtSecond_ && !StartAtThird_)
        this->firstStage();

    // Trigger when Start At second but not third stage
    if(StartAtSecond_){
        geometry_msgs::Point pt;
        pt.x = 3.5;
        pt.y = -0.5;
        pt.z = 0;
        this->ResetLocalization(pt);
    }

    if(StartAtSecond_ || !StartAtThird_)
        this->secondStage();

    if(!StartAtSecond_ && StartAtThird_){
        geometry_msgs::Point pt;
        pt.x = 7.0;
        pt.y = -0.5;
        pt.z = 0;
        this->ResetLocalization(pt);
    }
    this->thirdStage();
}

void MainSmach::ResetLocalization(geometry_msgs::Point resetPoint){

    localization::Reset resetMes;

    resetMes.request.x = resetPoint.x;
    resetMes.request.y = resetPoint.y;
    resetMes.request.theta = resetPoint.z;
    while(!this->ResetLocal_cli.call(resetMes) && this->nh_->ok()); // TODO timeout

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
    if(this->navigation.MoveTo(this->pathTrace->getPath(FIRST_CATCH))){
        ROS_INFO_STREAM("STAGE 1 : NAVIGATION to first catch point");

        /* Docking in first point */
        // this->calibrate.StartCalibration(this->calibrate.DockingName[0]);
        // geometry_msgs::Point AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[0]);
        // this->ResetLocalization(AfterDocking);


        /* Catch Block */
        /* Using camera state to record the block position in camera state */
        arm.MoveArmCatching(StartPoint, CapturePicture);
        
        ros::Duration(2.0).sleep();
		
		ROS_INFO_STREAM("STAGE 1 : start to capture camera");
        this->camera.CatchBlocks();
        ROS_INFO_STREAM("STAGE 1: finished capture the image");

        /* Category the blocks */
        this->ClassifyBlocks(camera.GetBlockPositions());
        this->CatchQuadrantBlock(this->GetQuadrantPoint(0, 0), 0);
        this->CatchQuadrantBlock(this->GetQuadrantPoint(1, 0), 0);
        ROS_INFO_STREAM("STAGE 1 : finished the first catch");
    }

    /* Calibrate before second catch */
    if(this->navigation.MoveTo(this->pathTrace->getPath(BEFORE_SECOND_CATCH))){
        geometry_msgs::Point beforeSecondCatch = this->pathTrace->getCalibrationPoint(BEFORE_SECOND_CATCH);
        this->ResetLocalization(beforeSecondCatch);
        ROS_INFO_STREAM("STAGE 1 : Before second catch");
    }


    /* Move to second catch stage
       Get path -> by PathTracker
       Navigate to second catch point */
    if(this->navigation.MoveTo(this->pathTrace->getPath(SECOND_CATCH))){
        ROS_INFO_STREAM("STAGE 1 : NAVIGATION to second catch point");

        /* Docking in second point */
        // this->calibrate.StartCalibration(this->calibrate.DockingName[1]);
        // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[1]);
        // this->ResetLocalization(AfterDocking);


        arm.MoveArmCatching(StartPoint, CapturePicture);
	
        ros::Duration(2.0).sleep();

		ROS_INFO_STREAM("STAGE 1 : start to capture camera");
        this->camera.CatchBlocks();
        ROS_INFO_STREAM("STAGE 1: finished capture the image");

        /* Category the blocks */
        this->ClassifyBlocks(camera.GetBlockPositions());
        this->CatchQuadrantBlock(this->GetQuadrantPoint(0, 1), 1);
        this->CatchQuadrantBlock(this->GetQuadrantPoint(1, 1), 1);
        ROS_INFO_STREAM("STAGE 1 : finished the second catch");
    }


    /* Move to third catch stage
       Get path -> by PathTracker
       Navigate to third catch point */
    if(this->navigation.MoveTo(this->pathTrace->getPath(THIRD_CATCH))){
        ROS_INFO_STREAM("STAGE 1 : NAVIGATION to third catch point");

        arm.MoveArmCatching(StartPoint, CapturePicture);

    }


    if(this->navigation.MoveTo(this->pathTrace->getPath(PUT_BLOCK_BEFORE))){
        geometry_msgs::Point putBlockBefore = this->pathTrace->getCalibrationPoint(PUT_BLOCK_BEFORE);
        this->ResetLocalization(putBlockBefore);
        ROS_INFO_STREAM("STAGE 1 : NAVIGATION to put block point before");

        // this->calibrate.StartCalibration(this->calibrate.DockingName[2]);
        // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[2]);
        // this->ResetLocalization(AfterDocking);
    }


    /* Move to putting position */
    this->navigation.MoveTo(this->pathTrace->getPath(PUT_BLOCK));
    ROS_INFO_STREAM("STAGE 1 : NAVIGATION to put block point");

    ros::Duration(2.0).sleep();

    /* Put thing
      Using putting state */
    arm.MoveArmCatching(StartPoint, PUT_BLOCK_ARM);

    /* Docking */
    this->navigation.MoveTo(this->pathTrace->getPath(BEFORE_SECOND));
    ROS_INFO_STREAM("STAGE 1 : before second");
    // this->calibrate.StartCalibration(this->calibrate.DockingName[3]);
    // AfterDocking = this->calibrate.GetCalibrationPoint(this->calibrate.DockingName[3]);
    // this->ResetLocalization(AfterDocking);

    arm.MoveArmCatching(StartPoint, Back);
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

    ROS_INFO_STREAM("Start calibration");
    this->calibrate.StartCalibration("DOCKING_STAGE_3", CalibrationDistance_);
    geometry_msgs::Point CalibPoint = this->calibrate.GetCalibrationPoint("DOCKING_STAGE_3");
    this->ResetLocalization(CalibPoint);
    ROS_INFO_STREAM("End calibration");

    this->navigation.MoveTo(this->pathTrace->getPath(MOVE_OUT));
    ROS_INFO_STREAM("STAGE 3 : Move out the downward part");

    this->navigation.MoveTo(this->pathTrace->getPath(FINISHED));
    ROS_INFO_STREAM("STAGE 3 : Finished the race");
}


void MainSmach::ClassifyBlocks(std::map<char, geometry_msgs::Point> blocks){

    for(int i = 0; i < 4; i++) this->CategoryBlocks[i].clear();

    for(auto x : blocks){

        // Catch failed
        if(x.second.x == -1 && x.second.y == -1) continue;

        if(x.second.x <= 20 && x.second.y <= 20)
            this->CategoryBlocks[0].push_back(x.second);
        else if(x.second.x > 20 && x.second.y <= 20)
            this->CategoryBlocks[1].push_back(x.second);
        else if(x.second.x > 20 && x.second.y > 20)
            this->CategoryBlocks[2].push_back(x.second);
        else
            this->CategoryBlocks[3].push_back(x.second);
    }

    ROS_INFO_STREAM("STAGE 1 : quadrant 0 -> " << this->CategoryBlocks[0].size());
    ROS_INFO_STREAM("STAGE 1 : quadrant 1 -> " << this->CategoryBlocks[1].size());
    ROS_INFO_STREAM("STAGE 1 : quadrant 2 -> " << this->CategoryBlocks[2].size());
    ROS_INFO_STREAM("STAGE 1 : quadrant 3 -> " << this->CategoryBlocks[3].size());
}

std::queue<std::pair<geometry_msgs::Point, char>> MainSmach::GetQuadrantPoint(int quadrant, int type){
    std::queue<std::pair<geometry_msgs::Point, char>> Points;

    for(auto x : this->CategoryBlocks[quadrant]){
        geometry_msgs::Point tempMovePoint = x;
        geometry_msgs::Point MovePoint;
        if(type == 0){
            tempMovePoint.x = ((x.x + 2 - 1.5) / 100);
            tempMovePoint.y = (x.y / 100.) - this->ArmLength_; // TODO

            MovePoint.x = x.y; // 1.0 + tempMovePoint.y;
            MovePoint.y = -tempMovePoint.x - this->ArmCentralLength_; // Original 0 becomes -0.065

            Points.push({MovePoint, 'b'});

            ROS_INFO_STREAM("Quadrant 01, type : " << type << ", point : (" << MovePoint.x << ", " << MovePoint.y << ')');
        }
        else if(type == 1){
            tempMovePoint.x = (x.x / 100.);
            tempMovePoint.y = ((x.y + 2) / 100.) - this->ArmLength_;

            MovePoint.x = 1.0 + tempMovePoint.x + this->HalfCarLength_ + this->ArmCentralLength_;
            MovePoint.y = x.y; // tempMovePoint.y - 0.44 - this->HalfCarLength_ + this->ArmCentralLength_;
            MovePoint.z = this->Omega;

            Points.push({MovePoint, 'b'});

            ROS_INFO_STREAM("Quadrant 01, type : " << type << ", point : (" << MovePoint.x << ", " << MovePoint.y << ')');
        }
        else if(type == 2){
            tempMovePoint.x = ((x.x + 2) / 100) + 0;
            tempMovePoint.y = (x.y / 100.) - this->ArmLength_; // TODO

            MovePoint.x = x.x; // 1.42 - tempMovePoint.y + this->HalfCarLength_ + this->ArmCentralLength_;
            MovePoint.y = -0.44 + tempMovePoint.x - this->ArmCentralLength_;
            MovePoint.z = this->Omega;

            Points.push({MovePoint, 'b'});

            ROS_INFO_STREAM("Quadrant 01, type : " << type << ", point : (" << MovePoint.x << ", " << MovePoint.y << ')');
        }
    }

    return Points;
}

void MainSmach::CatchQuadrantBlock(std::queue<std::pair<geometry_msgs::Point, char>> blocks, int type){
    while(!blocks.empty() && this->nh_->ok()){
        std::queue<std::pair<geometry_msgs::Point, char>> points;
        points.push(blocks.front());
        if(type == 0){
            points.front().first.x = 0.97;
            points.front().second = 'b';
            this->navigation.MoveTo(points);

            geometry_msgs::Point pointToArm;
            pointToArm.x = 0;  // (type == 2) ? -20 : 0;
            pointToArm.y = std::max(blocks.front().first.x + 9, 14.); // (type == 2) ? 0 : 20;
            pointToArm.z = 4;
            arm.MoveArmCatching(pointToArm, Basic);

            ROS_INFO_STREAM("ARM MOVE TO : (" << pointToArm.x << ", " << pointToArm.y << ")");

            ROS_INFO_STREAM("NAV MOVE TO : (" << points.front().first.x << ", " << points.front().first.y << ")");

            blocks.pop();
        }
        else if(type == 1){
            points.front().first.y = -0.6;
            points.front().second = 'b';
            this->navigation.MoveTo(points);

            geometry_msgs::Point pointToArm;
            pointToArm.x = 0;  // (type == 2) ? -20 : 0;
            pointToArm.y = std::max(blocks.front().first.y + 9, 14.); // (type == 2) ? 0 : 20;
            pointToArm.z = 4;
            arm.MoveArmCatching(pointToArm, Basic);

            ROS_INFO_STREAM("ARM MOVE TO : (" << pointToArm.x << ", " << pointToArm.y << ")");

            ROS_INFO_STREAM("NAV MOVE TO : (" << points.front().first.x << ", " << points.front().first.y << ")");

            blocks.pop();
        }
        else if(type == 2){
            points.front().first.x = 1.68;
            points.front().second = 'b';
            this->navigation.MoveTo(points);

            geometry_msgs::Point pointToArm;
            pointToArm.x = 0;  // (type == 2) ? -20 : 0;
            pointToArm.y = std::max(blocks.front().first.y + 9, 14.); // (type == 2) ? 0 : 20;
            pointToArm.z = 4;
            arm.MoveArmCatching(pointToArm, Basic);

            ROS_INFO_STREAM("ARM MOVE TO : (" << pointToArm.x << ", " << pointToArm.y << ")");

            ROS_INFO_STREAM("NAV MOVE TO : (" << points.front().first.x << ", " << points.front().first.y << ")");

            blocks.pop();
        }
    }
}
