#include <ros/ros.h>

#include "std_msgs/Int16.h"

using namespace std;

static void ButtonCallback(const std_msgs::Int16::ConstPtr &msg);

double CurrentTime;
double LastKillTime;

static void StartFirst();
static void StartSecond();
static void StartThird();

int main(int argc, char **argv) {
    ros::init(argc, argv, "start_node");

    ros::NodeHandle start_nh;
    ros::Subscriber ButtonState_sub = start_nh.subscribe("/Button", 1, ButtonCallback);

    LastKillTime = ros::Time::now().toSec();
    ros::spin();
}

void ButtonCallback(const std_msgs::Int16::ConstPtr &msg) {
    CurrentTime = ros::Time::now().toSec();
    if (CurrentTime - LastKillTime < 2.0) {
        return;
    }

    if (msg->data == 1) {
        StartFirst();
    } else if (msg->data == 2) {
        StartSecond();
    } else if (msg->data == 3) {
        StartThird();
    }
    ROS_INFO_STREAM("Kill stage: " << msg->data << ".");
    LastKillTime = CurrentTime;
}

void StartFirst() {
    // system("rosnode kill /main_state_machine");
    // system("roslaunch main_state_machine service_on.launch &");
    // system("roslaunch main_state_machine stage_1.launch & ");
}

void StartSecond() {
    // system("rosnode kill /main_state_machine");
    // system("roslaunch main_state_machine service_on.launch &");
    // system("roslaunch main_state_machine stage_2.launch & ");
}

void StartThird() {
    // system("rosnode kill /main_state_machine");
    // system("roslaunch main_state_machine service_on.launch &");
    // system("roslaunch main_state_machine stage_3.launch & ");
}