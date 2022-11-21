#include "main_smach.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "main_smach_node");

	ros::NodeHandle main_state_machine_nh;

	MainSmach main_machine(main_state_machine_nh);

	main_machine.execute();
}
