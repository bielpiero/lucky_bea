#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	printf("Shutdown application Requested. Stopping Services...\n");
	continue_execution = false;
	//robot->stopVideoStreaming();
	//robot->Close();
	if(ros::ok()){
		ros::shutdown();
	}
	printf("Succesfully closed...\n");
	exit(1);

}

int main(int argc, char** argv){
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
	
	ros::init(argc, argv, "lucky_bea");
	ros::start();
	ros::NodeHandle nh;
	ROS_INFO("Lucky Bea for Doris. To control the IC Group Robot named Doris");
	ROS_INFO("Press Ctrl+C to exit");
    robot = new GeneralController(nh);
    robot->Init("", 14004, SOCKET_SERVER);
    robot->StartThread();

	//robot->trackRobot();
	robot->OnConnection();
	ros::spin();
	ROS_INFO( "Lucky Bea: Quitting... \n" );
    delete robot;
    
    return 0;
}
