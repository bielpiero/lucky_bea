#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	printf("Shutdown application Requested. Stopping Services...\n");
	continue_execution = false;

	robot->Close();
	if(ros::ok())
	{
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
	std::cout << "Lucky Bea for Doris. To control the IC Group Robot named Doris" << std::endl << std::endl << "Press Ctrl+Z to exit." << std::endl;
	ros::init(argc, argv, "bea_con_suerte");
	ros::start();
	ros::NodeHandle nh;
	
    robot = new GeneralController(nh);
    robot->Init("", 14004, SOCKET_SERVER);
    robot->StartThread();
    robot->OnConnection();
	
	ros::Subscriber bumper_state = nh.subscribe("/RosAria/bumper_state", 100, &GeneralController::bumperStateCallback, robot);
	ros::Subscriber pose_state = nh.subscribe("/RosAria/pose", 1, &GeneralController::poseStateCallback, robot);
	
	ros::spin();
	
    while(continue_execution){
    }
    
    return 0;
}
