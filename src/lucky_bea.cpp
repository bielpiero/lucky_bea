#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	printf("Shutdown application Requested. Stopping Services...\n");
	continue_execution = false;
	//robot->stopVideoStreaming();
	//robot->Close();
	if(robot != NULL){
    	delete robot;
    	robot = NULL;
    }
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

	if(argc == 1 || argc == 3){
		ROS_INFO("Lucky Bea for Doris. To control the IC Group Robot named Doris");
		ROS_INFO("Press Ctrl+C to exit");
		std::string port = "/dev/ttyS0";
		if(argc == 3){
			if(strcmp(argv[1], "-p") == 0){
				port = std::string(argv[2]);
			}
		}
	    robot = new GeneralController(nh, port.c_str());
	    if(robot->init("", 14004, SOCKET_SERVER) == 0){
	    	robot->startThread();
		    ROS_INFO("No clients connected...");
			//robot->trackRobot();
			//robot->OnConnection();
			ros::spin();
			ROS_INFO( "Lucky Bea: Quitting... \n" );	
	    }
	    if(robot != NULL){
	    	delete robot;
	    	robot = NULL;
	    }

	}	
    
    return 0;
}
