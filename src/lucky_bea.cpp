#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	RNUtils::shutdown();
	RNUtils::printLn("Shutdown application Requested.");
	RNUtils::printLn("Lucky Bea: Quitting..." );	
	
	if(robot != NULL){
    	delete robot;
    }
	RNUtils::printLn("Succesfully closed...\n");
	Aria::exit(0);
}

int main(int argc, char** argv){
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
    RNUtils::init(argc, argv);
	
	RNUtils::printLn("Lucky Bea for Doris. To control the IC Group Robot named Doris");
	RNUtils::printLn("Press Ctrl+C to exit");
	
	std::string port = std::string(RN_DEFAULT_PORT);
	if(RNUtils::isVirtualScenarioActivated()){
		port = RNUtils::getVirtualScenarioPort();
	}

    robot = new GeneralController(port.c_str());
    if(robot->init("", 14004, SOCKET_SERVER) == 0){
    	robot->startThread();
	    RNUtils::printLn("No clients connected...");

		RNUtils::spin();
    }

		
    
    return 0;
}