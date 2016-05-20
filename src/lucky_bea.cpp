#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	RNUtils::printLn("Shutdown application Requested.");
	RNUtils::printLn("Lucky Bea: Quitting..." );	
	
	if(robot != NULL){
    	delete robot;
    }
    RNUtils::setStatus(false);
	RNUtils::printLn("Succesfully closed...\n");
	exit(1);

}

int main(int argc, char** argv){
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
    RNUtils::setStatus(true);
    RNUtils::setApplicationPathName(argv[0]);
	if(argc == 1 || argc == 3){
		RNUtils::printLn("Lucky Bea for Doris. To control the IC Group Robot named Doris");
		RNUtils::printLn("Press Ctrl+C to exit");
		std::string port = "/dev/ttyS0";
		if(argc == 3){
			if(strcmp(argv[1], "-p") == 0){
				port = std::string(argv[2]);
			}
		}
	    robot = new GeneralController(port.c_str());
	    if(robot->init("", 14004, SOCKET_SERVER) == 0){
	    	robot->startThread();
		    RNUtils::printLn("No clients connected...");

			RNUtils::spin();
	    }

	}	
    
    return 0;
}
