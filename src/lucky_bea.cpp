#include <signal.h>
#include "GeneralController.h"

GeneralController *robot;

bool continue_execution = true;

void signalHandler(int s){
	printf("Shutdown application Requested. Stopping Services...\n");
	continue_execution = false;
	robot->stopVideoStreaming();
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
	ros::Subscriber battery_voltage_state = nh.subscribe("/RosAria/battery_voltage", 1, &GeneralController::batteryVoltageCallback, robot);
	
	ros::Subscriber sonar_state = nh.subscribe("/RosAria/sonar", 1, &GeneralController::sonarStateCallback, robot);
	ros::Subscriber sonar_pointcloud2_state = nh.subscribe("/RosAria/sonar_pointcloud2", 1, &GeneralController::sonarPointCloud2StateCallback, robot);
	
	ros::Subscriber laser_state = nh.subscribe("/RosAria/laser", 1, &GeneralController::laserScanStateCallback, robot);
	ros::Subscriber laser_pointcloud_state = nh.subscribe("/RosAria/laser_pointcloud", 1, &GeneralController::laserPointCloudStateCallback, robot);
	
	ros::spin();
	
    /*while(continue_execution){
    }*/

    delete robot;
    ROS_INFO( "Lucky Bea: Quitting... \n" );
    return 0;
}
