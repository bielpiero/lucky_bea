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
	
	ros::init(argc, argv, "bea_con_suerte");
	ros::start();
	ros::NodeHandle nh;
	ROS_INFO("Lucky Bea for Doris. To control the IC Group Robot named Doris");
	ROS_INFO("Press Ctrl+C to exit");
    robot = new GeneralController(nh);
    robot->Init("", 14004, SOCKET_SERVER);
    robot->StartThread();
    
	
	ros::Subscriber bumper_state = nh.subscribe("/RosAria/bumper_state", 100, &GeneralController::bumperStateCallback, robot);
	ros::Subscriber pose_state = nh.subscribe("/RosAria/pose", 1, &GeneralController::poseStateCallback, robot);
	ros::Subscriber battery_voltage_state = nh.subscribe("/RosAria/battery_voltage", 1, &GeneralController::batteryVoltageCallback, robot);
	ros::Subscriber battery_recharge_state = nh.subscribe("/RosAria/battery_recharge_state", 1, &GeneralController::batteryRechargeStateCallback, robot);

	ros::Subscriber goal_achievement_state = nh.subscribe("/RosAria/goal_achived", 1, &GeneralController::goalAchievementStateCallback, robot);
	
	//ros::Subscriber sonar_state = nh.subscribe("/RosAria/sonar", 1, &GeneralController::sonarStateCallback, robot);
	//ros::Subscriber sonar_pointcloud2_state = nh.subscribe("/RosAria/sonar_pointcloud2", 1, &GeneralController::sonarPointCloud2StateCallback, robot);
	
	ros::Subscriber laser_state = nh.subscribe("/RosAria/laser", 1, &GeneralController::laserScanStateCallback, robot);
	ros::Subscriber laser_pointcloud_state = nh.subscribe("/RosAria/laser_pointcloud", 1, &GeneralController::laserPointCloudStateCallback, robot);

	int i = 10;
	while (i > 0){
		ros::spinOnce();
		Sleep(100);
		i--;
	}
	robot->trackRobot();
	robot->OnConnection();
	ros::spin();
	
    //delete robot;
    ROS_INFO( "Lucky Bea: Quitting... \n" );
    return 0;
}
