#include "RobotNode.h"

RobotNode::RobotNode(const char* port){
	Aria::init();

	robot = new ArRobot();

	ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
	args->add("-robotPort"); // pass robot's serial port to Aria
    args->add(port);

  	ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  	argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
  	argparser->addDefaultArgument("-connectLaser -lp /dev/ttyS2 -lrb 1ref -ld 180 -li half -lf true"); //-lp /dev/ttyS2 port to connect

	connector = new ArRobotConnector(argparser, robot);

	connector->connectRobot();

	robot->enableMotors();
    robot->disableSonar();   

    robot->runAsync(true);

	this->maxTransVel = robot->getTransVelMax();
	this->maxAbsoluteTransVel = robot->getAbsoluteMaxTransVel();
	this->maxRotVel = robot->getRotVelMax();
	this->maxAbsoluteRotVel = robot->getAbsoluteMaxRotVel();
    
    isDirectMotion = false;
    isGoingForward = false;
    wasDeactivated = false;
    doNotMove = false;
    
    gotoPoseAction = new ArActionGoto("goto", ArPose(0.0, 0.0, 0.0), 100, 100, 100);
 	robot->addAction(gotoPoseAction, 89); 


 	laserConnector = new ArLaserConnector(argparser, robot, connector);
	if(!laserConnector->connectLasers(false, false, true)){
		printf("Could not connect to configured lasers.\n");
	}

	laser = robot->findLaser(1);
	sick = (ArSick*)laser;
	if(!laser){
		printf("Error. Not Connected to any laser.\n");
	} else {
		printf("Connected to SICK LMS200 laser.\n");
	}
}

RobotNode::~RobotNode(){
	sick->disconnect();
	robot->disableMotors();
	robot->disableSonar();
	robot->stopRunning();
	robot->waitForRunExit();
	Aria::shutdown();
}

LaserScan* RobotNode::getLaserScan(void){
    LaserScan* data = NULL;
	robot->lock();
	sick = (ArSick*)laser;
	if(sick != NULL){
		sick->lockDevice();

		std::vector<ArSensorReading> *currentReadings = sick->getRawReadingsAsVector();
		ArDrawingData* draw = sick->getCurrentDrawingData();
		data = new LaserScan();
		for(size_t it = 0; it < currentReadings->size(); it++){
			data->addLaserScanData((float)currentReadings->at(it).getRange() / 1000, currentReadings->at(it).getExtraInt());
			data->setScanPrimaryColor(draw->getPrimaryColor().getRed(), draw->getPrimaryColor().getGreen(), draw->getPrimaryColor().getBlue());
			draw++;
	        
		}
        sick->unlockDevice();
	}
	robot->unlock();
    return data;
}

void RobotNode::getRobotPosition(){
	myPose = new ArPose(robot->getPose());
	//printf("Robot Pose: {x: %lf m, y: %lf m, theta: %lf rad}\n", myPose->getX()/1e3, myPose->getY()/1e3, myPose->getThRad());
}

void RobotNode::stop(){
    robot->lock();
    robot->stop();
    robot->unlock();
}

void RobotNode::move(double distance, double speed){
	robot->lock();
	robot->setAbsoluteMaxTransVel(speed);

	robot->move(distance);

	robot->setAbsoluteMaxTransVel(this->maxAbsoluteTransVel);
	robot->unlock();

}

void RobotNode::moveAtSpeed(double linearVelocity, double angularVelocity){
    isDirectMotion = true;
    isGoingForward = false;
    robot->lock();
    gotoPoseAction->cancelGoal();
    if(linearVelocity == 0.0 && angularVelocity == 0.0){
        robot->stop();
        isDirectMotion = false;
    } else {
        if(linearVelocity > 0.0){
            isGoingForward = true;
        }
        
        robot->setVel(linearVelocity*1e3);

        robot->setRotVel(angularVelocity*180/M_PI);
    }
    
    robot->unlock();
}

void RobotNode::gotoPosition(double x, double y, double theta, double transSpeed, double rotSpeed){
    ArPose newPose(x * 1000, y * 1000);
    newPose.setThRad(theta);
    robot->lock();
    isDirectMotion = false;
    
    robot->setAbsoluteMaxTransVel(transSpeed);
    robot->setAbsoluteMaxRotVel(rotSpeed);
    robot->clearDirectMotion();
    
    if(x == 0.0 && y == 0.0 && theta != 0.0){
        robot->setHeading(theta * 180/M_PI);
    } else {
        gotoPoseAction->setGoal(newPose);
    }
    
    robot->setAbsoluteMaxTransVel(this->maxAbsoluteTransVel);
    robot->setAbsoluteMaxRotVel(this->maxAbsoluteRotVel);
    
    robot->unlock();
    ArUtil::sleep(100);
}

void RobotNode::setRobotPosition(double x, double y, double theta){
    ArPose newPose(x *1000, y * 1000);
    newPose.setThRad(theta);
    robot->lock();
    robot->clearDirectMotion();
    robot->moveTo(newPose);
    robot->unlock();
}

void RobotNode::setMotorsStatus(bool enabled){
    robot->lock();
    if (enabled) {
        if(!robot->isEStopPressed()){
            robot->enableMotors();
        }
        
    } else {
        robot->disableMotors();
    }
    robot->unlock();
}

bool RobotNode::getMotorsStatus(){
    return robot->areMotorsEnabled();
}

bool RobotNode::isGoalAchieved(){
    return gotoPoseAction->haveAchievedGoal();
}

std::vector<bool> RobotNode::getFrontBumpersStatus(void){
    std::vector<bool> bumpers(robot->getNumFrontBumpers());
    int stall = robot->getStallValue();
    unsigned char front_bumpers = (unsigned char)(stall >> 8);

    for (unsigned int i = 0; i < robot->getNumFrontBumpers(); i++){
        bumpers.at(i) = (front_bumpers & (1 << (i + 1))) == 0 ? false : true;
    }
    return bumpers;
}

std::vector<bool> RobotNode::getRearBumpersStatus(void){
    std::vector<bool> bumpers(robot->getNumRearBumpers());
    int stall = robot->getStallValue();

    unsigned char rear_bumpers = (unsigned char)(stall);
    for (unsigned int i = 0; i < robot->getNumRearBumpers(); i++){
        bumpers.at(i) = (rear_bumpers & (1 << (robot->getNumRearBumpers() - i))) == 0 ? false : true;
    }
    return bumpers;
}