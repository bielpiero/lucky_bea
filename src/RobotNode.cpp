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

    this->prevBatteryChargeState = -5;

	this->maxTransVel = robot->getTransVelMax();
	this->maxAbsoluteTransVel = robot->getAbsoluteMaxTransVel();
	this->maxRotVel = robot->getRotVelMax();
	this->maxAbsoluteRotVel = robot->getAbsoluteMaxRotVel();
    
    isDirectMotion = false;
    isGoingForward = false;
    wasDeactivated = false;
    doNotMove = false;

    this->prevLeftEncoderData = 0;
    this->prevRightEncoderData = 0;
    this->isFirstFakeEstimation = true;

    robot->requestEncoderPackets();
    myRawPose = new ArPose(0.0, 0.0, 0.0);
    
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

    pthread_t sensorDataThread;
    pthread_create(&sensorDataThread, NULL, dataPublishingThread, (void *)(this)  );

    pthread_t distanceThread;
    //pthread_create(&distanceThread, NULL, securityDistanceThread, (void *)(this)  );
}

RobotNode::~RobotNode(){
	
}

void RobotNode::disconnect(){
    sick->disconnect();
    robot->disableMotors();
    robot->disableSonar();
    robot->stopRunning();
    robot->waitForRunExit();
    Aria::shutdown();
}

void RobotNode::getLaserScan(void){
    LaserScan* data = NULL;
	//robot->lock();
	sick = (ArSick*)laser;
	if(sick != NULL){
		sick->lockDevice();
        
		std::vector<ArSensorReading> *currentReadings = sick->getRawReadingsAsVector();
		ArDrawingData* draw = sick->getCurrentDrawingData();

		data = new LaserScan();
		for(size_t it = 0; it < currentReadings->size(); it++){
			data->addLaserScanData((float)currentReadings->at(it).getRange() / 1000, (float)currentReadings->at(it).getExtraInt());
			data->setScanPrimaryColor(draw->getPrimaryColor().getRed(), draw->getPrimaryColor().getGreen(), draw->getPrimaryColor().getBlue());
			draw++;
		}
        sick->unlockDevice();
        onLaserScanCompleted(data);
	}
	//robot->unlock();
}

void RobotNode::getRobotPosition(){
	myPose = new ArPose(robot->getPose());
    onPositionUpdate(myPose->getX()/1e3, myPose->getY()/1e3, myPose->getThRad(), robot->getVel()/1e3, robot->getRotVel()*M_PI/180);
}

void RobotNode::stopRobot(){
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

void RobotNode::setPosition(double x, double y, double theta){
    ArPose newPose(x *1000, y * 1000);
    newPose.setThRad(theta);
    robot->lock();
    robot->clearDirectMotion();
    myRawPose->setPose(x*1000, y*1000);
    myRawPose->setThRad(theta);
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

void RobotNode::setSonarStatus(bool enabled){
    robot->lock();
    if (enabled) {
        robot->enableSonar();        
    } else {
        robot->disableSonar();
    }
    robot->unlock();
}

bool RobotNode::getSonarsStatus(){
    return robot->areSonarsEnabled();
}

void RobotNode::getBumpersStatus(void){
    std::vector<bool> frontBumpers(robot->getNumFrontBumpers());
    std::vector<bool> rearBumpers(robot->getNumRearBumpers());

    int stall = robot->getStallValue();
    unsigned char front_bumpers = (unsigned char)(stall >> 8);
    unsigned char rear_bumpers = (unsigned char)(stall);

    for (unsigned int i = 0; i < robot->getNumFrontBumpers(); i++){
        frontBumpers.at(i) = (front_bumpers & (1 << (i + 1))) == 0 ? false : true;
    }

    for (unsigned int i = 0; i < robot->getNumRearBumpers(); i++){
        rearBumpers.at(i) = (rear_bumpers & (1 << (robot->getNumRearBumpers() - i))) == 0 ? false : true;
    }

    onBumpersUpdate(frontBumpers, rearBumpers);
}

void RobotNode::getSonarsScan(void){
    if(getSonarsStatus()){
        std::vector<PointXY*>* data = new std::vector<PointXY*>();
        for (int i = 0; i < robot->getNumSonar(); i++) {
            ArSensorReading* reading = NULL;
            reading = robot->getSonarReading(i);
            if(reading) {
                data->push_back(new PointXY(reading->getLocalX() / 1000.0, reading->getLocalY() / 1000.0));
            }
        }
        onSonarsDataUpdate(data);
    }
}

void RobotNode::computePositionFromEncoders(){
    long int rightEncoderData = getRightEncoder();
    long int leftEncoderData = getLeftEncoder();

    if(isFirstFakeEstimation){
        prevRightEncoderData = rightEncoderData;
        prevLeftEncoderData = leftEncoderData;
        isFirstFakeEstimation = false;
    }

    double newLeftEncoderData = (double)leftEncoderData;
    double newRightEncoderData = (double)rightEncoderData;
    double newPrevLeftEncoderData = (double)prevLeftEncoderData;
    double newPrevRightEncoderData = (double)prevRightEncoderData;

    bool resultCheck[4];
    resultCheck[0] = checkForwardLimitTransition(prevLeftEncoderData, leftEncoderData);
    resultCheck[1] = checkBackwardLimitTransition(prevLeftEncoderData, leftEncoderData);
    resultCheck[2] = checkForwardLimitTransition(prevRightEncoderData, rightEncoderData);
    resultCheck[3] = checkBackwardLimitTransition(prevRightEncoderData, rightEncoderData);


    if(resultCheck[0] == false and resultCheck[1] == false and resultCheck[2] == false and resultCheck[3] == true){
        newPrevRightEncoderData = (double)prevRightEncoderData + (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == false and resultCheck[1] == false and resultCheck[2] == true and resultCheck[3] == false){
        newPrevRightEncoderData = (double)prevRightEncoderData - (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == false and resultCheck[1] == true and resultCheck[2] == false and resultCheck[3] == false){
        newPrevLeftEncoderData = (double)prevLeftEncoderData + (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == false and resultCheck[1] == true and resultCheck[2] == false and resultCheck[3] == true){
        newPrevRightEncoderData = (double)prevRightEncoderData + (2 * FULL_ENCODER_TICKS);
        newPrevLeftEncoderData = (double)prevLeftEncoderData + (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == false and resultCheck[1] == true and resultCheck[2] == true and resultCheck[3] == false){
        newPrevRightEncoderData = (double)prevRightEncoderData - (2 * FULL_ENCODER_TICKS);
        newPrevLeftEncoderData = (double)prevLeftEncoderData + (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == true and resultCheck[1] == false and resultCheck[2] == false and resultCheck[3] == false){
        newPrevLeftEncoderData = (double)prevLeftEncoderData - (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == true and resultCheck[1] == false and resultCheck[2] == false and resultCheck[3] == true){
        newPrevRightEncoderData = (double)prevRightEncoderData - (2 * FULL_ENCODER_TICKS);
        newPrevLeftEncoderData = (double)prevLeftEncoderData + (2 * FULL_ENCODER_TICKS);

    } 
    if(resultCheck[0] == true and resultCheck[1] == false and resultCheck[2] == true and resultCheck[3] == false){
        newPrevRightEncoderData = (double)prevRightEncoderData - (2 * FULL_ENCODER_TICKS);
        newPrevLeftEncoderData = (double)prevLeftEncoderData - (2 * FULL_ENCODER_TICKS);

    }

    double deltaLeft = (newLeftEncoderData - newPrevLeftEncoderData)/((double)getTicksMM());
    double deltaRight = (newRightEncoderData - newPrevRightEncoderData)/((double)getTicksMM());
    

    deltaDistance = (deltaLeft + deltaRight) / 2.0;
    deltaDegrees = ((robot->getEncoderTh() * M_PI / 180) - myRawPose->getThRad());
    //deltaDegrees = (deltaRight - deltaLeft) / (2 * robot->getRobotRadius());

    getRawPoseFromOdometry();

    prevRightEncoderData = rightEncoderData;
    prevLeftEncoderData = leftEncoderData;
}


bool RobotNode::checkForwardLimitTransition(double enc_k, double enc_k_1){
    bool result = false;
    if(enc_k > enc_k_1){
        if((enc_k > (FULL_ENCODER_TICKS/2) and enc_k <= (FULL_ENCODER_TICKS - 1)) and (enc_k_1 >= -FULL_ENCODER_TICKS and enc_k_1 < -(FULL_ENCODER_TICKS/2))){
            result = true;
        }
    }
    return result;
}

bool RobotNode::checkBackwardLimitTransition(double enc_k, double enc_k_1){
    bool result = false;
    if(enc_k_1 > enc_k){
        if((enc_k >= -FULL_ENCODER_TICKS and enc_k < -(FULL_ENCODER_TICKS/2)) and (enc_k_1 > (FULL_ENCODER_TICKS/2) and enc_k_1 <= (FULL_ENCODER_TICKS-1))){
            result = true;
        }
    }
    return result;
}

void RobotNode::getRawPoseFromOdometry(){
    double x = 0, y = 0, th = 0;

    x = myRawPose->getX() + (deltaDistance * cos(myRawPose->getThRad() + deltaDegrees/2.0));
    y = myRawPose->getY() + (deltaDistance * sin(myRawPose->getThRad() + deltaDegrees/2.0));
    th = myRawPose->getThRad() + deltaDegrees;

    myRawPose->setPose(x, y);
    myRawPose->setThRad(th);

}

int RobotNode::getDriftFactor(){
    return robot->getOrigRobotConfig()->getDriftFactor();
}

int RobotNode::getRevCount(){
    return robot->getOrigRobotConfig()->getRevCount();
}

int RobotNode::getTicksMM(){
    return robot->getOrigRobotConfig()->getTicksMM();
}

double RobotNode::getDiffConvFactor(){
    return robot->getRobotParams()->getDiffConvFactor();
}

double RobotNode::getDistConvFactor(){
    return robot->getRobotParams()->getDistConvFactor();
}

double RobotNode::getAngleConvFactor(){
    return robot->getRobotParams()->getAngleConvFactor();
}

long int RobotNode::getRightEncoder(){
    return robot->getRightEncoder();
}

long int RobotNode::getLeftEncoder(){
    return robot->getLeftEncoder();
}

double RobotNode::getDeltaDegrees(){
    return deltaDegrees;
}

double RobotNode::getDeltaDistance(){
    return deltaDistance;
}

void RobotNode::getBatterChargeStatus(void){
    char s = robot->getChargeState();
    if(s != prevBatteryChargeState){
        prevBatteryChargeState = s;
        onBatteryChargeStateChanged(s);
    }
}

void* RobotNode::dataPublishingThread(void* object){
    RobotNode* self = (RobotNode*)object;
    ArUtil::sleep(2000);
    while(true){
        self->computePositionFromEncoders();
        self->getBatterChargeStatus();
        self->getLaserScan();
        self->getBumpersStatus();
        self->getRobotPosition();
        self->getSonarsScan();
        ArUtil::sleep(30);
    }
    return NULL;
}

void* RobotNode::securityDistanceThread(void* object){
    RobotNode* self = (RobotNode*)object;
    while(true){
        ArSick* sickLaser = (ArSick*)self->laser;
        if(sickLaser != NULL){
            sickLaser->lockDevice();
            
            std::vector<ArSensorReading> *currentReadings = sickLaser->getRawReadingsAsVector();
            for(size_t it = 0; it < currentReadings->size(); it++){
                if(sickLaser->getDegrees() == ArSick::DEGREES180){
                    if(sickLaser->getIncrement() == ArSick::INCREMENT_HALF){
                        if(it > 80 && it < 280){
                            if((currentReadings->at(it).getRange() / 1000.0) < 0.5){
                                if(self->isDirectMotion && self->isGoingForward && !self->doNotMove){
                                    self->doNotMove = true;
                                    self->robot->stop();
                                } else if(self->gotoPoseAction->isActive()){
                                    self->gotoPoseAction->deactivate();
                                    self->wasDeactivated = true;
                                }
                            } else { 
                                self->doNotMove = false;
                                if(self->wasDeactivated){
                                    self->wasDeactivated = false;
                                    self->gotoPoseAction->activate();
                                }
                            }
                        }
                    } else{
                        if(it > 40 && it < 140){
                            if((currentReadings->at(it).getRange() / 1000.0) < 0.5){
                                if(self->isDirectMotion && self->isGoingForward && !self->doNotMove){
                                    self->doNotMove = true;
                                    self->robot->stop();
                                } else if(self->gotoPoseAction->isActive()){
                                    self->gotoPoseAction->deactivate();
                                    self->wasDeactivated = true;
                                }
                            } else { 
                                self->doNotMove = false;
                                if(self->wasDeactivated){
                                    self->wasDeactivated = false;
                                    self->gotoPoseAction->activate();
                                }
                            }
                        }
                    }
                }
            }
            sickLaser->unlockDevice();
        }
    }
    return NULL;
}