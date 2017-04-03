#include "RobotNode.h"

const float RobotNode::SECURITY_DISTANCE = 0.5;

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
    robot->setNoTimeWarningThisCycle(true);

    this->prevBatteryChargeState = -5;

	this->maxTransVel = robot->getTransVelMax();
	this->maxAbsoluteTransVel = robot->getAbsoluteMaxTransVel();
	this->maxRotVel = robot->getRotVelMax();
	this->maxAbsoluteRotVel = robot->getAbsoluteMaxRotVel();

    laserDataScan = NULL;
    
    isDirectMotion = false;
    isGoingForward = false;
    wasDeactivated = false;
    doNotMove = false;

    this->prevLeftEncoderData = 0;
    this->prevRightEncoderData = 0;
    this->isFirstFakeEstimation = true;

    timerSecs = 0;
    this->securityDistanceWarningTime =  DEFAULT_SECURITY_DISTANCE_WARNING_TIME;
    this->securityDistanceStopTime = DEFAULT_SECURITY_DISTANCE_STOP_TIME;

    robot->requestEncoderPackets();
    myRawPose = new ArPose(0.0, 0.0, 0.0);
    
    gotoPoseAction = new RNActionGoto;
 	robot->addAction(gotoPoseAction, 89);

 	/*laserConnector = new ArLaserConnector(argparser, robot, connector);
	if(!laserConnector->connectLasers(false, false, true)){
		printf("Could not connect to configured lasers.\n");
	}

	laser = robot->findLaser(1);
	if(!laser){
		printf("Error. Not Connected to any laser.\n");
	} else {
		printf("Connected to SICK LMS200 laser.\n");
	}*/
    this->keepActiveSecurityDistanceTimerThread = NO;
    this->keepActiveSensorDataThread = NO;

    pthread_mutex_init(&mutexRawPositionLocker, NULL);
    pthread_mutex_init(&mutexLaserReadingsLocker, NULL);

    pthread_create(&sensorDataThread, NULL, dataPublishingThread, (void *)(this)  );
    printf("Connection Timeout: %d\n", robot->getConnectionTimeoutTime());
    printf("TicksMM: %d, DriftFactor: %d, RevCount: %d\n", getTicksMM(), getDriftFactor(), getRevCount());
    printf("DiffConvFactor: %f, DistConvFactor: %f, AngleConvFactor: %f\n", getDiffConvFactor(), getDistConvFactor(), getAngleConvFactor());
    
    pthread_create(&distanceTimerThread, NULL, securityDistanceTimerThread, (void *)(this)  );
}

RobotNode::~RobotNode(){
    robot->disableSonar();
    robot->disableMotors();
    robot->disconnect();
    //robot->stopRunning();
    delete sonar;
    RNUtils::printLn("Deleted sonar...");
    //delete robot;
    delete connector;
    RNUtils::printLn("Deleted connector...");
    delete myRawPose;
     RNUtils::printLn("Deleted myRawPose...");
    delete gotoPoseAction;
     RNUtils::printLn("Deleted gotoPoseAction...");
    //delete laser;
    //RNUtils::printLn("Disconnected from laser...");
    //delete laserConnector;
    //RNUtils::printLn("Deleted objects...");

    //pthread_mutex_destroy(&mutexRawPositionLocker);
    //robot->waitForRunExit();
    Aria::exit(0);
    RNUtils::printLn("Destroyed RobotNode...");
}

void RobotNode::disconnect(){
    finishThreads();
    RNUtils::printLn("Thread objects finished...");
    //laser->disconnect();
    //RNUtils::printLn("Disconnected from laser...");
    
}

void RobotNode::finishThreads(){
    
    if (keepActiveSecurityDistanceTimerThread == YES) {
        keepActiveSecurityDistanceTimerThread = MAYBE;
        while (keepActiveSecurityDistanceTimerThread != NO) {
            ArUtil::sleep(10);
        }
        pthread_cancel(distanceTimerThread);
    }
    
    RNUtils::printLn("Stopped Security Distance Timer Thread");
    if (keepActiveSensorDataThread == YES) {
        keepActiveSensorDataThread = MAYBE;
        while (keepActiveSensorDataThread != NO) {
            ArUtil::sleep(10);
        }
        pthread_cancel(sensorDataThread);
    }
    RNUtils::printLn("Stopped Robot Data Aquisition Thread");
}

void RobotNode::getLaserScan(void){
	if(laser != NULL){
		laser->lockDevice();
        
		const std::list<ArSensorReading*> *currentReadings = new std::list<ArSensorReading*>(*laser->getRawReadings());
        lockLaserReadings();
        if(laserDataScan == NULL){
            laserDataScan = new LaserScan();
        }
		laserDataScan->clear();
		for(std::list<ArSensorReading*>::const_iterator it = currentReadings->begin(); it != currentReadings->end(); ++it){
			laserDataScan->addLaserScanData((float)(*it)->getRange() / 1000, (float)(*it)->getExtraInt());
		}
        unlockLaserReadings();
        laser->unlockDevice();

        securityDistanceChecker();
        onLaserScanCompleted(laserDataScan);

        delete currentReadings;
	}
}

int RobotNode::lockLaserReadings(){
    return pthread_mutex_lock(&mutexLaserReadingsLocker);
}

int RobotNode::unlockLaserReadings(){
    return pthread_mutex_unlock(&mutexLaserReadingsLocker);
}

LaserScan* RobotNode::getRawLaserReadings(void){
    return laserDataScan;
}

void RobotNode::getRobotPosition(){
	myPose = new ArPose(robot->getPose());
    onPositionUpdate(myPose->getX()/1e3, myPose->getY()/1e3, myPose->getThRad(), robot->getVel()/1e3, robot->getRotVel()*M_PI/180);
    delete myPose;
    myPose = NULL;
}

void RobotNode::getRawRobotPosition(){
    computePositionFromEncoders();
    onRawPositionUpdate(myRawPose->getX()/1e3, myRawPose->getY()/1e3, myRawPose->getThRad(), this->deltaDistance, this->deltaDegrees);
}

void RobotNode::stopRobot(){
    this->lockRobot();
    gotoPoseAction->cancelGoal();
    robot->stop();
    this->unlockRobot();
}

void RobotNode::move(double distance, double speed){
	this->lockRobot();
	robot->setAbsoluteMaxTransVel(speed);

	robot->move(distance);

	robot->setAbsoluteMaxTransVel(this->maxAbsoluteTransVel);
	this->unlockRobot();

}

void RobotNode::moveAtSpeed(double linearVelocity, double angularVelocity){
    isDirectMotion = true;
    isGoingForward = false;
    this->lockRobot();
    gotoPoseAction->cancelGoal();
    if(gotoPoseAction->isActive()){
        gotoPoseAction->deactivate();
    }
    
    if(linearVelocity == 0.0 && angularVelocity == 0.0){
        robot->stop();
        isDirectMotion = false;
    } else {
        robot->clearDirectMotion();
        if(linearVelocity > 0.0){
            isGoingForward = true;
        }
        RNUtils::printLn("{LinVel: %f, AngVel: %f}\n", linearVelocity, angularVelocity);
        robot->setVel(linearVelocity*1e3);

        robot->setRotVel(angularVelocity*180/M_PI);
    }
    
    this->unlockRobot();
}

void RobotNode::gotoPosition(double x, double y, double theta, double transSpeed, double rotSpeed){
    ArPose newPose(x * 1000, y * 1000);
    newPose.setThRad(theta);

    isDirectMotion = false;
    this->lockRobot();

    doNotMove = false;
    wasDeactivated = false;
    robot->clearDirectMotion();
    if(not gotoPoseAction->isActive()){
        gotoPoseAction->activate();
    }
    RNUtils::printLn("Going to: {x: %f, y: %f, \u03d1: %f}", x, y, theta);
    gotoPoseAction->setGoal(newPose);
        
    this->unlockRobot();
    ArUtil::sleep(100);
}

void RobotNode::setPosition(double x, double y, double theta){
    
    this->lockRobot();
    pthread_mutex_lock(&mutexRawPositionLocker);
    myRawPose->setPose(x * 1000, y * 1000, theta * 180 / M_PI);
    pthread_mutex_unlock(&mutexRawPositionLocker);
    robot->moveTo(ArPose(x * 1000, y * 1000, theta * 180 / M_PI));
    this->unlockRobot();
}

void RobotNode::setMotorsStatus(bool enabled){
    this->lockRobot();
    if (enabled) {
        if(!robot->isEStopPressed()){
            robot->enableMotors();
        }
        
    } else {
        robot->disableMotors();
    }
    this->unlockRobot();
}

bool RobotNode::getMotorsStatus(){
    return robot->areMotorsEnabled();
}

bool RobotNode::isGoalAchieved(){
    return gotoPoseAction->haveAchievedGoal();
}

bool RobotNode::isGoalCanceled(){
    return gotoPoseAction->haveCanceledGoal();
}

void RobotNode::setSonarStatus(bool enabled){
    this->lockRobot();
    if (enabled) {
        robot->enableSonar();        
    } else {
        robot->disableSonar();
    }
    this->unlockRobot();
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
            delete reading;
        }
        onSonarsDataUpdate(data);
        for (int i = 0; i < data->size(); i++) {
            delete data->at(i);
        }
        data->clear();
        delete [] data;
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

    double deltaLeftMM = (newLeftEncoderData - newPrevLeftEncoderData)/((double)getTicksMM());
    double deltaRightMM = (newRightEncoderData - newPrevRightEncoderData)/((double)getTicksMM());

    double deltaLeftRad = ((newLeftEncoderData - newPrevLeftEncoderData) * getAngleConvFactor());
    double deltaRightRad = ((newRightEncoderData - newPrevRightEncoderData) * getAngleConvFactor());
    

    deltaDistance = (deltaLeftMM + deltaRightMM) / 2.0;
    deltaDegrees = robot->getPose().getThRad() - myRawPose->getThRad();
    //deltaDegrees = (deltaRightRad - deltaLeftRad) * robotLength;

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
    pthread_mutex_lock(&mutexRawPositionLocker);
    x = myRawPose->getX() + (deltaDistance * cos(myRawPose->getThRad() + deltaDegrees/2.0));
    y = myRawPose->getY() + (deltaDistance * sin(myRawPose->getThRad() + deltaDegrees/2.0));
    th = myRawPose->getThRad() + deltaDegrees;
    pthread_mutex_unlock(&mutexRawPositionLocker);
    this->setPosition(x / 1000.0, y / 1000.0, th);

    //myRawPose->setPose(x, y, th * 180 / M_PI);
    //pthread_mutex_unlock(&mutexRawPositionLocker);

}

void RobotNode::lockRobot(){
    while(this->robot->lock() != 0){
        ArUtil::sleep(5);
        printf("trying to lock...\n");
    }
}

void RobotNode::unlockRobot(){
    while(this->robot->unlock() != 0){
        ArUtil::sleep(5);
        printf("trying to unlock...\n");
    }
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
    self->keepActiveSensorDataThread = YES;
    ArUtil::sleep(2000);
    while(RNUtils::ok() and self->keepActiveSensorDataThread == YES){
        self->getRawRobotPosition();
        //self->getLaserScan();
        self->getBatterChargeStatus();
        self->getBumpersStatus();
        self->getRobotPosition();
        self->getSonarsScan();
        self->onSensorsScanCompleted();
        ArUtil::sleep(10);
    }
    self->keepActiveSensorDataThread = NO;
    return NULL;
}

void RobotNode::securityDistanceChecker(){
    
    if(laserDataScan != NULL){
        
        bool thereIsSomethingInFront = false;
        //pthread_mutex_lock(&mutexLaserDataLocker);
        for(int it = MIN_INDEX_LASER_SECURITY_DISTANCE; (it < (laserDataScan->size() - MAX_INDEX_LASER_SECURITY_DISTANCE)) and not thereIsSomethingInFront; it++){
            if(laserDataScan->getRange(it) < SECURITY_DISTANCE){
                thereIsSomethingInFront = true;                
            }
        }
        //pthread_mutex_unlock(&mutexLaserDataLocker);
        if(thereIsSomethingInFront){
            if(isDirectMotion and isGoingForward and not doNotMove){
                doNotMove = true;
                robot->stop();
            } else if(gotoPoseAction->isActive()){
                gotoPoseAction->deactivate();
                wasDeactivated = true;
                
            }
        } else {
            doNotMove = false;
            if(wasDeactivated){
                wasDeactivated = false;
                gotoPoseAction->activate();
            }
        }
    }
}

void* RobotNode::securityDistanceTimerThread(void* object){
    RobotNode* self = (RobotNode*)object;
    self->keepActiveSecurityDistanceTimerThread = YES;
    while(RNUtils::ok() and self->keepActiveSecurityDistanceTimerThread == YES){
        if(self->wasDeactivated){
            ArUtil::sleep(987);
            self->timerSecs++;
            if(self->timerSecs == self->securityDistanceWarningTime){
                self->onSecurityDistanceWarningSignal();
            } else if(self->timerSecs == self->securityDistanceStopTime){
                self->lockRobot();
                self->gotoPoseAction->cancelGoal();
                self->robot->clearDirectMotion();
                self->unlockRobot();
                self->wasDeactivated = false;
                self->doNotMove = false;
                self->timerSecs = 0;
                self->onSecurityDistanceStopSignal();
            }
        } else{
            self->timerSecs = 0;
        }
        ArUtil::sleep(13);
    }
    self->keepActiveSecurityDistanceTimerThread = NO;
    return NULL;
}