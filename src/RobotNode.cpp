#include "RobotNode.h"
#include "RNActionGoto.h"
#include "RNLaserTask.h"
#include "RNFactorySensorsTask.h"
#include "RNDistanceTimerTask.h"

RobotNode::RobotNode(const char* port){
	Aria::init();

	robot = new ArRobot();

	ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
	args->add("-robotPort"); // pass robot's serial port to Aria
    args->add(port);

  	argparser = new ArArgumentParser(args); // Warning never freed
  	argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
  	argparser->addDefaultArgument("-connectLaser -lp /dev/ttyS2 -lrb 1ref -ld 180 -li half -lf true"); //-lp /dev/ttyS2 port to connect

	connector = new ArRobotConnector(argparser, robot);

	connector->connectRobot();

	robot->enableMotors();
    robot->disableSonar();   

    robot->runAsync(true);
    robot->startStabilization();
    dataLaser = NULL;
    //robot->setCycleTime(1000);
    robot->setCycleWarningTime(0);
    //robot->setMutexUnlockWarningTime(2000);
    //robot->setNoTimeWarningThisCycle(true);

    connectedCB = ArFunctorC<RobotNode>(this, &RobotNode::connected);
    connFailCB = ArFunctorC<RobotNode>(this, &RobotNode::connectionFailed);
    disconnectedCB = ArFunctorC<RobotNode>(this, &RobotNode::disconnected);
    connLostCB = ArFunctorC<RobotNode>(this, &RobotNode::connectionLost);

    robot->addConnectCB(&connectedCB, ArListPos::FIRST);
    robot->addFailedConnectCB(&connFailCB, ArListPos::FIRST);
    robot->addDisconnectNormallyCB(&disconnectedCB, ArListPos::FIRST);
    robot->addDisconnectOnErrorCB(&connLostCB, ArListPos::FIRST);

    this->prevBatteryChargeState = -5;

	this->maxTransVel = robot->getTransVelMax();
	this->maxAbsoluteTransVel = robot->getAbsoluteMaxTransVel();
	this->maxRotVel = robot->getRotVelMax();
	this->maxAbsoluteRotVel = robot->getAbsoluteMaxRotVel();
        
    directMotion = false;
    goingForward = false;
    wasDeactivated = false;
    doNotMove = false;

    this->driftFactorIncrement = 0;
    
    
    sensors = new RNFactorySensorsTask(this);
    sensors->go();

    distanceTimer = new RNDistanceTimerTask(this);
    distanceTimer->go();

    //robot->requestEncoderPackets();
    //myRawPose = new ArPose(0.0, 0.0, 0.0);
    isFirstFakeEstimation = true;
    gotoPoseAction = new RNActionGoto(this);
 	robot->addAction(gotoPoseAction, 89);
    
 	//this->keepActiveSecurityDistanceTimerThread = RN_NO;
    //pthread_mutex_init(&mutexRawPositionLocker, NULL);
    pthread_mutex_init(&mutexIncrements, NULL);
    pthread_mutex_init(&mutexLaser, NULL);

    printf("Connection Timeout: %d\n", robot->getConnectionTimeoutTime());
    printf("TicksMM: %d, DriftFactor: %d, RevCount: %d\n", getTicksMM(), getDriftFactor(), getRevCount());
    printf("DiffConvFactor: %f, DistConvFactor: %f, VelocityConvFactor: %f, AngleConvFactor: %f\n", getDiffConvFactor(), getDistConvFactor(), getVelConvFactor(), getAngleConvFactor());

}

RobotNode::~RobotNode(){
    sensors->kill();
    delete sensors;

    distanceTimer->kill();
    delete distanceTimer;

    if(connector){
        delete connector;
        RNUtils::printLn("Deleted connector...");
    }

    if(gotoPoseAction){
        delete gotoPoseAction;
        RNUtils::printLn("Deleted gotoPoseAction...");
    }
    
    pthread_mutex_destroy(&mutexIncrements);
    RNUtils::printLn("Deleted mutexIncrements...");
    pthread_mutex_destroy(&mutexLaser);
    RNUtils::printLn("Deleted mutexIncrements...");
    
    RNUtils::printLn("Destroyed RobotNode...");
}

const char* RobotNode::getClassName() const{
    return "RobotNode";
}

// called if the connection was sucessfully made
void RobotNode::connected(void){
    RNUtils::printLn("Connected to robot!!!!! YAY!!!!");
}
// called if the connection failed. stop the robot processing thread.
void RobotNode::connectionFailed(void){
    RNUtils::printLn("OH NO!!!! Something went wrong!!!");
    Aria::exit(1);
}
// called when the connection is closed
void RobotNode::disconnected(void){
    RNUtils::printLn("Bye!!!! Come back soon....");     
}
// called if the connection is lost due to an error
void RobotNode::connectionLost(void){
    RNUtils::printLn("OH NO!!!! Connection is lost :-(");
    Aria::exit(1);
}

ArRobotConnector* RobotNode::getRobotConnector() const{
    return connector;
}

ArRobot* RobotNode::getRobot(){
    return robot;
}

ArArgumentParser* RobotNode::getArgumentParser() const{
    return argparser;
}

void RobotNode::disconnect(){
    RNUtils::printLn("Starting disconnection process...");
    //robot->unlock();
    robot->disableMotors();
    robot->stopRunning();
    //connector->disconnectAll();
    //finishThreads();    
}

void RobotNode::getRobotPosition(){
    ArPose *myPose = new ArPose(robot->getPose());
    onPositionUpdate(myPose->getX()/1e3, myPose->getY()/1e3, myPose->getThRad(), robot->getVel()/1e3, robot->getRotVel()*M_PI/180);
    delete myPose;
    myPose = NULL;
}

bool RobotNode::getSonarsStatus(){
    return robot->areSonarsEnabled();
}

void RobotNode::getBatterChargeStatus(void){
    char s = robot->getChargeState();
    if(s != prevBatteryChargeState){
        prevBatteryChargeState = s;
        onBatteryChargeStateChanged(s);
    }
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
    directMotion = true;
    goingForward = false;
    this->lockRobot();
    gotoPoseAction->cancelGoal();
    if(gotoPoseAction->isActive()){
        gotoPoseAction->deactivate();
    }
    
    if(linearVelocity == 0.0 && angularVelocity == 0.0){
        robot->stop();
        directMotion = false;
    } else {
        robot->clearDirectMotion();
        if(linearVelocity > 0.0){
            goingForward = true;
        }
        RNUtils::printLn("{LinVel: %f, AngVel: %f}\n", linearVelocity, angularVelocity);
        robot->setVel(linearVelocity*1e3);

        robot->setRotVel(angularVelocity*180/M_PI);
    }
    
    this->unlockRobot();
}

void RobotNode::gotoPosition(double x, double y, double theta, bool isHallway, double transSpeed, double rotSpeed){
    ArPose newPose(x * 1000, y * 1000);
    newPose.setThRad(theta);

    directMotion = false;
    this->lockRobot();

    doNotMove = false;
    wasDeactivated = false;
    robot->clearDirectMotion();
    if(not gotoPoseAction->isActive()){
        gotoPoseAction->activate();
    }
    RNUtils::printLn("Going to: {x: %f, y: %f, \u03d1: %f}", x, y, theta);
    gotoPoseAction->setGoal(newPose, isHallway);
        
    this->unlockRobot();
    ArUtil::sleep(100);
}

void RobotNode::setPosition(double x, double y, double theta){
    
    robot->lock();
    //pthread_mutex_lock(&mutexRawPositionLocker);
    //myRawPose->setPose(x * 1000, y * 1000, theta * 180 / M_PI);
    //pthread_mutex_unlock(&mutexRawPositionLocker);
    sensors->kill();
    isFirstFakeEstimation = true;
    robot->resetTripOdometer();
    robot->moveTo(ArPose(x * 1000, y * 1000, theta * 180 / M_PI));
    sensors->reset();
    robot->unlock();
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

void RobotNode::cancelGoal(){
    this->wasDeactivated = false;
    gotoPoseAction->cancelGoal();
}

bool RobotNode::isGoalActive(void){
    return gotoPoseAction->isActive();
}

void RobotNode::activateGoal(void){
    this->wasDeactivated = false;
    gotoPoseAction->activate();
}

void RobotNode::deactivateGoal(void){
    this->wasDeactivated = true;
    gotoPoseAction->deactivate();
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

bool RobotNode::isDirectMotion(){
    return this->directMotion;
}
bool RobotNode::isGoingForward(){
    return this->goingForward;
}

bool RobotNode::isNotAllowedToMove(){
    return this->doNotMove;
}

void RobotNode::notAllowedToMove(bool notAllowed){
    this->doNotMove = notAllowed;
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
    return (2 * robot->getOrigRobotConfig()->getRevCount());
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

double RobotNode::getVelConvFactor(){
    return robot->getRobotParams()->getVelConvFactor();
}

double RobotNode::getAngleConvFactor(){
    return robot->getRobotParams()->getAngleConvFactor();
}

double RobotNode::getEncoderX(){
    return robot->getEncoderX();
}

double RobotNode::getEncoderY(){
    return robot->getEncoderY();
}

double RobotNode::getEncoderTh(){
    return robot->getEncoderTh();
}

double RobotNode::getEncoderScaleFactor(){
    return (((double)WHEEL_DIAMETER_MM) * M_PI / (double)getRevCount());
}


void RobotNode::setIncrementPosition(double deltaDistance, double deltaDegrees){
    pthread_mutex_lock(&mutexIncrements);
    this->deltaDegrees = deltaDegrees;
    this->deltaDistance = deltaDistance;
    pthread_mutex_unlock(&mutexIncrements);
}

void RobotNode::getIncrementPosition(double* deltaDistance, double* deltaDegrees){
    if(deltaDistance != NULL and deltaDegrees != NULL){
        double currentDistance = robot->getOdometerDistance();
        double currentRads = RNUtils::deg2Rad(robot->getOdometerDegrees());

        double currentVel = robot->getVel();
        double currentRotVel = robot->getRotVel();
    
        //RNUtils::printLn("odometer: {d: %lf, th: %lf}, vel: {lin: %lf, rot: %lf}", currentDistance, currentRads, robot->getVel(), robot->getRotVel());

        if(isFirstFakeEstimation){
            prevDistance = currentDistance;
            prevRads = currentRads;
            prevVel = currentVel;
            prevRotVel = currentRotVel;
            isFirstFakeEstimation = false;
        }

        this->deltaDistance = currentDistance - prevDistance;
        this->deltaDegrees = currentRads - prevRads;

        if(robot->getVel() < 0){
            this->deltaDistance *= -1.0;
        }

        if(currentRotVel < 0){
            if(this->deltaDegrees > 0){
                this->deltaDegrees *= -1.0;    
            }
        }
        prevVel = currentVel;
        prevRotVel = currentRotVel;
        prevDistance = currentDistance;
        prevRads = currentRads;

        pthread_mutex_lock(&mutexIncrements);
        *deltaDegrees = this->deltaDegrees;
        *deltaDistance = this->deltaDistance/1.0e3;
        pthread_mutex_unlock(&mutexIncrements);
    }
}

void RobotNode::onLaserScanCompleted(LaserScan* data){
    pthread_mutex_lock(&mutexLaser);
    if(dataLaser != NULL){
        delete dataLaser;
        dataLaser = NULL;
    }
    dataLaser = new LaserScan(*data);
    pthread_mutex_unlock(&mutexLaser);
}

LaserScan* RobotNode::getLaserScan(){
    LaserScan* result;
    pthread_mutex_lock(&mutexLaser);
    result = new LaserScan(*dataLaser);
    pthread_mutex_unlock(&mutexLaser);
    return result;
}


