#include "RobotNode.h"
#include "RNActionGoto.h"
#include "RNLaserTask.h"
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
    //dataLaser = NULL;
    laserReady = false;
    robotRawEncoderPosition = Matrix(3, 1);
    pthread_mutex_init(&rawPositionLocker, NULL);
    //robot->setCycleTime(1000);
    robot->setCycleWarningTime(0);
    //robot->setMutexUnlockWarningTime(2000);
    //robot->setNoTimeWarningThisCycle(true);

    connectedCB = ArFunctorC<RobotNode>(this, &RobotNode::connected);
    connFailCB = ArFunctorC<RobotNode>(this, &RobotNode::connectionFailed);
    disconnectedCB = ArFunctorC<RobotNode>(this, &RobotNode::disconnected);
    connLostCB = ArFunctorC<RobotNode>(this, &RobotNode::connectionLost);
    positionUpdateCB = ArFunctorC<RobotNode>(this, &RobotNode::positionUpdate);
    keyHandler = Aria::getKeyHandler();
    if (not keyHandler){
        keyHandler = new ArKeyHandler;
        Aria::setKeyHandler(keyHandler);
        robot->attachKeyHandler(keyHandler, false);
    }
    upCB = ArFunctor2C<RobotNode, double, double>(this, &RobotNode::moveAtSpeed, 0.10, 0.0);
    downCB = ArFunctor2C<RobotNode, double, double>(this, &RobotNode::moveAtSpeed, -0.10, -0.0);
    leftCB = ArFunctor2C<RobotNode, double, double>(this, &RobotNode::moveAtSpeed, 0.0, 0.05);
    rightCB = ArFunctor2C<RobotNode, double, double>(this, &RobotNode::moveAtSpeed, 0.0, -0.05);
    spaceCB = ArFunctor2C<RobotNode, double, double>(this, &RobotNode::moveAtSpeed, 0.0, 0.0);

    keyHandler->addKeyHandler(ArKeyHandler::UP, &upCB);
    keyHandler->addKeyHandler(ArKeyHandler::DOWN, &downCB);
    keyHandler->addKeyHandler(ArKeyHandler::RIGHT, &rightCB);
    keyHandler->addKeyHandler(ArKeyHandler::LEFT, &leftCB);
    keyHandler->addKeyHandler(ArKeyHandler::SPACE, &spaceCB);

    robot->addConnectCB(&connectedCB, ArListPos::FIRST);
    robot->addFailedConnectCB(&connFailCB, ArListPos::FIRST);
    robot->addDisconnectNormallyCB(&disconnectedCB, ArListPos::FIRST);
    robot->addDisconnectOnErrorCB(&connLostCB, ArListPos::FIRST);

    robot->addSensorInterpTask("Position Update Task", 50, &positionUpdateCB);

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

    isFirstFakeEstimation = true;
    altPose = NULL;
    gotoPoseAction = new RNActionGoto(this);
    robot->addAction(gotoPoseAction, 89);
    
    //distanceTimer = new RNDistanceTimerTask(this);
    //distanceTimer->go();

    pthread_mutex_init(&mutexIncrements, NULL);
    pthread_mutex_init(&mutexLaser, NULL);

    printf("Connection Timeout: %d\n", robot->getConnectionTimeoutTime());
    printf("TicksMM: %d, DriftFactor: %d, RevCount: %d\n", getTicksMM(), getDriftFactor(), getRevCount());
    printf("RotKp: %d, RotKi: %d, RotKd: %d, TransKp: %d, TransKi: %d, TransKd: %d\n", robot->getOrigRobotConfig()->getRotKP(), robot->getOrigRobotConfig()->getRotKI(), robot->getOrigRobotConfig()->getRotKV(), robot->getOrigRobotConfig()->getTransKP(), robot->getOrigRobotConfig()->getTransKI(), robot->getOrigRobotConfig()->getTransKV());
    printf("DiffConvFactor: %f, DistConvFactor: %f, VelocityConvFactor: %f, AngleConvFactor: %f\n", getDiffConvFactor(), getDistConvFactor(), getVelConvFactor(), getAngleConvFactor());

}

RobotNode::~RobotNode(){

    /*if(distanceTimer){
        distanceTimer->kill();
        delete distanceTimer;
    }*/

    if(connector){
        delete connector;
        RNUtils::printLn("Deleted connector...");
    }

    if(gotoPoseAction){
        delete gotoPoseAction;
        RNUtils::printLn("Deleted gotoPoseAction...");
    }

    pthread_mutex_destroy(&rawPositionLocker);
    RNUtils::printLn("Deleted rawDeltaEncoderLocker...");
    pthread_mutex_destroy(&mutexIncrements);
    RNUtils::printLn("Deleted mutexIncrements...");
    pthread_mutex_destroy(&mutexLaser);
    RNUtils::printLn("Deleted mutexIncrements...");
    
    RNUtils::printLn("Destroyed RobotNode...");
    //robot->waitForRunExit();
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

void RobotNode::positionUpdate(void){
    lockRawPosition();
    robotRawEncoderPosition(0, 0) = robot->getX()/1e3;
    robotRawEncoderPosition(1, 0) = robot->getY()/1e3;
    robotRawEncoderPosition(2, 0) = robot->getTh() * M_PI / 180;
    unlockRawPosition();
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
    //robot->stopRunning();
    //robot->waitForRunExit();
    //connector->disconnectAll();
    //finishThreads();    
}

void RobotNode::getRobotPosition(){
    this->lockRobot();
    ArPose *myPose = new ArPose(robot->getPose());
    onPositionUpdate(myPose->getX()/1e3, myPose->getY()/1e3, myPose->getThRad(), robot->getVel()/1e3, robot->getRotVel()*M_PI/180);
    delete myPose;
    myPose = NULL;
    this->unlockRobot();
}

bool RobotNode::getSonarsStatus(){
    return robot->areSonarsEnabled();
}

Matrix RobotNode::getRawEncoderPosition(){
    Matrix r;
    lockRawPosition();
    r = Matrix(robotRawEncoderPosition);
    unlockRawPosition();
    return r;
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
        RNUtils::printLn("{LinVel: %f, AngVel: %f}", linearVelocity, angularVelocity);
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
    
    isFirstFakeEstimation = true;
    robot->resetTripOdometer();
    robot->moveTo(ArPose(x * 1000, y * 1000, theta * 180 / M_PI));
    
    robot->unlock();

    lockRawPosition();
    robotRawEncoderPosition(0, 0) = x;
    robotRawEncoderPosition(1, 0) = y;
    robotRawEncoderPosition(2, 0) = theta;
    unlockRawPosition();
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

ArPose* RobotNode::getAltPose(){
    pthread_mutex_lock(&mutexAltPose);
    ArPose* r = NULL;
    if(altPose != NULL){
        r = new ArPose(*altPose);
    } else{ 
        r = new ArPose(0.0, 0.0, 0.0);
    }
    pthread_mutex_unlock(&mutexAltPose);
    return r;
}

void RobotNode::setAltPose(ArPose pose){
    pthread_mutex_lock(&mutexAltPose);
    if(altPose != NULL){
        delete altPose;
        altPose = NULL;
    }
    altPose = new ArPose(pose.getX()*1000, pose.getY()*1000, pose.getTh());
    pthread_mutex_unlock(&mutexAltPose);
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


void RobotNode::setIncrementPosition(double deltaDistance, double deltaDegrees){
    pthread_mutex_lock(&mutexIncrements);
    this->deltaDegrees = deltaDegrees;
    this->deltaDistance = deltaDistance;
    pthread_mutex_unlock(&mutexIncrements);
}

void RobotNode::getIncrementPosition(double* deltaDistance, double* deltaDegrees){
    if(deltaDistance != NULL and deltaDegrees != NULL){
        robot->lock();
        double currentDistance = robot->getOdometerDistance();
        //double currentDistance = RNUtils::distanceTo(robot->getPose().getX(), robot->getPose().getY(), 0, 0);
        double currentRads = RNUtils::deg2Rad(robot->getPose().getTh());
        //double currentRads = RNUtils::deg2Rad(robot->getOdometerDegrees());

        double currentVel = robot->getVel();
        double currentRotVel = robot->getRotVel();
        robot->unlock();
    
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

        /*if(currentRotVel < 0){
            this->deltaDegrees = -currentRads + prevRads;
            //if(this->deltaDegrees > 0){
                //this->deltaDegrees *= -1.0;
            //}
        }*/
        
        prevVel = currentVel;
        prevRotVel = currentRotVel;
        prevDistance = currentDistance;
        prevRads = currentRads;

        pthread_mutex_lock(&mutexIncrements);
        *deltaDegrees = this->deltaDegrees;
        *deltaDistance = this->deltaDistance/1.0e3;
        pthread_mutex_unlock(&mutexIncrements);
        robot->lock();
        robot->resetTripOdometer();
        robot->unlock();
    }
}

int RobotNode::lockRawPosition(){
    return pthread_mutex_lock(&rawPositionLocker);
}

int RobotNode::unlockRawPosition(){
    return pthread_mutex_unlock(&rawPositionLocker);
}

void RobotNode::onLaserScanCompleted(LaserScan* data){
    pthread_mutex_lock(&mutexLaser);
    /*if(dataLaser != NULL){
        delete dataLaser;
        dataLaser = NULL;
    }*/
    if(data != NULL){
        dataLaser = *data;
    }
    pthread_mutex_unlock(&mutexLaser);
}

LaserScan* RobotNode::getLaserScan(){
    LaserScan* result = NULL;
    pthread_mutex_lock(&mutexLaser);
    //if(dataLaser != NULL){
        result = new LaserScan(dataLaser);
    //}
    pthread_mutex_unlock(&mutexLaser);
    return result;
}


