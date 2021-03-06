#ifndef ROBOT_NODE_H
#define ROBOT_NODE_H

#include "RNUtils.h"

#define FULL_ENCODER_TICKS 32768
#define WHEEL_DIAMETER_MM 194
#define DRIFT_FACTOR_TICKS 8192

#define NO_MOVEMENT 0
#define LOWER_LIMIT_TRANSITION 1
#define UPPER_LIMIT_TRANSITION 2
#define RN_MOVEMENT 3

class RNDistanceTimerTask;

class RNActionGoto;

class RobotNode{
private:
	ArRobotConnector *connector;
    ArArgumentParser *argparser;
    RNActionGoto *gotoPoseAction;
    ArKeyHandler *keyHandler;
    ArRobot *robot;
    ArPose *myPose;
    ArPose *altPose;
    LaserScan dataLaser;

    ArFunctorC<RobotNode> connectedCB;
    ArFunctorC<RobotNode> connFailCB;
    ArFunctorC<RobotNode> disconnectedCB;
    ArFunctorC<RobotNode> connLostCB;
    ArFunctorC<RobotNode> positionUpdateCB;
    ArFunctor2C<RobotNode, double, double> upCB;
    ArFunctor2C<RobotNode, double, double> downCB;
    ArFunctor2C<RobotNode, double, double> rightCB;
    ArFunctor2C<RobotNode, double, double> leftCB;
    ArFunctor2C<RobotNode, double, double> spaceCB;

    double maxTransVel;
    double maxAbsoluteTransVel;
    double maxRotVel;
    double maxAbsoluteRotVel;
    
    bool directMotion;
    bool goingForward;
    bool wasDeactivated;
    bool doNotMove;
    char prevBatteryChargeState;

    int driftFactorIncrement;

    int deltaLeftMM;
    int deltaRightMM;
    
    double deltaDistance;
    double deltaDegrees;

    double prevDistance;
    double prevRads;
    double prevVel;
    double prevRotVel;

    bool isFirstFakeEstimation;
    bool laserReady;

    RNDistanceTimerTask* distanceTimer;

    pthread_mutex_t mutexIncrements;
    pthread_mutex_t mutexAltPose;
    pthread_mutex_t mutexLaser;
    pthread_mutex_t rawPositionLocker;

    Matrix robotRawEncoderPosition;

public:
	RobotNode(const char* port);
	virtual ~RobotNode();
    virtual const char* getClassName() const;
    
	void disconnect();

    ArRobotConnector* getRobotConnector() const;
    ArRobot* getRobot();
    ArArgumentParser* getArgumentParser() const;

    bool isGoalAchieved(void);
    bool isGoalCanceled(void);

    bool isDirectMotion();
    bool isGoingForward();
    bool isNotAllowedToMove();
    void notAllowedToMove(bool allowed);

    bool isGoalActive(void);
    void activateGoal(void);
    void deactivateGoal(void);
    void cancelGoal();
    
    void gotoPosition(double x, double y, double theta, bool isHallway = false, double transSpeed = 200, double rotSpeed = 4.75);
	void move(double distance, double speed = 200);
    void moveAtSpeed(double linearVelocity, double angularVelocity);
    void stopRobot(void);

    void getBatterChargeStatus(void);
    LaserScan* getLaserScan();
		
    bool getMotorsStatus(void);
    bool getSonarsStatus(void);
	void getRobotPosition(void);
	    
    void setMotorsStatus(bool enabled);
    void setPosition(double x, double y, double theta);
    void setSonarStatus(bool enabled);

    void setIncrementPosition(double deltaDistance, double deltaDegrees);
    void getIncrementPosition(double* deltaDistance, double* deltaDegrees);

    int getDriftFactor();
	int getRevCount();
	int getTicksMM();
	double getEncoderScaleFactor();

    ArPose* getAltPose();
    void setAltPose(ArPose pose);
    
	double getDiffConvFactor();
	double getDistConvFactor();
	double getVelConvFactor();
	double getAngleConvFactor();

    int lockRawPosition();
    int unlockRawPosition();

    Matrix getRawEncoderPosition();

private:
    
    void lockRobot();
    void unlockRobot();
    void securityDistanceChecker();

    // called if the connection was sucessfully made
    void connected(void);
    // called if the connection failed. stop the robot processing thread.
    void connectionFailed(void);
    // called when the connection is closed
    void disconnected(void);
    // called if the connection is lost due to an error
    void connectionLost(void);
    // called to update position
    void positionUpdate(void);

    
public:
	virtual void onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear) = 0;
	virtual void onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed) = 0;
	virtual void onSonarsDataUpdate(std::vector<PointXY*>* data) = 0;
	virtual void onBatteryChargeStateChanged(char data) = 0;
    virtual void onSecurityDistanceWarningSignal() = 0;
    virtual void onSecurityDistanceStopSignal() = 0;
    virtual void onSensorsScanCompleted() = 0;
    virtual void onLaserScanCompleted(LaserScan* data);
};
#endif