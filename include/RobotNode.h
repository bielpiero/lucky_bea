#ifndef ROBOT_NODE_H
#define ROBOT_NODE_H

#include "RNUtils.h"
#include "RNActionGoto.h"

#define FULL_ENCODER_TICKS 32768
#define WHEEL_DIAMETER_MM 194
#define DRIFT_FACTOR_TICKS 8192

#define NO_MOVEMENT 0
#define LOWER_LIMIT_TRANSITION 1
#define UPPER_LIMIT_TRANSITION 2
#define RN_MOVEMENT 3

#define MIN_INDEX_LASER_SECURITY_DISTANCE 90
#define MAX_INDEX_LASER_SECURITY_DISTANCE 120

#define DEFAULT_SECURITY_DISTANCE_WARNING_TIME 30
#define DEFAULT_SECURITY_DISTANCE_STOP_TIME 60

class RobotNode{
private:
	ArRobotConnector *connector;
	ArLaserConnector *laserConnector;
	ArLaser *laser;
    RNActionGoto *gotoPoseAction;
    ArRobot *robot;
    ArPose *myPose;
    ArPose *myRawPose;
    ArSonarDevice *sonar;
    LaserScan *laserDataScan;
    
    double maxTransVel;
    double maxAbsoluteTransVel;
    double maxRotVel;
    double maxAbsoluteRotVel;
    
    bool isDirectMotion;
    bool isGoingForward;
    bool wasDeactivated;
    bool doNotMove;
    char prevBatteryChargeState;

    long int prevLeftEncoderData;
    long int prevRightEncoderData;

    double prevDistance;
    double prevRads;
    double prevVel;
    double prevRotVel;

    int driftFactorIncrement;

    int deltaLeftMM;
    int deltaRightMM;
    
    double deltaDistance;
    double deltaDegrees;

    bool isFirstFakeEstimation;
    
    unsigned int timerSecs;
    unsigned int securityDistanceWarningTime;
    unsigned int securityDistanceStopTime;
    
    char keepActiveSensorDataThread;
    char keepActiveLaserDataThread;
    char keepActiveSecurityDistanceTimerThread;
    
    static const float SECURITY_DISTANCE;

    pthread_mutex_t mutexRawPositionLocker;
    pthread_mutex_t mutexSensorsReadingsLocker;
    pthread_t sensorDataThread;
    pthread_t laserDataThread;
    pthread_t distanceThread;
    pthread_t distanceTimerThread;

public:
	RobotNode(const char* port);
	virtual ~RobotNode();

	void disconnect();
    
    bool isGoalAchieved(void);
    bool isGoalCanceled(void);
    
    void gotoPosition(double x, double y, double theta, double transSpeed = 200, double rotSpeed = 4.75);
	void move(double distance, double speed = 200);
    void moveAtSpeed(double linearVelocity, double angularVelocity);
    void stopRobot(void);

    void getBatterChargeStatus(void);
    void getBumpersStatus(void);
	void getLaserScan(void);
	LaserScan* getRawLaserReadings(void);
    bool getMotorsStatus(void);
    bool getSonarsStatus(void);
    void getSonarsScan(void);
	void getRobotPosition(void);
	void getRawRobotPosition(void);
	    
    void setMotorsStatus(bool enabled);
    void setPosition(double x, double y, double theta);
    void setSonarStatus(bool enabled);

    int getDriftFactor();
	int getRevCount();
	int getTicksMM();
	double getEncoderScaleFactor();
	
	double getDeltaDegrees();
	double getDeltaDistance();

	double getEncoderX();
	double getEncoderY();
	double getEncoderTh();

	long int getLeftEncoder();
	long int getRightEncoder();

	double getDiffConvFactor();
	double getDistConvFactor();
	double getVelConvFactor();
	double getAngleConvFactor();

	int lockSensorsReadings();
	int unlockSensorsReadings();

private:
    static void* securityDistanceTimerThread(void* object);
    static void* laserPublishingThread(void* object);
	static void* dataPublishingThread(void* object);
    
    void finishThreads();
    void lockRobot();
    void unlockRobot();
	void computePositionFromEncoders();
	void getRawPoseFromOdometry();
    void securityDistanceChecker();
    
protected:
	virtual const char* getClassName() const;
	virtual void onLaserScanCompleted(LaserScan* data) = 0;
	virtual void onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear) = 0;
	virtual void onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed) = 0;
	virtual void onRawPositionUpdate(double x, double y, double theta, double deltaDistance, double deltaDegrees) = 0;
	virtual void onSonarsDataUpdate(std::vector<PointXY*>* data) = 0;
	virtual void onBatteryChargeStateChanged(char data) = 0;
    virtual void onSecurityDistanceWarningSignal() = 0;
    virtual void onSecurityDistanceStopSignal() = 0;
    virtual void onSensorsScanCompleted() = 0;
};
#endif