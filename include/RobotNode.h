#include "RNUtils.h"
#include "RNActionGoto.h"

#define FULL_ENCODER_TICKS 32768
#define MIN_INDEX_LASER_SECURITY_DISTANCE 90
#define MAX_INDEX_LASER_SECURITY_DISTANCE 120

#define DEFAULT_SECURITY_DISTANCE_WARNING_TIME 30
#define DEFAULT_SECURITY_DISTANCE_STOP_TIME 60

class LsColor{
private:
	unsigned char red;
	unsigned char green;
	unsigned char blue;
public:
	LsColor(unsigned char red, unsigned char green, unsigned char blue){
		this->red = red;
		this->green = green;
		this->blue = blue;
	}
	~LsColor(){}
	unsigned char getRed(){ return red; }
	unsigned char getGreen(){ return green; }
	unsigned char getBlue(){ return blue; }
	std::string getHexadecimalColorString(){
		char buf[6]={};
		sprintf(buf, "%02x%02x%02x", red & 0xFF, green & 0xFF, blue & 0xFF);
		return buf;

	}
};

class LaserScan{
private:
	std::vector<float>* ranges;
	std::vector<float>* intensities;
	std::vector<LsColor*>* color;

public:
	LaserScan(){
		ranges = new std::vector<float>();
		intensities = new std::vector<float>();
		color = new std::vector<LsColor*>();
	}
    ~LaserScan(){
        ranges->clear();
        intensities->clear();
        delete ranges;
        delete intensities;
        for (int i = 0; i < color->size(); i++) {
            delete color->at(i);
        }
        color->clear();
        delete color;
    }
	std::vector<float>* getRanges() { return ranges; }
	std::vector<float>* getIntensities() { return intensities; }
	std::vector<LsColor*>* getColors() { return color; }

	void addLaserScanData(float range, float intensity = 0) { 
		ranges->push_back(range); 
		intensities->push_back(intensity);
	}

	void setScanPrimaryColor(unsigned char red, unsigned char green, unsigned char blue){
		color->push_back(new LsColor(red, green, blue));
	}

	float getRange(int index) { return ranges->at(index); }
	float getIntensity(int index) { return intensities->at(index); }
	LsColor* getColor(int index) { return color->at(index); }

	int size() { return ranges->size(); }
	float getAngleMin() { return (-M_PI / 2.0); }
	float getAngleMax() { return (M_PI / 2.0); }
	float getIncrement() { return (0.5 * M_PI / 180); }
};

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

    double deltaDistance;
    double deltaDegrees;

    bool isFirstFakeEstimation;
    
    unsigned int timerSecs;
    unsigned int securityDistanceWarningTime;
    unsigned int securityDistanceStopTime;
    
    char keepActiveSensorDataThread;
    char keepActiveSecurityDistanceTimerThread;
    
    static const float SECURITY_DISTANCE;

    pthread_mutex_t mutexRawPositionLocker;
    pthread_t sensorDataThread;
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

	double getDeltaDegrees();
	double getDeltaDistance();

	long int getLeftEncoder();
	long int getRightEncoder();

	double getDiffConvFactor();
	double getDistConvFactor();
	double getAngleConvFactor();

private:
    static void* securityDistanceTimerThread(void* object);
	static void* dataPublishingThread(void* object);
    
    void finishThreads();
    void lockRobot();
    void unlockRobot();
	void computePositionFromEncoders();
	void getRawPoseFromOdometry();
    bool checkForwardLimitTransition(double enc_k, double enc_k_1);
    bool checkBackwardLimitTransition(double enc_k, double enc_k_1);

    void securityDistanceChecker();
    
protected:
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