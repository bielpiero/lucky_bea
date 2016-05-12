#ifndef RN_ACTION_GOTO_H
#define RN_ACTION_GOTO_H

#include "RNUtils.h"
#include "RNPIDController.h"

#define SATURATION_DISTANCE_MM 1000
#define SATURATION_ANGLE_DEG 10

class RNActionGoto : public ArAction{
public:
	RNActionGoto(const char* name = "goto", ArPose goal = ArPose(0.0, 0.0, 0.0), double linearSpeed = 200, double angularSpeed = 20, double minimumDistance = 50, double minimumAngle = 0.3);
	virtual ~RNActionGoto();

	virtual ArActionDesired* fire(ArActionDesired current);

	bool haveAchievedGoal(void);
	bool haveCanceledGoal(void);

	void cancelGoal(void);
	void setGoal(ArPose goal);
	ArPose getGoal(void);

	double getLinearSpeed(void);
	void setLinearSpeed(double speed);

	double getAngularSpeed(void);
	void setAngularSpeed(double speed);

	double getMinimumDistance(void);
	void setMinimumDistance(double distance);
	
	double getMinimumAngle(void);
	void setMinimumAngle(double angle);

	double getActionDistance(void);
	void setActionDistance(double distance);

	double getActionDegrees(void);
	void setActionDegrees(double degrees);

private:
	ArPose goal;
	ArPose previousGoal;

	RNPIDController* linearController;
	RNPIDController* angularController;

	double linearSpeed;
	double angularSpeed;
	double minimumDistance;
	double minimumAngle;
	double actionDistance;
	double actionDegrees;
	double directionToTurn;
	ArActionDesired* myDesired;
	enum RNState{
		STATE_NO_GOAL,
		STATE_ACHIEVED_GOAL,
		STATE_GOING_TO_GOAL
	};
	RNState currentState;
};

#endif