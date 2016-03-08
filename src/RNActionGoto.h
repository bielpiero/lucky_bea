#ifndef RN_ACTION_GOTO_H
#define RN_ACTION_GOTO_H

#include "Aria.h"

class RNActionGoto : public ArAction{
public:
	RNActionGoto(const char* name = "goto", ArPose goal = ArPose(0.0, 0.0, 0.0), double linearSpeed = 100, double angularSpeed = 50, double minimumDistance = 100, double minimumAngle = 0.1);
	virtual ~RNActionGoto();

	virtual ArActionDesired* fire(ArActionDesired current);

	bool haveAchievedGoal(void);
	bool haveCanceledGoal(void);

	void cancelGoal(void);
	void setGoal(ArPose goal);

	double getLinearSpeed(void);
	void setLinearSpeed(double speed);

	double getAngularSpeed(void);
	void setAngularSpeed(double speed);

	double getMinimumDistance(void);
	void setMinimumDistance(double distance);
	
	double getMinimumAngle(void);
	void setMinimumAngle(double angle);

private:
	ArPose goal;
	ArPose previousGoal;
	double linearSpeed;
	double angularSpeed;
	double minimumDistance;
	double minimumAngle;
	double directionToTurn;
	ArActionDesired myDesired;
	enum RNState{
		STATE_NO_GOAL,
		STATE_ACHIEVED_GOAL,
		STATE_GOING_TO_GOAL
	};
	RNState currentState;
};

#endif