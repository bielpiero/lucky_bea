#ifndef RN_DUAL_QUATERNION_H
#define RN_DUAL_QUATERNION_H


#include "RNQuaternion.h"

class RNDualQuaternion{
private:
	RNQuaternion q1;
	RNQuaternion q2;
public:
	RNDualQuaternion(const RNQuaternion q1 = RNQuaternion(), const RNQuaternion q2 = RNQuaternion());
	const char* toString() const;
};

#endif