#include "RNDualQuaternion.h"

RNDualQuaternion::RNDualQuaternion(const RNQuaternion q1, const RNQuaternion q2){
	this->q1 = q1;
	this->q2 = q2;
}