#ifndef ROBOT_VIRTUAL_FACE_H
#define ROBOT_VIRTUAL_FACE_H

#include "RNSocketClient.h"

class RNVirtualFace : public RNSocketClient{
public:
	RNVirtualFace(){}
	~RNVirtualFace(void){}

	virtual void onConnection(){ }
	virtual void onMsg(char* cad, unsigned long long int length){}
};
#endif