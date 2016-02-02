#ifndef ROBOT_DATA_STREAMER_H
#define ROBOT_DATA_STREAMER_H

#include "SocketNode2.h"

class RobotDataStreamer : public CSocketNode{
public:
	RobotDataStreamer(){}
	~RobotDataStreamer(void){}

	virtual void onConnection(int socketIndex){ }
	virtual void onMsg(int socketIndex, char* cad, unsigned long long int length){}
};

#endif