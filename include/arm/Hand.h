#ifndef HAND_H
#define HAND_H

#define PORT_PULGAR 6
#define PORT_INDICE 7
#define PORT_MEDIO  8

#define openThumb 		730
#define closeThumb 		1180
#define openIndex		880
#define closeIndex 		1730
#define openAnular 		1330
#define closeAnular		2140

#include "arm/Finger.h"
#include "SerialPort.h"

class Finger;
//class SerialPort;

class Hand{
private:
	std::vector<Finger*>* fingers;
	SerialPort* controller;
public:

	Hand(SerialPort* controller);

		//funciones
	void add(Finger* finger);
	Finger* at(int index);
	size_t size();

	void setToZero();
	void position();
	void setSpeedFinger(int, int);
	void showAngleLimits();
	void moveHand(int*);
	void moveFinger(int, int);
	void emergencyStop();
};

#endif