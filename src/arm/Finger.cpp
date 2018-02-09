#include "Finger.h"

Finger::Finger(int p, const char* fingerName, int open, int close){
	dedoPort=p;
	this->fingerName = std::string(fingerName);
	openDedo=open;
	closeDedo=close;
}

//DEVUELVE EL PUERTO DEL DEDO
int Finger::getPort(){
	return dedoPort;
}

//DEVUELVE EL NOMBRE DEL DEDO
const char* Finger::getFingerName() const{
	return fingerName.c_str();
}

//ESCRIBE LA POSICION DEL DEDO EN ANGULO
void Finger::setAnglePosition(uint16_t currentPosition){
	this->currentPositionAngle = currentPosition;
}
//DEVUELVE LA POSICION DEL DEDO EN ANGULO
uint16_t Finger::getAnglePosition(){
	//transformar del valor que tenga a angulo entre 90 y 180
	std::cout<<"dedo "<<getFingerName()<<" posicion: "<<currentPositionAngle<<std::endl;
	return this->currentPositionAngle;
}
//ESCRIBE LA POSICION DEL DEDO EN STEPS
void Finger::setStepsPosition(uint16_t currentPosition){
	this->currentPositionSteps = currentPosition;
}
//DEVUELVE LA POSICION DEL DEDO EN STEPS
uint16_t Finger::getStepsPosition(){
	//transformar del valor que tenga a angulo entre 90 y 180
	std::cout<<"dedo "<<getFingerName()<<" posicion: "<<currentPositionSteps<<std::endl;
	return this->currentPositionSteps;
}
//FIJA LA VELOCIDAD DE UN DEDO
void Finger::setSpeed(uint16_t speed){
	this->speed = speed;
}

//DEVUELVE LA VELOCIDAD DEL DEDO
uint16_t Finger::getSpeed(){
	return this->speed;
}

//CONVIERTE ANGULO EN PASOS DEL MOTOR DEL DEDO
int Finger::angleToSteps(int a){
	int step=closeDedo - (closeDedo-openDedo)*a/90;
	std::cout<<"steps AngleToSteps: "<<step<<std::endl;
	return step;
}

//CONVIERTE PASOS DEL SERVO EN ANGULO
int Finger::stepsToAngle(int s){
	int angle= (s-closeDedo)*80/(openDedo-closeDedo);
	return angle;
}

//DEVUELVE LA POSICION ZERO DEL DEDO
int Finger::getZero(){
	return closeDedo;
}

//COMPRUEBA QUE ES UN ANGULO AL QUE PUEDE IR EL DEDO
bool Finger::checkAngle(int ang){
	bool valid=false;
	//ojo que maxDedo<minDedo
	if((ang<=90)&&(ang>=0)){
		valid=true;
	}
	return valid;
}

//COMPRUEBA QUE LLEGA A ESOS PASOS
bool Finger::checkSteps(int s){
	bool valid=false;
	//ojo que maxDedo<minDedo
	if((s<=closeDedo)&&(s>=openDedo)){
		valid=true;
	}
	return valid;
}
