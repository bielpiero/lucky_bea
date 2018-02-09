#include "arm/Hand.h"



//CONSTRUCTOR
Hand::Hand(SerialPort* controller){
	fingers = new std::vector<Finger*>();
	fingers->push_back(new Finger(PORT_PULGAR, "Thumb",openThumb,closeThumb));
	fingers->push_back(new Finger(PORT_INDICE, "Index",openIndex,closeIndex));
	fingers->push_back(new Finger(PORT_MEDIO, "Anular",openAnular,closeAnular));
	
	std::cout<<" mano de tamaño "<< fingers->size() <<std::endl;
	this->controller = controller;
}

//PONER LA MANO EN POSICION INICIAL
void Hand::setToZero(){
	for (unsigned int i=0;i<fingers->size();i++){
		controller->setTarget(0,fingers->at(i)->getPort(), fingers->at(i)->getZero()*4);
	}
}

//MUESTRA LA POSICIÓN DE LA MANO
void Hand::position(){
	for (unsigned int i =0;i<fingers->size();i++){
		fingers->at(i)->getAnglePosition();
	}
}

//MUESTRA ANGULOS LIMITES DE LOS DEDOS
void Hand::showAngleLimits(){
	std::cout<<"0 dedo cerrado, 90 dedo estirado"<<std::endl;
}
//VELOCIDAD DE UN DEDO
void Hand::setSpeedFinger(int f, int s){
	controller ->setSpeed(0, fingers->at(f)->getPort(),s);
}
//MUEVE SOLO UN DEDO
void Hand::moveFinger(int finger, int steps){
	std::cout<<"move finger"<< steps<< "pasos"<<std::endl;
	//int steps= fingers->at(finger)->angleToSteps(angle);
	//mueve el dedo

	controller->setTarget(0, fingers->at(finger)->getPort(), steps*4);
	//actualiza la posicion
	fingers->at(finger)->setStepsPosition(steps);
	fingers->at(finger)->setAnglePosition(fingers->at(finger)->stepsToAngle(steps));
}

//GOAL POSITION DE LA MANO
void Hand::moveHand(int* goalPositHand){
	for(unsigned int i=0;i<fingers->size();i++){
		std::cout<<"finger "<<i<<" move to: "<<*goalPositHand<<std::endl;
		moveFinger(i,*goalPositHand);
		goalPositHand++;	
	}
}

//ACCESO A UN DEDO DE LA MANO
Finger* Hand::at(int index){
	if(index < 0){
		return NULL;
	}
	return fingers->at(index);
}

//NUMERO DE DEDOS QUE HAY
size_t Hand::size(){
	return fingers->size();
}

//AÑADIR DEDO
void Hand::add(Finger* finger){
	if(finger != NULL){
		fingers->push_back(finger);
	}
}

//PARADA DE EMERGENCIA
void Hand::emergencyStop(){
	for(unsigned int i=0;i<fingers->size();i++){
		setSpeedFinger(i, 0);
	}
}
