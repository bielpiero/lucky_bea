
#include "arm/Arm.h"

//CONSTRUCTOR
Arm::Arm(SerialPort* handController, std::string deviceName, float protocolVersion){
	portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
	packetHandler = dynamixel::PacketHandler::getPacketHandler(protocolVersion);

	vect_brazo = new std::vector<DynamixelMotor*>();

	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 1, speed1, arc1_grados, arcArtic1));
	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 2, speed2, arc2_grados, arcArtic2));
	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 3, speed3, arc3_grados, arcArtic3));
	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 4, speed4, arc4_grados, arcArtic4));
	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 5, speed5, arc5_grados, arcArtic5));
	vect_brazo->push_back(new DynamixelMotor(packetHandler, portHandler, 6, speed6, arc6_grados, arcArtic6));

	//mano_objeto = new Hand(handController);
	hand = new Hand(handController);

	goalPositionSteps1 = new int[vect_brazo->size()];
    goalPositionSteps2 = new int[vect_brazo->size()];
    goalPositionAngle1 = new int[vect_brazo->size()];
    goalPositionAngle2 = new int[vect_brazo->size()];

    handGoalPositAngle1 = new int[hand->size()];
    handGoalPositAngle2 = new int[hand->size()];
    handGoalPositSteps1 = new int[hand->size()];
    handGoalPositSteps2 = new int[hand->size()];
    
    speedArray = new int[vect_brazo->size()];
}

Arm::~Arm(){
	//borrar aqui;
}

//OPEN PORT
bool Arm::openPort(){
	bool open=false;
  	if (portHandler->openPort()){
    	RNUtils::printLn("Succeeded to open the port!");
    	open=true;
  	}
  	else{
  		RNUtils::printLn("Failed to open the port!");
  	}
  	return open;
}

//SET BAUDRATE DEL PUERTO
bool Arm::setPortBaudrate(const unsigned int baud){
	bool baudSet=false;
	if (portHandler->setBaudRate(baud)){
	    RNUtils::printLn("Succeeded to change the baudrate!");
	    baudSet=true;
	}
	else{
		RNUtils::printLn("Failed to change the baudrate!");
	}
  return baudSet;
}

//CERRAR PUERTO
void Arm::closePort(){
	portHandler->closePort();
}

//LEE UN BYTE DE CADA DynamixelMotor DEL BRAZO UNO A UNO
void Arm::armRead1Byte(uint16_t address){
	for (unsigned int i=0;i<vect_brazo->size();i++){
   		(vect_brazo->at(i))->read1Byte(address);
  }
}

//LEE DOS BYTES DE CADA DynamixelMotor DEL BRAZO UNO A UNO
void Arm::armRead2Bytes(uint16_t address){
	for (unsigned int i=0;i<vect_brazo->size();i++){
   		(vect_brazo->at(i))->read2Bytes(address);
  }
}

//ESCRIBE UN BYTE DE CADA DynamixelMotor DEL BRAZO UNO A UNO
void Arm::armWrite1Byte(uint16_t address,uint8_t value){
	for (unsigned int i=0; i<vect_brazo->size(); i++){
   		(vect_brazo->at(i))->write1Byte(address,value);
	}
}

//ESCRIBE DOS BYTES DE CADA DynamixelMotor DEL BRAZO UNO A UNO
void Arm::armWrite2Bytes(uint16_t address,uint16_t value){
	for (unsigned int i=0;i<vect_brazo->size();i++){
   		(vect_brazo->at(i))->write2Bytes(address,value);
  }
}

//ESCRIBIR 1 BYTE EN TODOS LOS MOTORES DEL BRAZO CON UN COMANDO
bool Arm::armSyncWrite1Byte(uint16_t address, int value){
	bool valid=false;
	bool escrito=true;
	bool dxl_addparam_result;
	int dxl_comm_result = COMM_TX_FAIL;
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, 1);
	uint8_t param_1_byte;
	param_1_byte = DXL_LOBYTE(value);
	
	for(unsigned int i=0;i<vect_brazo->size();i++) {
		dxl_addparam_result = groupSyncWrite.addParam(i+1,&param_1_byte);
		if (dxl_addparam_result != true){
     		fprintf(stderr, "[ID:%d] groupSyncWrite addparam failed",i);
     		escrito=false;
    	}		
	}

    if (escrito){
		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			//packetHandler->printTxRxResult(dxl_comm_result);
		} else {
			valid=true;
		}
	}
	else{
        RNUtils::printLn(" no puedes cambiar el ID de todos los motores a la vez \n");
	}
	groupSyncWrite.clearParam();
	return valid;
}


//ESCRIBIR 2 BYTES EN TODOS LOS MOTORES DEL BRAZO CON UN COMANDO
bool Arm::armSyncWrite2Bytes(uint16_t address,int* value){
	bool escrito=true;
	bool valid=false;
	bool dxl_addparam_result=false;
	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t param_goal_position[2];

	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, 2);
	
	for(unsigned int i=0;i<vect_brazo->size();i++){
		param_goal_position[0] = DXL_LOBYTE(value[i]);
    	param_goal_position[1] = DXL_HIBYTE(value[i]);
    	dxl_addparam_result = groupSyncWrite.addParam(i+1,param_goal_position);
		if (dxl_addparam_result != true){
     		fprintf(stderr, "[ID:%d] groupSyncWrite addparam failed \n",i+1);
     		escrito=false;
    	}	
	}
	
    if (escrito){
		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			//packetHandler->printTxRxResult(dxl_comm_result);
		} else {
			valid=true;
		}
	}
	groupSyncWrite.clearParam();
	return valid;
}

		
//LEER DE TODOS LOS MOTORES DEL BRAZO CON UN COMANDO EL MISMO DATO
bool Arm::armBulkRead(uint16_t address, uint8_t lenght){
	bool dxl_addparam_result;
	bool leido=true;
	int dxl_comm_result = COMM_TX_FAIL;
	bool available=false;
	int lecturas[vect_brazo->size()];

  	dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  	//indicamos qué direccion queremos leer
  	for (unsigned int i=0;i<vect_brazo->size();i++){
    	dxl_addparam_result = groupBulkRead.addParam(vect_brazo->at(i)->getID(), address, lenght);
    	if (dxl_addparam_result != true){
      		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", i+1);
      		leido=false;
    	}
  	}

  	if (leido){
  		dxl_comm_result = groupBulkRead.txRxPacket();	
  		if (dxl_comm_result != COMM_SUCCESS){
  	 		//packetHandler->printTxRxResult(dxl_comm_result);
  		}
  		else{
  			for(unsigned int i=0; i< vect_brazo->size();i++){
  				if (!(groupBulkRead.isAvailable(vect_brazo->at(i)->getID(), address, lenght))){
  					available=false;
  					fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", i+1);
  				}
  				//lo hago asi para q si available es falso una vez, no se reescriba
  			}
  			//entra aqui si todos los group bulk read estan disponibles
  			if(available){
  				for(unsigned int i=0; i< vect_brazo->size();i++){
  					lecturas[i]=groupBulkRead.getData(i,address,lenght);
  				}
  				for(unsigned int i=0; i< vect_brazo->size();i++){
  					std::cout<<"lectura[ "<<i+1<<" ] = "<< lecturas[i]<<std::endl;
  				}
  			}
  		}
  	}
  	if((!leido)||(!available)) return false;
  	else return true;
}

//PONER EL BRAZO UNICAMENTE EN POSICION INICIAL
void Arm::setArmToZero(){
	int zeros[vect_brazo->size()];
  	for (unsigned int i=0;i<vect_brazo->size();i++){
  		zeros[i]=vect_brazo->at(i)->getZero();
  	}
  	setSpeedSameTime(&zeros[0]);
  	//vect_brazo->at(4)->write2Bytes(addr_GoalPosition, vect_brazo->at(4)->getZero());
    //vect_brazo->at(5)->write2Bytes(addr_GoalPosition, vect_brazo->at(5)->getZero());
  	armSyncWrite2Bytes(addr_GoalPosition, &zeros[0]);

}

//PONER TODO EN POSICION INICIAL
void Arm::setToZero(){
	setArmToZero();
	hand->setToZero();
}

//SET TORQUE
void Arm::setTorque(int v){
	armSyncWrite1Byte(addr_Torque,v);
}

//GUARDA LA VELOCIDAD POR DEFECTO DE CADA DynamixelMotor DEL BRAZO EN UN ARRAY
void Arm::setBasicSpeedArray(){
	for (unsigned int i=0;i<vect_brazo->size();i++){
		speedArray[i]=vect_brazo->at(i)->getSpeed();
	}
}

//GUARDA LOS LIMITES DE CADA DynamixelMotor DEL BRAZO EN UN ARRAY
void Arm::setArmLimites(){
	for (unsigned int i=0;i<vect_brazo->size();i++){
		vect_brazo->at(i)->setLimites();
	}
}

//MIRA QUE MOTORES SE ESTAN MOVIENDO
//no tiene en cuenta a los dedos pq no dan feedback de si se estan moviendo
void Arm::waitFinishMove(){
	bool move_bool=false;
	int move_int[vect_brazo->size()];
	int torqueWarning[]={0,0,0,0,0,0};
	int torqueStop[]={0,0,0,0,0,0};
	int torqueMoving[vect_brazo->size()];
	bool torqueS=false;
	int lectura;
	int vez=0;
	long double sumVolt[]={0,0,0,0,0,0};
	long double sumCurrent[]={0,0,0,0,0,0};
	double veces=0;
	
	do{
		int result=0;
		for (unsigned int i=0;i<vect_brazo->size();i++){
			if((i==5)&&(vez<vecesLectTorque)){
				vez++;
			}
			if (vez==vecesLectTorque){
				torqueMoving[i]=vect_brazo->at(i)->read2Bytes(addr_PresentLoad);
				//std::cout<<"torque[ "<<i+1<<" ]="<<torqueMoving[i]<<std::endl;
				if (i==4) vez++;
			}
			move_bool=false;
			move_int[i]=vect_brazo->at(i)->read1Byte(46);
			if(move_int[i]!=0){
				result++;
				//check torque de los motores que se mueven
				if(vez>vecesLectTorque){
					lectura=vect_brazo->at(i)->read2Bytes(addr_PresentLoad);
					//std::cout<<"lectura en "<<i+1<<"es "<<lectura<<" position "<<vect_brazo->at(i)->getCurrentPositSteps()<<std::endl;
					if(abs(lectura-torqueMoving[i])>umbralTorqueStop){
						if(abs(lectura-torqueMoving[i])<900){
							torqueStop[i]++;
							std::cout<<"torqueStop+1 en"<<i+1<<std::endl;
							if(torqueStop[i]>5){
								torqueS=true;
								std::cout<<"torque problem en DynamixelMotor "<<i+1<<" STOP"<<std::endl;
								//emergencyStop();
							}
						}
					}
					else if(abs(lectura-torqueMoving[i])>umbralTorqueW){
						torqueWarning[i]++;
						//std::cout<<"torqueWarning+1 en"<<i+1<<std::endl;
						if(torqueWarning[i]>5){
							std::cout<<"torque warning en DynamixelMotor "<<i+1<<std::endl;
						}
					}
				}
			}
		
			else {
                vect_brazo->at(i)->write1Byte(addr_Led,DISABLE);
			}

		}
		if(result==0){
			move_bool=false;
		}
		else move_bool=true;
	}while(move_bool&&(!torqueS));
	//intento mover a posicion inicial
	if(torqueS and !(toInitialPosit)){
		//setGoalPosition(10);
		easyMovPredef(1);
	}
	//si ya esta yendo a posic inicial
	else if (torqueS && toInitialPosit){
		std::cout<<"Problema para ir a posic inicial"<<std::endl;
	}
	//si ha hecho el mov sin problema
	else {
		//std::cout<<"MOVIM FINISHED"<<std::endl;
		//si estaba yendo a posic inicial
		if(toInitialPosit){
			toInitialPosit=false;
		}
	}
}

//COMPRUEBA QUE EN AL MOVER UN DynamixelMotor LA MANO NO SE CHOCA CON EL CUERPO
//para un DynamixelMotor se le pasa la ID y nueva posicion del DynamixelMotor
//quizas tengo que cambiar la ELBOW_HAND_DISTANCE segun el dedo este stirado o no
bool Arm::checkNoChoque(int ID, int new_pos){
	bool no_choca = true;
	int aux = goalPositionAngle1[ID];
	goalPositionAngle1[ID] = new_pos;
	
	if(goalPositionAngle1[0] < 25){
		if(goalPositionAngle1[2] < 30){
			int h2 = std::abs(SHOULDER_ELBOW_DISTANCE * sin(goalPositionAngle1[1]));
			int h4 = std::abs(ELBOW_HAND_DISTANCE * sin(goalPositionAngle1[3]));
			if(h2 < h4){
				no_choca=false;
			}
		}
	}
	goalPositionAngle1[ID] = aux;
	return no_choca;
}

//COMPRUEBA SI SE CHOCA AL MOVER VARIOS MOTORES
//solo mira el choque en la posicion final, no en la trayectoria
/*bool Arm::checkNoChoque(int max,int *futurosAngulos){
	bool no_choca=true;
	int DistElbowHand=ELBOW_HAND_DISTANCE+FINGER_LENGTH*sin(max);
	if(futurosAngulos[0]<25){
		std::cout<<"	ang1 es menor que 25" <<std::endl;
		if(futurosAngulos[2] < 30){
			std::cout<<"	ang2 es menor que 30" <<std::endl;
			int h2 = abs(SHOULDER_ELBOW_DISTANCE*sin(futurosAngulos[1]));
			int h4 = abs(DistElbowHand*sin(futurosAngulos[3]));
			std::cout<<"	h2:"<<h2<<"y h4: "<<h4<<std::endl;
			if(h2<h4){
				std::cout<<"	h2 < h4: "<<std::endl;
				no_choca=false;
			}
		}
	}
	return no_choca;
}*/

//EMERGENCY STOP BRAZO
void Arm::armEmergencyStop(){
	int disable_array[]={0,0,0,0,0,0};
	armSyncWrite2Bytes(addr_MovingSpeed,&disable_array[0]);
}

//EMERGENCY STOP TODO
void Arm::emergencyStop(){
	armEmergencyStop();
	hand->emergencyStop();
}

//SET ARRAY DE POSICIONES
void Arm::setGoalPositionKeyboard(std::vector<uint16_t> positions){	
  	uint16_t goal_angle_int;
  	bool mov_valid=false;
		
	for (unsigned int i = 0; i < positions.size(); i++){	
			//recibe un angulo valido para cada DynamixelMotor
		if(i < vect_brazo->size()){
			mov_valid = vect_brazo->at(i)-> checkAngle(positions.at(i));
		} else {
			mov_valid = hand->at(i-vect_brazo->size())->checkAngle(positions.at(i));
		}
		if(mov_valid){
  			if(i < vect_brazo->size()){
  				goalPositionAngle1[i] = positions.at(i);
  			} else {
  				handGoalPositAngle1[i - vect_brazo->size()] = positions.at(i);
  			}
  		}
	}
		//std::cout<<"sali del for"<<std::endl;
		//comprueba que todos los angulos en conjunto llevan al brazo a una posicion posible
		//si se puede ejecutar el movimiento -->convierte a pasos
	//if(checkNoChoque(&goalPositionAngle1[0])){
		for (unsigned int i = 0; i < positions.size(); i++){	
			if(i < vect_brazo->size()){
				goalPositionSteps1[i] = vect_brazo->at(i)->pasosLogicos(goalPositionAngle1[i]);
			} else{
				handGoalPositSteps1[i - vect_brazo->size()] = hand->at(i - vect_brazo->size())->angleToSteps(handGoalPositAngle1[i - vect_brazo->size()]);	
			}
		}
	//}
}

 //CALCULAR SPEED PARA QUE EL MOV DE LOS MOTORES ACABE A LA VEZ
void Arm::setSpeedSameTime(int* future_posit){

	double distancia_steps[vect_brazo->size()];	
	double tiempo[vect_brazo->size()];		
	int speed_new[vect_brazo->size()];
	double biggestTime=0.0;
	int lento=0;
	int enableArray[] = {1, 1, 1, 1, 1, 1};
	for(unsigned int i=0; i<vect_brazo->size();i++){
		distancia_steps[i]=abs(*future_posit - vect_brazo->at(i)->getCurrentPositSteps());
		tiempo[i]= distancia_steps[i]/speedArray[i];
		//std::cout<<"ID "<<i+1<<" dist: "<<distancia_steps[i]<<"\ttiempo "<<tiempo[i]<<std::endl;
		if (tiempo[i]>biggestTime) {
			biggestTime=tiempo[i];	
			lento=i;
			//std::cout<<"lento es "<<i+1<<std::endl;
		}	
		future_posit++;	
	}
	
	for(unsigned int i=0;i<vect_brazo->size()-1;i++){
		if (distancia_steps[i]!=0 && i!=lento ){
		//std::cout<<"speed prev "<<i+1<<" is "<< speed_new[i]<<std::endl; 	
			speed_new[i]=distancia_steps[i]/biggestTime;
			if(speed_new[i]<1){
				speed_new[i]=1.0;
			}			
		}
		else{
			speed_new[i]=speedArray[i];
		}	
		if(i==4 ){
			speed_new[i]*=2; //valor a estima, comprobar con la mano puesta	
		}
	}
	speed_new[5]=speed_new[4];
	armSyncWrite2Bytes(addr_MovingSpeed,&speed_new[0]);
	armSyncWrite2Bytes(addr_Led,&enableArray[0]);
}

//GET POSICION DE ARCHIVO
bool Arm::setGoalPosition(ArmGesture* gesture){
	bool problem = 0;

	/*if(seleccion==10){
		toInitialPosit=true;
	}*/

	for(unsigned int i = 0; i < gesture->stateAt(0)->motorsSize(); i++){
		//i entre 0 y 5 es GP de los motores del brazo
		if (i < vect_brazo->size()){
			goalPositionAngle1[i] = std::atoi(gesture->stateAt(0)->motorAt(i)->getDegrees().c_str());
			if(gesture->getType() == "1"){
				goalPositionAngle2[i] = std::atoi(gesture->stateAt(1)->motorAt(i)->getDegrees().c_str());
			}
		}
		//i mayor que 5 es DynamixelMotor del dedo
		else {
			handGoalPositAngle1[i - vect_brazo->size()] = std::atoi(gesture->stateAt(0)->motorAt(i)->getDegrees().c_str());
			if(gesture->getType() == "1"){
				handGoalPositAngle2[i - vect_brazo->size()] = std::atoi(gesture->stateAt(1)->motorAt(i)->getDegrees().c_str());
			}
		}					
	}			
			
	//una vez encontrado el gesto
	for(unsigned int i = 0; i < gesture->stateAt(0)->motorsSize(); i++){
		//motores dynamixel
		if(i < vect_brazo->size()){
			if(vect_brazo->at(i)->checkAngle(goalPositionAngle1[i])){
				goalPositionSteps1[i] = vect_brazo->at(i)->pasosLogicos(goalPositionAngle1[i]);
			} else {
				problem = true;
			}
			if(gesture->getType() == "1"){
				if(vect_brazo->at(i)->checkAngle(goalPositionAngle2[i])){
					goalPositionSteps2[i] = vect_brazo->at(i)->pasosLogicos(goalPositionAngle2[i]);
				} else {
					problem = true;
				}
			}
		} else{ //motores de la mano
			if(hand->at(i - vect_brazo->size())->checkAngle(handGoalPositAngle1[i - vect_brazo->size()])){
				handGoalPositSteps1[i - vect_brazo->size()] = hand->at(i - vect_brazo->size())->angleToSteps(handGoalPositAngle1[i - vect_brazo->size()]);
				//std::cout<<"goalPositionHand1 del "<<hand->at(i-6)->getFingerName()<<" = "<<handGoalPositAngle1[i]<<std::endl;
				//std::cout<<" en pasos es "<<handGoalPositSteps1[i]<<std::endl;
			} else {
				problem = true;
			}
			if(gesture->getType() == "1"){
				if(hand->at(i - vect_brazo->size())->checkAngle(handGoalPositAngle2[i - vect_brazo->size()])){
					handGoalPositSteps2[i - vect_brazo->size()] = hand->at(i - vect_brazo->size())->angleToSteps(handGoalPositAngle2[i - vect_brazo->size()]);
				} else {
					problem = true;
				}
			}
		}
	}
	return problem;
}
//al salir de la funcion los vectores estan en steps

void Arm::easyMovPredef(int m){
	if(m==1){
		//std::cout<<"easymove"<<std::endl;
		setSpeedSameTime(goalPositionSteps1);
   		hand->moveHand(handGoalPositSteps1);
    	armSyncWrite2Bytes(addr_GoalPosition, goalPositionSteps1);	
	}
	if(m==2){
		setSpeedSameTime(goalPositionSteps2);
    	hand->moveHand(handGoalPositSteps2);
    	armSyncWrite2Bytes(addr_GoalPosition, goalPositionSteps2);
	}
    waitFinishMove();
    armSyncWrite2Bytes(addr_MovingSpeed, speedArray); 
}

void Arm::predifinedMovement(ArmGesture* gesture){

	if(not setGoalPosition(gesture)){
		//movimientos estaticos
		if(gesture->getType() == "0"){	
			easyMovPredef(1);		
		} else { 	//movimientos dinamicos
			for(unsigned int i = 0; i < 7; i++){
				if (i % 2 == 0){
					easyMovPredef(1);
				} else{
					easyMovPredef(2);
				}
			}
		}
	}

    RNUtils::sleep(2000);
    setToZero(); 
    waitFinishMove();		
}

void Arm::bulkControl(std::vector<uint16_t> positions){
	
	setGoalPositionKeyboard(positions);
	setSpeedSameTime(goalPositionSteps1);		//escribo velocidad en cada DynamixelMotor para q lleguen a la vez

	armSyncWrite2Bytes(addr_GoalPosition, goalPositionSteps1);	//Escribe goal position
	hand->moveHand(handGoalPositAngle1);
	waitFinishMove();
	armSyncWrite2Bytes(addr_MovingSpeed, speedArray);	//restaura velocidad
}

void Arm::singleMotor(int id, uint16_t angle){
	armSyncWrite2Bytes(addr_MovingSpeed, speedArray);
	RNUtils::printLn("Arm (singleMotor): {ID: %d, angle: %u}", id, angle);
	//armShowAngleLimits();
  	int steps_dxl;
  	int steps_hand;
  	bool mov_valid =false;
  	bool no_choca = true;
  	
  	
	//comprueba que es un angulo valido para las articulaciones
	if(id < vect_brazo->size()){
		mov_valid = vect_brazo->at(id)->checkAngle(angle);	
	} else{ //comprueba que es un angulo valido para los dedos
		mov_valid = hand->at(id - vect_brazo->size())->checkAngle(angle);
	}	
	//si no es un angulo valido le muestra los limites al usuario			
	if(!mov_valid){
		vect_brazo->at(id)->showAngleLimits();
	} else {
		//no_choca = checkNoChoque(id, angle);
	}
				
	//setMostrarDatos();
	//convierto los angulos en pasos
	if(no_choca){
		if(id < vect_brazo->size()){
			steps_dxl = vect_brazo->at(id)->pasosLogicos(angle);
			vect_brazo->at(id)->write1Byte(addr_Led,ENABLE);
			vect_brazo->at(id)->write2Bytes(addr_GoalPosition,steps_dxl);
			waitFinishMove();
		} else{
			steps_hand=hand->at(id-vect_brazo->size())->angleToSteps(angle);
			hand->moveFinger(id - vect_brazo->size(), steps_hand);
		}
	}
    
}


void Arm::PPT(){
	
	/*while(1){
		RNUtils::printLn("Press any key to continue! (or press ESC to quit!)\n");
    	if (FuncBasic::getch() == ESC_ASCII_VALUE)
      		break;
		std::cout<<"juguemos 3 partidas!!!"<<std::endl;
		int resultado=0;
		std::string r_str;
		int veces=0;
		//juega tres partidas
		do{
			for (unsigned int i=0;i<vect_brazo->size();i++){
				speedArray[i]=vect_brazo->at(i)->getSpeed()*0.8;
				std::cout<<"speed for "<<i+1<<" is: "<<speedArray[i]<<std::endl;
			}
			setGoalPosition(99);
			easyMovPredef(1);
			srand(time(NULL));	
			int juego= rand()%3+100;
			setGoalPositionFile (juego);
			RNUtils::printLn("Press any key to continue! (or press ESC to quit!)\n");
    		if (FuncBasic::getch() == ESC_ASCII_VALUE)
      			break;
			
			easyMovPredef(1);		
    		do{
    			std::cout<<"¿he ganado (G), perdido(P) o empate (E)?"<<std::endl;
    			std::getline(std::cin,r_str);
    		}while(!((!(r_str.compare("G")))||(!(r_str.compare("P")))||(!(r_str.compare("E")))));
    		
    		if(!(r_str.compare("G"))){
    			resultado++;
    			veces++;
    		}
    		else if(!(r_str.compare("P"))){
    			veces++;
    		}
    		std::cout<<"llevo "<<veces<< "veces"<<std::endl;
		}while (veces<3);
		std::cout<<"resultado: TU: "<<3-resultado<<" YO: "<<resultado<<std::endl;
		if(resultado>1){
			std::cout<<"GANE :) "<<std::endl;
			setGoalPositionFile(8);
			easyMovPredef(1);
		}
		else{
			std::cout<<"ENHORABUENAA "<<std::endl;
			setGoalPositionFile(9);
			easyMovPredef(1);
		}
	}*/
	
}


//INICIALIZAR BRAZO
void Arm::init(){
	
    setArmLimites(); 
	setTorque(ENABLE);
	//set alarmLED y alarmShutdown -- no hace falta pq son EEPROM
	armSyncWrite1Byte(addr_Led, ENABLE);
	//inicializo el array de velocidades con la veloc usual de cada DynamixelMotor
	setBasicSpeedArray(); 
	//set speed de cada DynamixelMotor xq se borra al desenchufar
    armSyncWrite2Bytes(addr_MovingSpeed, &speedArray[0]);
	//position();
	//showLimits();
	//armShowStepsLimits();
}

//APAGAR BRAZO
void Arm::shutdown(){
	armSyncWrite2Bytes(addr_MovingSpeed, &speedArray[0]);
	setToZero();
	waitFinishMove();
	armSyncWrite1Byte(addr_Led,DISABLE);
	setTorque(DISABLE);
}

