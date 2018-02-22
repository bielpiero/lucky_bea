#include "DynamixelMotor.h"


//FUNCIONES DE LA CLASE DynamixelMotor 


DynamixelMotor::DynamixelMotor (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, uint id, uint16_t speed, int maxDegrees, int maxSteps){
    this->portHandler = portHandler;
    this->packetHandler = packetHandler;
    this->ID=id;
    this->speed = speed;
    this->arcGrados = maxDegrees;
    this->arcSteps = maxSteps;
}

//get 
uint16_t DynamixelMotor::getZero(){
    return zero;
}
void DynamixelMotor::setZero(int z){
    this->zero=z;
}
uint DynamixelMotor::getID(){
    return ID;
}

uint16_t DynamixelMotor::getSpeed(){
    return speed;
}

//SET LIMITES SEGUN LA POSICION INICIAL DEL BRAZO
void DynamixelMotor::setLimites(){
    if(ID == 3){
        lim_min_steps = 0;
        lim_max_steps = 4095;
        setZero(zero3);
    } else if((ID == 5) || (ID == 6)){
        lim_min_steps = 1;
        lim_max_steps = 1023;
        setZero(zero5);
    } else if(ID == 1){
        lim_min_steps = read2Bytes(addr_PresentPosition);
        //write2Bytes(6,lim_min_steps);
        lim_max_steps = lim_min_steps+arcSteps;
        if (lim_max_steps > max10bits){
            lim_max_steps = lim_max_steps - max10bits;
        }
    //write2Bytes(8,lim_max_steps);
        setZero(lim_min_steps);
    } else {
        lim_max_steps = read2Bytes(addr_PresentPosition);
        lim_min_steps = lim_max_steps - arcSteps;
        if (lim_min_steps < 0){
            lim_min_steps = max10bits - arcSteps + lim_max_steps;
        }
        setZero(lim_max_steps);
    }
    //RNUtils::printLn("ID: %d, lim_min_steps: %d, lim_max_steps: %d", this->ID, this->lim_min_steps, this->lim_max_steps);
    
}

//ESCRIBIR 1 BYTE
void DynamixelMotor::write1Byte (uint16_t address, uint8_t value){
    bool write;
    if(address == addr_ID){
        write=false;
        std::string yon;
        do{
            RNUtils::printLn("WARNING: Want to change the DynamixelMotor's id? (Y/N) ");
            std::getline(std::cin, yon);
        } while(!((!(yon.compare("Y")))||(!(yon.compare("N")))));

        if(!(yon.compare("Y"))){
            write =true;
        } else {
            write=false;
        }
    } else {
        write = true;
    }
    
    if (write){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,ID, address, value, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            //packetHandler->printTxRxResult(dxl_comm_result);
        } else if (dxl_error != 0){
            //packetHandler->printRxPacketError(dxl_error);
        }
        
    }
}

//ESCRIBIR 2 BYTES
void DynamixelMotor::write2Bytes(uint16_t address, uint16_t value){                                          
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, address, value,&dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
    } else if (dxl_error != 0){
        //packetHandler->printRxPacketError(dxl_error);
    }
    //else printf("2 Bytes successfully written \n");
}

//LEER 1 BYTE
int DynamixelMotor::read1Byte(uint16_t address){
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, ID, address, &lectura1b, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
    } else if (dxl_error != 0){
        //packetHandler->printRxPacketError(dxl_error);
    }
    return lectura1b;
}

//LEER 2 BYTES
int DynamixelMotor::read2Bytes(uint16_t address){
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, address, &lectura2b, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        //packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
        //packetHandler->printRxPacketError(dxl_error);
    }
    return lectura2b;
}

//CONVERTIR ANGULO EN PASOS DEL DynamixelMotor
int DynamixelMotor::AngletoSteps(float angle){
    int steps;
    if (ID > 4){
        steps = angle / 0.29;
    } else {
        steps = angle / 0.088;
    }
    return steps;
}

//CONVERTIR PASOS EN ANGULO
float DynamixelMotor::StepstoAngle(int steps){
    if (ID>4){
        return steps*0.29;
    } else{
        return steps*0.088;
    }  
}

//LEER POSICIÓN ACTUAL DEL DynamixelMotor EN PASOS
uint16_t DynamixelMotor::getCurrentPositSteps(){
    return read2Bytes(addr_PresentPosition);
}

//LEER POSICIÓN ACTUAL DEL DynamixelMotor EN ANGULO
uint16_t DynamixelMotor::getCurrentPositAngle(){
    int s= read2Bytes(addr_PresentPosition);
    //std::cout<<"mi posic en pasos es: "<<s<<std::endl;
    return anguloLogico(s);
}

//SET POSICION A LA QUE VA EL DynamixelMotor
void DynamixelMotor::setGoalPosition(int angle){
    int steps = AngletoSteps(angle);
    write2Bytes(addr_GoalPosition,steps);
}

//COMPRUEBA QUE EL DynamixelMotor PUEDA MOVERSE A ESE STEP
bool DynamixelMotor::checkSteps(int steps){
    bool valido=false;
    if((lim_min_steps<=steps) and (lim_max_steps>=steps)){
        valido=true;
    }
    return valido;
}

//COMPRUEBA QUE EL DynamixelMotor PUEDA MOVERSE A ESE ANGULO GEOMETRICO
bool DynamixelMotor::checkAngle(int angle){
    bool valido=false;  
    if((angle>=0) and (angle <= this->arcGrados)){
        valido=true;      
    }
    return valido;
}

//MOSTRAR LIMITES GEOMETRICOS DEL BRAZO
void DynamixelMotor::showAngleLimits(){
    std::cout<<"ID: "<<ID<<" Min angle: 0"<< "\t"<<" Max angle: "<< arcGrados<< std::endl;
}

//MOSTRAR STEPS LIMITE DEL BRAZO
void DynamixelMotor::showStepsLimits(){
    std::cout<<"ID: "<<ID<<" lim_min: "<<lim_min_steps<<"\t lim max: "<<lim_max_steps<<std::endl; 
}

//CONVIERTE EL ANGULO VISUAL DE LA ARTICULACIÓN EN EL REAL DE LA ARTICULACION
int DynamixelMotor::pasosLogicos(int angle){
    int steps_dxl;
    if((ID == 4) or (ID == 2)){
    //std::cout<<".................angle: "<<angle<<" lim_max_steps: "<<lim_max_steps<<" lim_min_steps: "<<lim_min_steps<<std::endl;
        if(lim_max_steps < lim_min_steps){
            steps_dxl=-angle*(lim_max_steps + max10bits-lim_min_steps)/arcGrados + lim_max_steps;
        } else{
            steps_dxl= -angle*(lim_max_steps-lim_min_steps)/arcGrados+lim_max_steps;
        }
    }
    else{
        if((ID==1)&&(lim_min_steps>lim_max_steps)){
            steps_dxl=angle*(lim_max_steps + max10bits - lim_min_steps)/arcGrados + max10bits-lim_min_steps;
        } else{
            steps_dxl= angle*(lim_max_steps-lim_min_steps)/arcGrados+lim_min_steps; 
        } 
    }
    //std::cout<<"angulo pedido es "<<angle<<" y pasos logicos son "<<steps_dxl<<std::endl;
    return steps_dxl;
}

//CONVIERTE EL ANGULO VISUAL EN PASOS DEL DynamixelMotor
int DynamixelMotor::anguloLogico(int steps){
    int angle;
    if((ID == 4) or (ID == 2)){
        angle = ((steps-lim_max_steps)*arcGrados)/(lim_min_steps-lim_max_steps);
    }
    else{
        angle=(steps-lim_min_steps)*arcGrados/(lim_max_steps-lim_min_steps);
    }
    //std::cout<<" Los pasos logicos son "<<steps<<"y el angulo pedido es "<<angle<<std::endl;
    return angle;
}

void DynamixelMotor::leerLimInternos(){
    std::cout<<"lim inferior de: "<<ID<<" es "<<read2Bytes(6)<<std::endl;
    std::cout<<"lim superior de: "<<ID<<" es "<<read2Bytes(8)<<std::endl;
}
