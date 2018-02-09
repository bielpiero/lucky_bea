#ifndef ARM_MOTOR_H
#define ARM_MOTOR_H

//#include "Arm.h"
//#include "FuncBasic.h"
#include "RNUtils.h"


//Direcciones tipicas de la tabla de control del motor
#define addr_ID 				3
#define addr_CWAngleLimit 		6
#define addr_CCWAngleLimit 		8
#define addr_Alarm				18
#define addr_Torque	 			24
#define addr_Led 				25
#define addr_GoalPosition 		30
#define addr_MovingSpeed 		32
#define addr_PresentPosition 	36
#define addr_PresentSpeed 		38
#define addr_PresentLoad 		40
#define addr_Voltage			42 
#define addr_Current			68 

#define zero3  3071
#define zero5  511
#define max10bits 65535

//clase motor
class DynamixelMotor {

	private:
		uint ID;
		uint16_t zero;
		uint16_t speed;
		uint16_t dxl_goal_position;
		uint8_t lectura1b;
		uint16_t lectura2b;
		uint8_t dxl_error;
		int dxl_comm_result;
		int lim_min_steps;
		int lim_max_steps;
		int arcGrados;
		int arcSteps;

		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;

	public:
		//constructor
		DynamixelMotor (dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,uint id,uint16_t speed, int arc, int aA);
		//destructor
		
		//get and set
		uint16_t getZero();
		void setZero(int);
		uint getID();
		uint16_t getSpeed();
		

		//read and write
		void write1Byte( uint16_t address, uint8_t value);
		void write2Bytes( uint16_t address, uint16_t value);
		int read1Byte( uint16_t address);
		int read2Bytes( uint16_t address);


		//funciones
		int AngletoSteps(float angle);
		float StepstoAngle(int steps);
		uint16_t getCurrentPositSteps();
		uint16_t getCurrentPositAngle();
		void setGoalPosition(int angle);		
		bool checkSteps(int steps);
		bool checkAngle(int angle);
		void showAngleLimits();
		void showStepsLimits();
		int pasosLogicos(int steps);
		int anguloLogico(int steps);
		void setLimites();
		void leerLimInternos();
};

#endif

//no tiene parámetros porque se leen y guardan en address table

//previamente a cada motor se le asignó su ID, que no se va a modificar nunca,
// al igual que el baudrate y la alarma 

