#include "RNFuzzyHallDNC.h"
#include <cmath>
#include <iostream>
using namespace std;

RNFuzzyHallDNC::RNFuzzyHallDNC(){
	radioDoris = 0.25;
	wideRegLimit = 120; //60grados, 120sectores
	distMax = 8;
	distSeg = 1;


	deltaTheta = 0;
	sentidoGiro = 0;
	

	// FUZZY CONTROLLER
	engine = new fl::Engine;
	engine->setName("FuzzyDNC");
	engine->setDescription("");
	
	//distances in meters
	distObs = new fl::InputVariable;
	distObs->setName("distObs");
	distObs->setDescription("");
	distObs->setEnabled(true);
	distObs->setRange(0, std::numeric_limits<double>::infinity());
	distObs->setLockValueInRange(true);
	distObs->addTerm(new fl::Triangle("close", -std::numeric_limits<double>::infinity(), 0, 0.6));
	distObs->addTerm(new fl::Triangle("inzone", 0, 0.4, 0.8));
	distObs->addTerm(new fl::Triangle("far", 0.4, 0.8, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(distObs);


	//Fuzzy Factor adimensional
	factor = new fl::OutputVariable;
	factor->setName("factor");
	factor->setDescription("");
	factor->setEnabled(true);
	factor->setRange(0, 1);
	factor->setLockValueInRange(true);
	factor->setAggregation(new fl::AlgebraicSum);
	factor->setDefuzzifier(new fl::Centroid(100));
	factor->setDefaultValue(0.0);
	factor->setLockPreviousValue(false);
	factor->addTerm(new fl::Triangle("null", -0.1, 0, 0.1));
	factor->addTerm(new fl::Triangle("half", 0.5, 0.6, 0.7));
	factor->addTerm(new fl::Triangle("full", 0.9, 1, 1.1));
	engine->addOutputVariable(factor);


	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

	ruleBlock->addRule(fl::Rule::parse("if distObs is close then factor is full", engine));
	ruleBlock->addRule(fl::Rule::parse("if distObs is inzone then factor is half", engine));
	ruleBlock->addRule(fl::Rule::parse("if distObs is far then factor is null", engine));
	
	engine->addRuleBlock(ruleBlock);
	fl::fuzzylite::setDebugging(false);
}

RNFuzzyHallDNC::~RNFuzzyHallDNC(){
}	


void RNFuzzyHallDNC::getSystemInput(double distance, double* factor){

	this->distObs->setValue(distance);

	engine->process();

	*factor = this->factor->getValue();
}



int RNFuzzyHallDNC::SectorDifference(int alfa, int beta){
	int delta1 = abs(alfa-beta);
	int delta2 = 720 - delta1;
	
	if(delta1<=delta2) return delta1;
	else return delta2;
}

float RNFuzzyHallDNC::AngleDifference(float alfa, float beta, int* giro){
	float delta;
	float delta1 = abs(alfa-beta);
	float delta2 = 720 - delta1;
	//Delta
	if(delta1<=delta2) delta = delta1;
	else delta = delta2;
	//Giro
	if((alfa>=0) && (beta>=0)){
		if(alfa > beta){
			*giro = -1; //CW
		}
		else if(beta > alfa){
			*giro = 1; //CCW
		}
		else *giro = 0;
	}
	else if((alfa<0) && (beta<0)){
		if(alfa > beta){
			*giro = -1; //CW
		}
		else if(beta > alfa){
			*giro = 1; //CCW
		}
		else *giro = 0;
	}
	else{
		if(alfa>=0){
			if(delta1>=delta2) *giro = -1; //CW
			else if (delta2>delta1) *giro = 1; //CCW
		}
		else if (beta>=0){
			if(delta1>=delta2) *giro = 1; //CCW
			else if (delta2>delta1) *giro = -1; //CW
		}
	}
	
	return delta;
	
}

float RNFuzzyHallDNC::AngleDifferenceCW(float alfa, float beta){
	float delta;
	float delta1 = abs(alfa-beta);
	float delta2 = 720 - delta1;

	//Giro
	if((alfa>=0) && (beta>=0)){
		if(alfa > beta){
			delta = delta1;
		}
		else if(beta > alfa){
			delta = delta2;
		}
		else delta = 0;
	}
	else if((alfa<0) && (beta<0)){
		if(alfa > beta){
			delta = delta1;
		}
		else if(beta > alfa){
			delta = delta2;
		}
		else delta = 0;
	}
	else{
		if(alfa>=0){
			delta = delta1;
		}
		else if (beta>=0){
			delta = delta2;
		}
	}
	
	return delta;
	
}

float RNFuzzyHallDNC::AngleDifferenceCCW(float alfa, float beta){
	float delta;
	float delta1 = abs(alfa-beta);
	float delta2 = 720 - delta1;

	//Giro
	if((alfa>=0) && (beta>=0)){
		if(alfa > beta){
			delta = delta2;
		}
		else if(beta > alfa){
			delta = delta1;
		}
		else delta = 0;
	}
	else if((alfa<0) && (beta<0)){
		if(alfa > beta){
			delta = delta2;
		}
		else if(beta > alfa){
			delta = delta1;
		}
		else delta = 0;
	}
	else{
		if(alfa>=0){
			delta = delta2;
		}
		else if (beta>=0){
			delta = delta1;
		}
	}
	
	return delta;
	
}

float RNFuzzyHallDNC::AngleMean(float alfa, float beta){
	float delta = 0; int sentido = 0;
	float media = 0;
	
	alfa = LimitAngle(alfa);
	beta = LimitAngle(beta);
	
	delta = AngleDifference(alfa,beta,&sentido);
	
	if(sentido == -1){
		media = alfa - delta/2;
	}
	else if(sentido == 1){
		media = alfa + delta/2;
	}
	else media = alfa;
	
	media = LimitAngle(media);
	
	
	return media;
	
}

float RNFuzzyHallDNC::LimitAngle(float angle){
	if(angle > PI) return (angle-2*PI);
	else if(angle < -PI) return (angle+2*PI);
	else return angle;
	
}

float RNFuzzyHallDNC::getDeltaTheta(LaserScan* laserData, ArPose* currPose, ArPose* goalPose){
	
	/* REASGINAR LASER*/
	
	float laser[laserData->size()];
	int i = 0;
	for(i = 0; i<laserData->size(); i++){
		laser[i] = laserData->getRange(i);
	}

	/* REASIGNAR POSE */
	auxPose pos;
	pos.x = currPose->getX();
	pos.y = currPose->getY();
	pos.angle = currPose->getTh();
	
	/* REASIGNAR GOAL */
	auxPose goal;
	goal.x = goalPose->getX();
	goal.y = goalPose->getY();
	goal.angle = goalPose->getTh();

	//Variables del algoritmo
	float pnd[laserData->size()];
	
	int gaps[50]; int gapsTotal = 0;
	
	int regions[50][2]; int regionsTotal = 0;
	int regNav[2]; int gapNav;
	int situacion = 0;
	
	/************** NEARNESS DIAGRAM **************/
	for(i=0; i<LASER_DIM; i++){
		pnd[i] = distMax - laser[i];
	}
	
	/************** GAPS *************************/
	//Artificial gap
	int gapGoal = 0;
	if((goal.sector > 0)&&(goal.sector < laserData->size())){
		if( sqrt((goal.x-pos.x)*(goal.x-pos.x)+(goal.y-pos.y)*(goal.y-pos.y)) <= laser[goal.sector]){
			pnd[goal.sector] = 0;
			gapGoal = goal.sector;
		}
	}

	//Finding discontinuities
	int noGap = 1;
	for(i=0; i<laserData->size()-1; i++){
		if(abs(pnd[i]-pnd[i+1]) > 2*radioDoris*2){ //ultimo 2 es margen
			if(pnd[i]>pnd[i+1]) gaps[gapsTotal] = i;
			else gaps[gapsTotal] = i+1;
			gapsTotal++;
			noGap = 0;
			
		}
	}
	
	/************** REGIONS *************************/			
	//Finding regions between gaps
	if(noGap == 0){ //Means there are GAPS
		//Move index to avoid segm fault
		if(gaps[0]==0) gaps[0] = 1;
		if (gaps[gapsTotal-1]==laserData->size()-1) gaps[gapsTotal-1] = laserData->size()-2;
		
		//CASE 1: First GAP is rising
		if(pnd[gaps[0]-1] < pnd[gaps[0]+1]){
			regions[regionsTotal][0] = 0;
			regions[regionsTotal][1] = gaps[0];
			regionsTotal++;
		}

		//CASE 2: Rest of the GAPS
		if(gapsTotal > 1){
			for(i=0;i<(gapsTotal-1);i++){
				//Rising to the left
				if(pnd[gaps[i]-1]>pnd[gaps[i]+1]){
					regions[regionsTotal][0] = gaps[i];
					regions[regionsTotal][1] = gaps[i+1];
					regionsTotal++;
					
				}			
				//Rising to the right
				else if(pnd[gaps[i+1]-1]<pnd[gaps[i+1]+1]){
					regions[regionsTotal][0] = gaps[i];
					regions[regionsTotal][1] = gaps[i+1];
					regionsTotal++;
				}
			}
		}
		//CASE 3: Last GAP
		if(pnd[gaps[gapsTotal-1]-1]>pnd[gaps[gapsTotal-1]+1]){
			regions[regionsTotal][0] = gaps[gapsTotal-1];
			regions[regionsTotal][1] = laserData->size()-1;
			regionsTotal++;
			
		}
		
	}
	else{ //Means there are NO GAPS, all is REGION
		regions[regionsTotal][0] = 0;
		regions[regionsTotal][1] = laserData->size()-1;
		regionsTotal++;
	}
	
	//Filter for contiguous regions
	i=0; int j;
	while(i<regionsTotal-1){
		if(regions[i][1]==regions[i+1][0]){
			regions[i][0]= regions[i][0];
			regions[i][1]= regions[i+1][1];
			if((i+1)== (regionsTotal-1)){
				regionsTotal--;
			}
			else{
				for(j=i+1; j<regionsTotal-1;j++){
					regions[j][0]=regions[j+1][0];
					regions[j][1]=regions[j+1][1];
				}
				regionsTotal--;			
			}
		}
		else i++;
	}
	
	//Filter for navigable regions
	float angleAux;
	float x1; float y1;float x2; float y2;
	if(regionsTotal>0){
		for(i=0;i<regionsTotal; i++){
			//Case: NOT an artificial region (Goal inside)
			if(gapGoal<regions[i][0] || gapGoal>regions[i][1]){
				angleAux = regions[i][0]*PI/LASER_DIM;
				x1 = laser[regions[i][0]]*cos(angleAux);
				y1 = laser[regions[i][0]]*sin(angleAux);
				
				angleAux = regions[i][1]*PI/LASER_DIM;
				x2 = laser[regions[i][1]]*cos(angleAux);
				y2 = laser[regions[i][1]]*sin(angleAux);
				
				//Case: gap to small for robot, delete region
				if(sqrt(pow(x1-x2,2)+pow(y1-y2,2))<4*radioDoris){
					for(j=i; j<regionsTotal-1;j++){
					regions[j][0]=regions[j+1][0];
					regions[j][1]=regions[j+1][1];
					}
					regionsTotal--;	
				}
			}

		}
		
	}
	
	/************** CHOSE REGION *************************/	
	int sectorDist; int distGoal = 720;
	for(i=0;i<regionsTotal;i++){
		//Gap to the left of region
		sectorDist = SectorDifference(regions[i][0],goal.sector);
		if(sectorDist < distGoal){
			distGoal = sectorDist;
			regNav[0] = regions[i][0];
			regNav[1] = regions[i][1];
			gapNav = regions[i][0];
		}
		//Gap to the right of region
		sectorDist = SectorDifference(regions[i][1],goal.sector);
		if(sectorDist < distGoal){
			distGoal = sectorDist;
			regNav[0] = regions[i][0];
			regNav[1] = regions[i][1];
			gapNav = regions[i][1];
		}
		
	}
	/************** DETERMINE STIUATION *************************/	
	//Goal in region?
	int goalInReg = 0;
	if((goal.sector>=regNav[0])&&(goal.sector<=regNav[1])){
		goalInReg = 1;
	}
	
	//Wide region?
	int wideReg = 0;
	if(abs(regNav[0]-regNav[1])>wideRegLimit){
		wideReg = 1;
	}
	
	//Classify situacion according to both parameters
	if(goalInReg) situacion=SIT_GR;
	else{
		if(wideReg) situacion = SIT_WR;
		else situacion = SIT_NR;
	}
	
	/************** ANGLE ACCORDING TO SITUATION *************************/
	float angle=0; int sentido = 0;
	int sentidoForzado = 0;
	double factor;
	
	switch(situacion){
		case  SIT_GR:
			{
				//Declaraciones
				auxPose obs;
				float beta;
				
				//Obstaculo mas cercano
				obs.dist = distMax;
				for(i=0; i<LASER_DIM;i++){
					if(laser[i]<obs.dist){
						obs.dist = laser[i];
						obs.sector = i;
					}
				}
				obs.angle = pos.angle + (obs.sector*0.5-90)*PI/180;
				obs.angle = LimitAngle(obs.angle);
				
				//Angulo de giro maximo
				angleAux = AngleDifference(PI+obs.angle,goal.angle,&sentido);
				angleAux = LimitAngle(angleAux);
				
				//Factor con Fuzzy Control
				getSystemInput(obs.dist, &factor);
				
				//Calculo de beta
				beta = factor*abs(angleAux);
				
				//Angulo de giro final (absoluto)
				angleAux = AngleDifference(obs.angle,goal.angle,&sentido);
				if(sentido == 1){
					angle = goal.angle + beta;
				}
				else if(sentido == -1){
					angle = goal.angle - beta;
				}
				break;
			}
			
		case SIT_WR:
			{
				//Declaraciones
				auxPose obs; auxPose disc;
				float alfa; float gamma;
				
				//Discontinuidad
				disc.sector = gapNav;
				disc.dist = laser[gapNav];
				disc.angle = pos.angle + (disc.sector*0.5-90)*PI/180;
				disc.angle = LimitAngle(disc.angle);
				
				//Obstaculo mas cercano
				obs.dist = distMax;
				for(i=0; i<LASER_DIM;i++){
					if(laser[i]<obs.dist){
						obs.dist = laser[i];
						obs.sector = i;
					}
				}
				obs.angle = pos.angle + (obs.sector*0.5-90)*PI/180;
				obs.angle = LimitAngle(obs.angle);
				
				//Alfa
				alfa = (radioDoris + distSeg)/disc.dist;
				if(alfa>1) alfa =1;
				alfa=asin(alfa);
				
				//Giro maximo
				angleAux = AngleDifference(goal.angle,disc.angle,&sentido);
				if(sentido == 1){
					angleAux = AngleDifference(PI+obs.angle,disc.angle+alfa,&sentido);
				}
				else if(sentido == -1){
					angleAux = AngleDifference(PI+obs.angle,disc.angle-alfa,&sentido);
				}
				angleAux =  LimitAngle(angleAux);
				
				//Factor con Fuzzy Control
				getSystemInput(obs.dist, &factor);
				
				//Gamma
				gamma = factor*abs(angleAux);
				
				//Sentido de alfa y gamma segun disposicion
				angleAux = AngleDifference(obs.angle,disc.angle,&sentido);
				if (sentido == 1) gamma = gamma;
				else if(sentido == -1) gamma = -gamma;
				
				angleAux = AngleDifference(goal.angle,disc.angle,&sentido);
				if (sentido == 1) alfa = alfa;
				else if(sentido == -1) alfa = -alfa;
				
				//Angulo de giro final (absoluto)
				angle = disc.angle + alfa + gamma;
				
				//Checkeamos sentido forzado si alfa+gamma es mas de 90
				if(abs(alfa+gamma)>PI/2){
					if((alfa+gamma)>=0) sentidoForzado = 1;
					else sentidoForzado = -1;
				}
				
				break;
			}
			
		case SIT_NR:
			{
				//Declaraciones
				auxPose disc1; auxPose disc2;
				auxPose obs1; auxPose obs2;
				float alfa1, alfa2, gamma1, gamma2;
				
				//Discontinuidad a la derecha(1)
				disc1.sector = regNav[0];
				disc1.dist = laser[disc1.sector];
				disc1.angle = pos.angle + (disc1.sector*0.5-90)*PI/180;
				disc1.angle = LimitAngle(disc1.angle);	
				
				//Discontinuidad a la izquierda (2)
				disc2.sector = regNav[1];
				disc2.dist = laser[disc2.sector];
				disc2.angle = pos.angle + (disc2.sector*0.5-90)*PI/180;
				disc2.angle = LimitAngle(disc2.angle);	
				
				//Obstaculo a la derecha (1)
				obs1.dist = distMax;
				for(i=0; i<disc1.sector;i++){
					if(laser[i]<obs1.dist){
						obs1.dist = laser[i];
						obs1.sector = i;
					}
				}
				obs1.angle = pos.angle + (obs1.sector*0.5-90)*PI/180;
				obs1.angle = LimitAngle(obs1.angle);
				
				//Obstaculo a la izquierda (2)
				obs2.dist = distMax;
				for(i=disc2.sector; i<LASER_DIM;i++){
					if(laser[i]<obs2.dist){
						obs2.dist = laser[i];
						obs2.sector = i;
					}
				}
				obs2.angle = pos.angle + (obs2.sector*0.5-90)*PI/180;
				obs2.angle = LimitAngle(obs2.angle);
				
				//Alfa 1
				alfa1 = (radioDoris + distSeg)/disc1.dist;
				if(alfa1>1) alfa1 =1;
				alfa1=asin(alfa1);
				
				//Alfa 2
				alfa2 = (radioDoris + distSeg)/disc2.dist;
				if(alfa2>1) alfa2 =1;
				alfa2=asin(alfa2);
				
				//Gamma 1 (con evaluacion Fuzzy)
				angleAux = PI+obs1.angle-abs(disc1.angle+alfa1);
				angleAux = LimitAngle(angleAux);
				
				getSystemInput(obs1.dist, &factor);
				
				gamma1 = factor*abs(angleAux);
				
				//Gamma 2 (con evaluacion Fuzzy)
				angleAux = PI+obs2.angle-abs(disc2.angle+alfa2);
				angleAux = LimitAngle(angleAux);
				
				getSystemInput(obs2.dist, &factor);
				
				gamma2 = factor*abs(angleAux);
				
				//Angulo final (absoluto)
				if(obs1.dist > obs2.dist){
					//Media restando porque obs peligroso a la izquierda
					angle = AngleMean(disc1.angle, disc2.angle) - (gamma1+gamma2)/2;
				}
				else if(obs2.dist >= obs1.angle){
					angle = AngleMean(disc1.angle, disc2.angle) + (gamma1+gamma2)/2;
				}
				
				break;
			}
		default:
			angle = 0;
			break;
				
	}
	
	//Evaluate the direction
	if(sentidoForzado == 0){
		deltaTheta = AngleDifference(pos.angle, angle, &sentidoGiro);
	}
	else{
		sentidoGiro = sentidoForzado;
		if(sentidoForzado == 1) deltaTheta = AngleDifferenceCCW(pos.angle,angle);
		else if(sentidoForzado == -1) deltaTheta = AngleDifferenceCW(pos.angle,angle);
	}
	
	//Cap angle to avoid oscillations
	if(deltaTheta> PI/6) deltaTheta = PI/6;
	else if(deltaTheta< -PI/6) deltaTheta = -PI/6;
	
	//Provide deltaTheta with the direction of turning
	return deltaTheta * sentidoGiro; //- for CW, + for CCW
	
}



