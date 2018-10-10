#include "RNPKalmanLocalizationTask.h"

const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.014;

//const float RNPKalmanLocalizationTask::MULTIPLIER_FACTOR = 4;

const float RNPKalmanLocalizationTask::odom_dist_sup = 0.044;
const float RNPKalmanLocalizationTask::odom_dist_inf = 0.060;
const float RNPKalmanLocalizationTask::odom_angl_sup = 0.066;
const float RNPKalmanLocalizationTask::odom_angl_inf = 0.100;

const float RNPKalmanLocalizationTask::camera_angl_sup = 0.045;
const float RNPKalmanLocalizationTask::camera_angl_inf = 0.085;

const float RNPKalmanLocalizationTask::laser_dist_sup = 0.0175;
const float RNPKalmanLocalizationTask::laser_dist_inf = 0.0250;
const float RNPKalmanLocalizationTask::laser_angl_sup = 0.0200;
const float RNPKalmanLocalizationTask::laser_angl_inf = 0.0475;

RNPKalmanLocalizationTask::RNPKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	test = std::fopen("laser_camera_pkalman.txt","w+");
	this->startThread();
}

RNPKalmanLocalizationTask::~RNPKalmanLocalizationTask(){

}

void RNPKalmanLocalizationTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0){
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
		CG = Matrix(3, 1);
		Pk_sup = gn->getP();
		Pk_inf = gn->getP(); 

		laserLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		cameraLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);

		xk_sup = gn->getRawEncoderPosition();
		xk_inf = gn->getRawEncoderPosition();
		//gn->setAltPose(ArPose(xk(0, 0), xk(1, 0), xk(2, 0) * 180/M_PI));
		//gn->unlockLaserLandmarks();
		//gn->unlockVisualLandmarks();

		int laserIndex = 0, cameraIndex = 0;
		if(gn->isLaserSensorActivated()){
			cameraIndex = 2 * laserLandmarksCount; 
		}
		int sizeR = (gn->isLaserSensorActivated() ? 2 * laserLandmarksCount : 0) + (gn->isCameraSensorActivated() ? cameraLandmarksCount : 0);
        currentR_sup = Matrix(sizeR, sizeR);
        currentR_inf = Matrix(sizeR, sizeR);

		// currentR_sup
		for (int kkk = laserIndex; kkk < cameraIndex; kkk += 2){
			currentR_sup(kkk, kkk) = std::pow(laser_dist_sup, 2) / 3.0;
			currentR_sup(kkk + 1, kkk + 1) = std::pow(laser_angl_sup, 2) / 3.0;

			currentR_inf(kkk, kkk) = std::pow(laser_dist_inf, 2) / 3.0;
			currentR_inf(kkk + 1, kkk + 1) = std::pow(laser_angl_inf, 2) / 3.0;
		}
		for (int kkk = cameraIndex; kkk < sizeR; kkk++){
			currentR_sup(kkk, kkk) = std::pow(camera_angl_sup, 2) / 3.0;

			currentR_inf(kkk, kkk) = std::pow(camera_angl_inf, 2) / 3.0;
		}

		enableLocalization = true;

	} else {
		enableLocalization = false;
	}
}


void RNPKalmanLocalizationTask::kill(){
	enableLocalization = false;
	RNRecurrentTask::kill();
}

void RNPKalmanLocalizationTask::task(){
	if(enableLocalization){
		xk_sup_1 = xk_sup;
		xk_inf_1 = xk_inf;
		
		pk_sup_1 = Pk_sup;
		pk_inf_1 = Pk_inf;
		/** Odometría */
		double deltaDistance = 0.0, deltaAngle = 0.0;
		
		// Obtener desplazamiento realizado
		gn->getIncrementPosition(&deltaDistance, &deltaAngle);

		//printf("pk_sup_1:\n");
		//pk_sup_1.print();
		//printf("pk_inf_1:\n");
		//pk_inf_1.print();

		
		CG(2, 0) = (2 * xk_sup_1(2, 0) * std::sqrt(pk_sup_1(2, 2)) + 2 * xk_inf_1(2, 0) * std::sqrt(pk_inf_1(2, 2)) + xk_sup_1(2, 0) * std::sqrt(pk_inf_1(2, 2)) + xk_inf_1(2, 0) * std::sqrt(pk_sup_1(2, 2))) / (3 * (std::sqrt(pk_sup_1(2, 2)) + sqrt(pk_inf_1(2, 2))));
		CG(2, 0) = RNUtils::fixAngleRad(CG(2, 0));
		//printf("CG:\n");
		//CG.print();
		/** Jacobianos */
		// Jacobiano del movimiento
		Ak(0, 2) = -deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);
		Ak(1, 2) = deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);

		// Jacobiano del ruido de la odometría
		Bk(0, 0) = std::cos(CG(2, 0) + deltaAngle/2.0);
		Bk(0, 1) = -0.5 * deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);

		Bk(1, 0) = std::sin(CG(2, 0) + deltaAngle/2.0);
		Bk(1, 1) = 0.5 * deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);

		Bk(2, 0) = 0.0;
		Bk(2, 1) = 1.0;

		// Actualización de las estimaciones de la posición según la odometría (pos anterior + incremento)
		RNUtils::getOdometryPose(xk_sup, deltaDistance, deltaAngle, xk_sup_1);
		RNUtils::getOdometryPose(xk_inf, deltaDistance, deltaAngle, xk_inf_1);
		
		/** Matriz ruido desplazamiento */	
		Matrix currentQ_sup = Matrix(2, 2);
		Matrix currentQ_inf = Matrix(2, 2);
		if(deltaDistance != 0.0 and deltaAngle != 0.0){
			currentQ_sup(0, 0) = pow(odom_dist_sup, 2) / 3.0;
			currentQ_sup(1, 1) = pow(odom_angl_sup, 2) / 3.0;
			
			currentQ_inf(0, 0) = pow(odom_dist_inf, 2) / 3.0;
			currentQ_inf(1, 1) = pow(odom_angl_inf, 2) / 3.0;
		}	

		/** Fiabilidad de la estimación por odometría */
		Pk_sup = (Ak * pk_sup_1 * ~Ak) + (Bk * currentQ_sup * ~Bk);
		Pk_inf = (Ak * pk_inf_1 * ~Ak) + (Bk * currentQ_inf * ~Bk);
		
		CG(0, 0) = (2 * xk_sup_1(0, 0) * std::sqrt(Pk_sup(0, 0)) + 2 * xk_inf_1(0, 0) * std::sqrt(Pk_inf(0, 0)) + xk_sup_1(0, 0) * std::sqrt(Pk_inf(0, 0)) + xk_inf_1(0, 0) * std::sqrt(Pk_sup(0, 0))) / (3 * (std::sqrt(Pk_sup(0, 0)) + sqrt(Pk_inf(0, 0))));
        CG(1, 0) = (2 * xk_sup_1(1, 0) * std::sqrt(Pk_sup(1, 1)) + 2 * xk_inf_1(1, 0) * std::sqrt(Pk_inf(1, 1)) + xk_sup_1(1, 0) * std::sqrt(Pk_inf(1, 1)) + xk_inf_1(1, 0) * std::sqrt(Pk_sup(1, 1))) / (3 * (std::sqrt(Pk_sup(1, 1)) + sqrt(Pk_inf(1, 1))));
		
		
		/** *** CORRECCIÓN *** */		
		
		int totalLandmarks = 0; //< Número de total de balizas posibles
		
		// Comprobar qué sensores están activos
		if(gn->isLaserSensorActivated()){
			totalLandmarks += laserLandmarksCount;
		}

		if(gn->isCameraSensorActivated()){
			totalLandmarks += cameraLandmarksCount;
		}

		/** Matriz de observación 
		 * Los sensores existentes son cámara y láser. Se puede usar uno de ambos o ambos a la vez.
		 * En caso de usar ambos a la vez la matriz H estára "dividida" en dos secciones: la correspondiente al laser y la correspondiente a la cámara
		 * Así las posiciones en la Matriz H serán adjudicadas por este orden: primero el láser y luego la cámara */
		// El tamaño de la matriz de observación es el número de balizas y el número de variables de estado (x,y,theta)
		int sizeH = (gn->isLaserSensorActivated() ? 2 * laserLandmarksCount : 0) + (gn->isCameraSensorActivated() ? cameraLandmarksCount : 0);
		Hk = Matrix(sizeH, STATE_VARIABLES);
		
		// Variables para establecer las posiciones para el laser y la cámara en H
		int laserIndex = 0, cameraIndex = 0;
		if(gn->isLaserSensorActivated()){
			cameraIndex = 2 * laserLandmarksCount;
		}
		
		// Se recorren todas las balizas existentes en el sector actual comprobando si corresponden con alguna baliza percibida
		for(int i = 0, zIndex = 0; i < gn->getCurrentSector()->landmarksSize(); i++){
			Matrix disp = Matrix(2, 1);//< Matriz de rotación
			double landmarkDistance;
			s_landmark* currLandmark = gn->getCurrentSector()->landmarkAt(i);
			
			// Si el laser está activo
			if(gn->isLaserSensorActivated()){
				// Si la baliza actual es del tipo laser
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){ //< XML_SENSOR..._LASER...= "laser"
					// Índice para alojar 
					zIndex = 2 * laserIndex; //< El láser toma dos medidas distancia y ángulo
					laserIndex++;

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (CG(0, 0) + disp(0, 0)), (CG(1, 0) + disp(1, 0)));

					Hk(zIndex, 0) = -(currLandmark->xpos - (CG(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex, 1) = -(currLandmark->ypos - (CG(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex, 2) = 0.0;

					Hk(zIndex + 1, 0) = (currLandmark->ypos - (CG(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex + 1, 1) = -(currLandmark->xpos - (CG(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex + 1, 2) = -1.0;
				}
			}
			
			// Si la cámara está activa
			if(gn->isCameraSensorActivated()){
				// Si la baliza actual es del tipo camera
				if(gn->getCurrentSector()->landmarkAt(i)->type == XML_SENSOR_TYPE_CAMERA_STR){ //< XML_SENSOR..._CAMERA... = "camera"
					// El índice de la cámara va a empezar después de todas las balizas del láser
					zIndex = cameraIndex;
					cameraIndex++;

					/** NOTA SOBRE EL ROBOT :
					 * La cámara no se encuentra localizada en el centro del robot, es necesario hacer una transformación entre el S.Ref de la cámara y el S.Ref del robot */
					disp(0, 0) = CAMERA_ERROR_POSITION_X * std::cos(CG(2, 0)) - CAMERA_ERROR_POSITION_Y * std::sin(CG(2, 0));
					disp(1, 0) = CAMERA_ERROR_POSITION_X * std::sin(CG(2, 0)) + CAMERA_ERROR_POSITION_Y * std::cos(CG(2, 0));

					landmarkDistance = RNUtils::distanceTo(currLandmark->xpos, currLandmark->ypos, (CG(0, 0) + disp(0, 0)), (CG(1, 0) + disp(1, 0)));

					Hk(zIndex, 0) = (currLandmark->ypos - (CG(1, 0) + disp(1, 0)))/landmarkDistance;
					Hk(zIndex, 1) = -(currLandmark->xpos - (CG(0, 0) + disp(0, 0)))/landmarkDistance;
					Hk(zIndex, 2) = -1.0;
				}
			}
			disp(0, 0) = 0.0; //< Siempre hay que reiniciarlas pues para el láser no hay que transformar S.Ref
			disp(1, 0) = 0.0;
		}
		//printf("Hk:\n");
		//Hk.print();
		/** Predicción de cuál debe ser la medida de los sensores según la posición estimada */
		Matrix zkl_sup;
		Matrix zkl_inf;
		getObservations(zkl_sup, zkl_inf);
		
		
		/** Cálculo del error por la medida */
		//zl = Observaciones reales realizadas - Oservaciones predichas 
		Matrix zl_sup(sizeH, 1);
		Matrix zl_inf(sizeH, 1);

		laserIndex = 0;
		cameraIndex = 0; 

		if(gn->isLaserSensorActivated()){
			cameraIndex = laserLandmarksCount;
		}
		
		int rsize = 0, vsize = 0;
		
		// Si el láser está activo
		if(gn->isLaserSensorActivated()){
			gn->lockLaserLandmarks();
			rsize = gn->getLaserLandmarks()->size();
				
			// Para todas las marcas del laser
            /** Con el láser la baliza no otorga información sobre su ID, hay que averiguar qué baliza es.
             *  El método que se va a utilizar para ello es Mahalanobis. 
             *  Se calcula para cada baliza detectada: (medida_tomada - medida_predicha) usando la baliza encontrada y cada una de las balizas posibles (teóricamente).
             *  La baliza teórica correspondiente a la baliza encontrada será la que produzca menor error (menor distancia de Mahalanobis)
             * 
             *  Ejemplo: Estamos en una sala con 3 balizas (a, b, c) y detectamos 2 (x, y). 
             *  - Se calcula Zkl y su D.Mahalanobis con la información de x y la información que se obtendría de a
             *  - "     "     "     "                                     x                                      b
             *  - "     "     "     "                                     x                                      c
             *  - La baliza x corresponderá a la baliza a, b o c que tenga una menor D.Mahalanobis
             *  - Lo mismo con y
             *  
             * 
             *  NOTAS: EKF Fuzzy tiene dos estimaciones. SÍ se puede dar el caso de que cada una de las estimaciones sup/inf obtenga catalogaciones diferentes. 
             *  ¿Cómo afectaría una catalogación distinta para cada estimación?
             *    + La corrección es específica para cada estimación
             *    - Que la corrección sea específica para cada estimación NO hace que esta sea mejor. De hecho puede mantenerlas separadas
             *    - Si una corrige bien y la otra mal se potencia que el trapecio se deforme
             * 
             *  ¿Cómo afectaría usar para ambos la misma?
             *    + Si acierta las dos estimaciones corrigen bien. Si falla ambas corrigen mal. -> Esto favorece que los trapecios se mantengane estables 
             *    + Respeta el hecho de que una medida es única, la misma para ambas estimaciones
             *   ESTA PARECE LA MÁS ACORDE
             * 
             *  ¿Tiene sentido calcular esto respecto al centro de gravedad en lugar de respecto a las estimaciones?
             *    + Si el centro de gravedad está más ajustado con la posición real -> es beneficioso aplicar esto en el CG pues se va a realizar una corrección mejor
             *    - Habría que obtener la posición predicha respecto del CG
             *    - Para obtener omega (Sk, matriz de fiabilidad de la inovación en la medida), habría que tener un ruido acorde al CG. 
             *   NO SE PUEDE!!!!
             */
			for (int i = 0; i < gn->getLaserLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				std::vector<std::pair<int, float> > euclideanDistances;
				
				/** Matriz de ruido del láser */

				/* NOTAS: Borrar si no se usa, añadido por Víctor
				// Solo se va a utilizar la baliza actual por lo que las dimensiones son 2, 2
				Matrix smallR_sup(2, 2); 
				smallR_sup(0,0) = pow(laser_dist_sup, 2);
				smallR_sup(1,1) = pow(laser_angl_sup, 2);
				*/
				// NOTAS: TIENE QUE RECIBIR LA SUPERIOR 
				Matrix smallR_sup(2, 2);
				smallR_sup(0,0) = pow(laser_dist_sup, 2) / 3.0;
                smallR_sup(1,1) = pow(laser_angl_sup, 2) / 3.0;


				/** Medidas tomadas para esta baliza */
				Matrix measure(2, 1);
				// Distancia
				measure(0, 0) = lndmrk->getPointsXMean();
				// Ángulo
				measure(1, 0) = lndmrk->getPointsYMean();
				
				// Para los índices relativos al laser
				for (int j = laserIndex; j < cameraIndex; j++){
					// Matriz de observación relativa al láser
					Matrix smallHk(2, 3);
					smallHk(0, 0) = Hk(2 * j, 0);
					smallHk(0, 1) = Hk(2 * j, 1);
					smallHk(0, 2) = Hk(2 * j, 2);
					smallHk(1, 0) = Hk(2 * j + 1, 0);
					smallHk(1, 1) = Hk(2 * j + 1, 1);
					smallHk(1, 2) = Hk(2 * j + 1, 2);
					
					// Fiabilidad de medida_tomada - medida_estimada
					Matrix omega = smallHk * Pk_sup * ~smallHk + smallR_sup;
					
					// Medida predicha según la posición estimada por odometría
					Matrix obs(2, 1);
					obs(0, 0) = zkl_sup(j, 0);
					obs(1, 0) = zkl_sup(j, 1);
					
					// Distancia de Mahalanobis
					Matrix mdk = ~(measure - obs) * omega * (measure - obs);
					float md = std::sqrt(mdk(0, 0));
					euclideanDistances.push_back(std::pair<int, float>(j, md));
					
					//RNUtils::printLn("Euclidean Distance[%d]: %f", j, euclideanDistances.at(j));	
				}

				/** Descarte por Mahalanobis. Se selecciona la más pequeña */
				float minorDistance = std::numeric_limits<float>::infinity();
				int indexFound = RN_NONE;
				for (int j = 0; j < euclideanDistances.size(); j++){
					if(euclideanDistances.at(j).second < minorDistance){
						minorDistance = euclideanDistances.at(j).second;
						indexFound = euclideanDistances.at(j).first;
					}	
				}
				
				//RNUtils::printLn("Matched landmark: {idx : %d, MHD: %f}", indexFound, minorDistance);
				
				/** Medida realizada - medida estimada_(sup/inf) */
				// Si se ha identificado una baliza
				if(indexFound > RN_NONE){
					// Distancia sup e inf
					zl_sup(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl_sup(indexFound, 0);
					zl_inf(2 * indexFound, 0) = lndmrk->getPointsXMean() - zkl_inf(indexFound, 0);
					
					// Ángulo sup
					zl_sup(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl_sup(indexFound, 1);
					if(zl_sup(2 * indexFound + 1, 0) > M_PI){
						zl_sup(2 * indexFound + 1, 0) = zl_sup(2 * indexFound + 1, 0) - 2 * M_PI;
					} else if(zl_sup(2 * indexFound + 1, 0) < -M_PI){
						zl_sup(2 * indexFound + 1, 0) = zl_sup(2 * indexFound + 1, 0) + 2 * M_PI;
					}
					
					// Ángulo inf
					zl_inf(2 * indexFound + 1, 0) = lndmrk->getPointsYMean() - zkl_inf(indexFound, 1);
					if(zl_inf(2 * indexFound + 1, 0) > M_PI){
						zl_inf(2 * indexFound + 1, 0) = zl_inf(2 * indexFound + 1, 0) - 2 * M_PI;
					} else if(zl_inf(2 * indexFound + 1, 0) < -M_PI){
						zl_inf(2 * indexFound + 1, 0) = zl_inf(2 * indexFound + 1, 0) + 2 * M_PI;
					}
					
					//RNUtils::printLn("markerId: %d, Estimación: {d: %f, a: %f}, BD: {d: %f, a: %f}, Error: {d: %f, a: %f}", indexFound, lndmrk->getPointsXMean() , lndmrk->getPointsYMean() , zkl(indexFound , 0), zkl(indexFound , 1), zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));





					/** NOTAS: Esto NO es mahalanobis. Es simplemente la inovación de la distancia o o o o o o o  del ángulo entre su varianza. NO juzga ambas a la vez
					 * - corrección distancia / varianza distancia
					 * - corrección ángulo / varianza ángulo
					 * 
					 * Simplemente podría aplicar que si la inovación es mayor que un rango a fregar
					 * Si el error / varianción típica láser 
					 * 
                    double mdDistance = std::abs(zl(2 * indexFound, 0) / std::sqrt(gn->getLaserDistanceVariance()));
					double mdAngle = std::abs(zl(2 * indexFound + 1, 0) / std::sqrt(gn->getLaserAngleVariance()));
					//printf("Mhd: (%d), %lg, %lg, nud: %lg, nua: %lg\n", i, mdDistance, mdAngle, zl(2 * indexFound, 0), zl(2 * indexFound + 1, 0));
					if (mdDistance > laserTMDistance or mdAngle > laserTMAngle)
					{
						zl(2 * indexFound, 0) = 0.0;
						zl(2 * indexFound + 1, 0) = 0.0;
						rsize--; //< Esto solo me indica cuantas balizas he usado
					}		
					*/	
				
				}
			}
			gn->unlockLaserLandmarks();
		}

		// Si la cámara está activa
		if(gn->isCameraSensorActivated()){
			gn->lockVisualLandmarks();
			int laserOffset = gn->isLaserSensorActivated() ? laserLandmarksCount : 0;

			vsize = gn->getVisualLandmarks()->size();
			
			// Para cada una de las balizas de la cámara
			for (int i = 0; i < gn->getVisualLandmarks()->size(); i++){
				RNLandmark* lndmrk = gn->getVisualLandmarks()->at(i);
				// Variable que indica si una baliza es aceptada o no
				bool validQR = true;

				/** Si la información de la baliza es lógica es aceptada */
				if (lndmrk->getMapId() > RN_NONE && lndmrk->getSectorId() > RN_NONE && lndmrk->getMarkerId() > RN_NONE){
					//Compares the current landmark with the remaining landmarks
					for (int j = 0; j < gn->getVisualLandmarks()->size() and (validQR); j++){
						if (j != i){ //< Que no se compare con ella misma
							/** Si la id de la baliza NO es única es rechazada por el momento, posteriormente se la "juzgará" con Mahalanobis*/
							if (lndmrk->getMarkerId() == gn->getVisualLandmarks()->at(j)->getMarkerId()){
								validQR = false;
								//RNUtils::printLn("Current landmark doesn't have a unique ID. Applying Mahalanobis distance...");
							}
						}
					}
				} else {
					validQR = false;
				}

				/** If the current landmark has a unique id, checks if there's a match with the database*/
				if (validQR){
					// Para cada baliza de la cámara. zkl_sup y zkl_inf tienen las mismas dimensiones
					for (int j = cameraIndex; j < zkl_sup.rows_size(); j++){
						// Si la baliza actual corresponde con una existente, comparando IDs
						if((lndmrk->getMapId() == gn->getCurrentSector()->getMapId()) and (lndmrk->getSectorId() == gn->getCurrentSector()->getId()) and (lndmrk->getMarkerId() == ((int)zkl_sup(j, 3)))){
							/** Medidas tomadas para esta baliza */
							double distanceFixed = lndmrk->getPointsXMean();
							double angleFixed = lndmrk->getPointsYMean();
							
							/** Medida realizada - medida estimada_(sup/inf) */
							zl_sup(j + laserOffset, 0) = angleFixed - zkl_sup(j, 1);
							if(zl_sup(j + laserOffset, 0) > M_PI){
								zl_sup(j + laserOffset, 0) = zl_sup(j + laserOffset, 0) - 2 * M_PI;
							} else if(zl_sup(j + laserOffset, 0) < -M_PI){
								zl_sup(j + laserOffset, 0) = zl_sup(j + laserOffset, 0) + 2 * M_PI;
							}
							
							zl_inf(j + laserOffset, 0) = angleFixed - zkl_inf(j, 1);
							if(zl_inf(j + laserOffset, 0) > M_PI){
								zl_inf(j + laserOffset, 0) = zl_inf(j + laserOffset, 0) - 2 * M_PI;
							} else if(zl_inf(j + laserOffset, 0) < -M_PI){
								zl_inf(j + laserOffset, 0) = zl_inf(j + laserOffset, 0) + 2 * M_PI;
							}
						}
					}
				}
			}
			
		
			gn->getVisualLandmarks()->clear();
			gn->unlockVisualLandmarks();
		}
		
		/** Matrices S y W */
		Matrix Sk_sup = Hk * Pk_sup * ~Hk + currentR_sup;
		Matrix Sk_inf = Hk * Pk_inf * ~Hk + currentR_inf;
		
		Matrix Wk_sup = Pk_sup * ~Hk * !Sk_sup;
		Matrix Wk_inf = Pk_inf * ~Hk * !Sk_inf;

		Wk_sup = fixFilterGain(Wk_sup);
		Wk_inf = fixFilterGain(Wk_inf);


		/** Corrección y fiabilidad */
		char bufferpk1_sup[256], bufferpk_sup[256];
		char bufferpk1_inf[256], bufferpk_inf[256];

		sprintf(bufferpk1_sup, "%.4e\t%.4e\t%.4e", Pk_sup(0, 0), Pk_sup(1, 1), Pk_sup(2, 2));
		sprintf(bufferpk1_inf, "%.4e\t%.4e\t%.4e", Pk_inf(0, 0), Pk_inf(1, 1), Pk_inf(2, 2));

		Pk_sup = (Matrix::eye(3) - Wk_sup * Hk) * Pk_sup;
		Pk_inf = (Matrix::eye(3) - Wk_inf * Hk) * Pk_inf;
		sprintf(bufferpk_sup, "%.4e\t%.4e\t%.4e", Pk_sup(0, 0), Pk_sup(1, 1), Pk_sup(2, 2));
		sprintf(bufferpk_inf, "%.4e\t%.4e\t%.4e", Pk_inf(0, 0), Pk_inf(1, 1), Pk_inf(2, 2));

		char bufferxk1_sup[256], bufferxk_sup[256];
		char bufferxk1_inf[256], bufferxk_inf[256];

		sprintf(bufferxk1_sup, "%.4lf\t%.4lf\t%.4lf", xk_sup_1(0, 0), xk_sup_1(1, 0), xk_sup_1(2, 0));
		sprintf(bufferxk1_inf, "%.4lf\t%.4lf\t%.4lf", xk_inf_1(0, 0), xk_inf_1(1, 0), xk_inf_1(2, 0));

		xk_sup = xk_sup_1 + Wk_sup * zl_sup;
		xk_sup(2, 0) = RNUtils::fixAngleRad(xk_sup(2, 0));
		xk_inf = xk_inf_1 + Wk_inf * zl_inf;
		xk_inf(2, 0) = RNUtils::fixAngleRad(xk_inf(2, 0));

		sprintf(bufferxk_sup, "%.4lf\t%.4lf\t%.4lf", xk_sup(0, 0), xk_sup(1, 0), xk_sup(2, 0));
		sprintf(bufferxk_inf, "%.4lf\t%.4lf\t%.4lf", xk_inf(0, 0), xk_inf(1, 0), xk_inf(2, 0));

		char buffer[1024];
		sprintf(buffer, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%d\t%d\n", bufferxk1_sup, bufferxk1_inf, bufferxk_sup, bufferxk_inf, bufferpk1_sup, bufferpk1_inf, bufferpk_sup, bufferpk_inf, rsize, vsize);
		if(test != NULL){
			fprintf(test, "%s", buffer);
		}

	} else {
		init();
	}
	RNUtils::sleep(10);
}

/** Descartar valores muy bajos ya que ralentizan la convergencia 
 * @param Matriz W (ganancia de Kalman)
 * @param Matriz (Medida_tomada - Medida_predicha) 
 * */
Matrix RNPKalmanLocalizationTask::fixFilterGain(const Matrix wk){
	Matrix result = wk;
	for(int i = 0; i < wk.rows_size(); i++){
		for(int j = 0; j < wk.cols_size(); j++){
			if(wk(i, j) >= -1.0e-5 and wk(i, j) <= 1.0e-5){
				result(i, j) = 0.0;
			}
		}
	}
	return result;
}


/** Predecir las observaciones.
 * 1º Se obtienen los datos de todas las varibles del sector
 * 2º Se predice dónde van a estar esas balizas
 * Sobreescribe las 4 columnas: (0=distancia, 1=angulo, 2=zpos, 3=id)
 * @param observations_sup Puntero a una matriz vacia que va a almacenar las predicciones de las observaciones superiores
 * @param observations_inf Puntero a una matriz vacia que va a almacenar las predicciones de las observaciones inferiores
*/
void RNPKalmanLocalizationTask::getObservations(Matrix& observations_sup, Matrix& observations_inf)
{
	int totalLandmarks = 0;
	int laserIndex = 0, cameraIndex = 0;
	
	// Balizas existentes totales 
	if(gn->isLaserSensorActivated()){
		totalLandmarks += laserLandmarksCount;
		cameraIndex += laserLandmarksCount;
	}
	if(gn->isCameraSensorActivated()){
		totalLandmarks += cameraLandmarksCount;
	}

	/** Matrices (sup, inf) que van a almacenar todas las posibles predicciones de medidas*/
	Matrix result_sup(totalLandmarks, 4);
	Matrix result_inf(totalLandmarks, 4);

	// Información de todas las balizas del sector actual
	for(int k = 0; k < gn->getCurrentSector()->landmarksSize(); k++){
		int zIndex = RN_NONE; //< RN_NONE = -1
		double distance_sup = 0, angle_sup = 0, distance_inf = 0, angle_inf = 0;
		Matrix disp = Matrix(2, 1); //< Matriz transformación S.Ref cámara - S.Ref robot
		
		// Baliza siendo actualmente tratada
		s_landmark* landmark = gn->getCurrentSector()->landmarkAt(k);
		
		// Si la baliza es del tipo cámara se hace la transformación S.Ref cámara - S.Ref robot 
		if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
			disp(0, 0) = CAMERA_ERROR_POSITION_X * std::cos(CG(2, 0)) - CAMERA_ERROR_POSITION_Y * std::sin(CG(2, 0));
			disp(1, 0) = CAMERA_ERROR_POSITION_X * std::sin(CG(2, 0)) + CAMERA_ERROR_POSITION_Y * std::cos(CG(2, 0));
		}
		
		/** Predicción de las medidas para esta baliza 
		 * Devuelve la distancia y el ángulo predicho*/
		landmarkObservation(xk_sup_1, disp, landmark, distance_sup, angle_sup);
		landmarkObservation(xk_inf_1, disp, landmark, distance_inf, angle_inf);

		// Índices para respetar el orden 1º laser, 2º cámara
		if(gn->isLaserSensorActivated()){
			if(landmark->type == XML_SENSOR_TYPE_LASER_STR)
			{
				zIndex = laserIndex;
				laserIndex++;
			}
		}
		if(gn->isCameraSensorActivated()){
			if(landmark->type == XML_SENSOR_TYPE_CAMERA_STR){
				zIndex = cameraIndex;
				cameraIndex++;
			}
		}
		
		// Si hay balizas 
		if(zIndex > RN_NONE){
			result_sup(zIndex, 0) = distance_sup;
			result_inf(zIndex, 0) = distance_inf;
			
			if(angle_sup > M_PI){
				angle_sup = angle_sup - 2 * M_PI;
			} else if(angle_sup < -M_PI){
				angle_sup = angle_sup + 2 * M_PI;
			}
			result_sup(zIndex, 1) = angle_sup;
			
			if(angle_inf > M_PI){
				angle_inf = angle_inf - 2 * M_PI;
			} else if(angle_inf < -M_PI){
				angle_inf = angle_inf + 2 * M_PI;
			}
			result_inf(zIndex, 1) = angle_inf;
			
			result_sup(zIndex, 2) = landmark->zpos;
			result_inf(zIndex, 2) = landmark->zpos;
			
			result_sup(zIndex, 3) = (double)landmark->id;
			result_inf(zIndex, 3) = (double)landmark->id;
		}
	}
	observations_sup = result_sup;
	observations_inf = result_inf;
}

void RNPKalmanLocalizationTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle){
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));
	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);
}

void RNPKalmanLocalizationTask::OnMessageReceivedWithData(unsigned char* cad, int length){
	gn->lockVisualLandmarks();
	RNLandmarkList* markers = gn->getVisualLandmarks();
	markers->initializeFromString((char*)cad);
	gn->unlockVisualLandmarks();
}