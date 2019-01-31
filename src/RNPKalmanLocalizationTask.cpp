;

/** VERSIÓN QUE USA SOLO EL LÁSER PARA CORREGIR 
 *  El láser no es capaz de identificar las balizas, por lo que hay que hacer una identificación
 */



#include "RNPKalmanLocalizationTask.h"

const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.22;
const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.014;

//const float RNPKalmanLocalizationTask::MULTIPLIER_FACTOR = 4;

// Valores cambiados a ojo para concordar con cada Iteración
const float centro_odom_dist_sup = 0.0; 
const float centro_odom_dist_inf = 0.0; 
const float RNPKalmanLocalizationTask::incertidumbre_odom_dist_sup = 0.006;  //0.005
const float RNPKalmanLocalizationTask::incertidumbre_odom_dist_inf = 0.010;  //0.010
const float centro_odom_angl_sup = 0.0; 
const float centro_odom_angl_inf = 0.0; 
const float RNPKalmanLocalizationTask::incertidumbre_odom_angl_sup = 0.007; // 0.005
const float RNPKalmanLocalizationTask::incertidumbre_odom_angl_inf = 0.012; // 0.010 

const float centro_laser_dist_sup = 0.0;
const float centro_laser_dist_inf = 0.0;
const float RNPKalmanLocalizationTask::incertidumbre_laser_dist_sup = 0.06; 
const float RNPKalmanLocalizationTask::incertidumbre_laser_dist_inf = 0.13; 
const float centro_laser_angl_sup = 0.0;
const float centro_laser_angl_inf = 0.0;
const float RNPKalmanLocalizationTask::incertidumbre_laser_angl_sup = 0.06; 
const float RNPKalmanLocalizationTask::incertidumbre_laser_angl_inf = 0.15; 
 

const float centro_camera_angl_sup = 0.055;
const float centro_camera_angl_inf = 0.055;
const float RNPKalmanLocalizationTask::incertidumbre_camera_angl_sup = 0.06; 
const float RNPKalmanLocalizationTask::incertidumbre_camera_angl_inf = 0.15; // Las balizas pueden estar mal colocadas

// Chi cuadradi, 2 gfl: 5% = 5.9915; 10% = 4.6052; 15% = 3.7946; 20% = 3.2189; 25% = 2.7726; 30% = 2.4079; 35% = 2.0996; 40% = 1.8326; 50% = 1.3863
// Chi cuadradi, 2 gfl: 5% = 3.8415; 10% = 2.7055; 15% = 2.0722; 20% = 1.6424; 25% = 1.3233; 30% = 1.0742; 35% = 0.8735; 40% = 0.7083; 50% = 0.4549
const float singleMahalanobisLimit = 2.7726; // 2 gdl
const float fullMahalanobisLimit = 5.9915; // 2 gdl
const float camera_mahalanobis_limit = 2.7055; // 1 gdl

const int stateLenght = 3;


int cont_deforme_x = 0;
int cont_deforme_y = 0;
int cont_deforme_th = 0;


RNPKalmanLocalizationTask::RNPKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	test = std::fopen("laser_camera_pkalman.txt","w+");
	test2 = std::fopen("medidas_sensores_pkalman.txt","w+");
	test3 = std::fopen("identificacion.txt","w+");
	this->startThread();
}

RNPKalmanLocalizationTask::~RNPKalmanLocalizationTask(){

}

void RNPKalmanLocalizationTask::init(){
	if(gn != NULL and gn->initializeKalmanVariables() == 0)
	{
		printf("Inicializacion\n");
		/** Posición inicial */
		xk_sup = gn->getRawEncoderPosition();
		xk_inf = gn->getRawEncoderPosition();
		xk_pred_sup = Matrix(3, 1);
		xk_pred_inf = Matrix(3, 1);


		/** Definicioń de matrices */
		Ak = Matrix::eye(3);
		Bk = Matrix(3, 2);
		CG = Matrix(3, 1);
		Pk_sup = Matrix(3, 3);
		Pk_inf = Matrix(3, 3);
		Q_sup = Matrix(2,2);
		Q_inf = Matrix(2,2);

		Q_sup(0,0) = std::pow(incertidumbre_odom_dist_sup, 2) / 3.0;
		Q_sup(1,1) = std::pow(incertidumbre_odom_angl_sup, 2) / 3.0;

		Q_inf(0,0) = std::pow(incertidumbre_odom_dist_inf, 2) / 3.0;
		Q_inf(1,1) = std::pow(incertidumbre_odom_angl_inf, 2) / 3.0;

		
		/** Inicialización de matrices  */
		/* FIABILIDAD INICIAL
	 		P Biel
			- desv tip X: 0.0238
			- desv tip Y 0.0238
			- desv tip Theta 0.102063 (5.73 grados)

			P superior:
			- desv tip X: 0.02
			- desv tip Y: 0.02
			- desv tip Theta: 0.087 (5 grados)

			P infierior:
			- desv tip X: 0.03
			- desv tip Y: 0.03
			- desv tip Theta: 0.122 (7 grados)
		*/
		Pk_sup(0,0) = std::pow(0.15, 2) / 3.0;   Pk_sup(1,1) = std::pow(0.15, 2) / 3.0;   Pk_sup(2,2) = std::pow(10*M_PI/180.0, 2) / 3.0;
		Pk_inf(0,0) = std::pow(0.20, 2) / 3.0;   Pk_inf(1,1) = std::pow(0.20, 2) / 3.0;   Pk_inf(2,2) = std::pow(20*M_PI/180.0, 2) / 3.0;


		// Balizas láser existentes en este sector
		currentSector = gn->getCurrentSector();
		laserLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		cameraLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);

		cont_deforme_x = 0;
		cont_deforme_y = 0;
		cont_deforme_th = 0;

		/** Activar localización */
		enableLocalization = true;

	} 
	else 
	{
		enableLocalization = false;
	}
}


void RNPKalmanLocalizationTask::kill(){
	enableLocalization = false;
	RNRecurrentTask::kill();
}

void RNPKalmanLocalizationTask::task()
{
printf("0.\n");
	if(enableLocalization)
	{
		static int iteracion = 0;
		iteracion ++;


		printf("Iteración: %d\n", iteracion);
		/** Estado anterior **/
		x_sup_1 = xk_sup;
		x_inf_1 = xk_inf;

		P_sup_1 = Pk_sup;
		P_inf_1 = Pk_inf;


		/***** 1º PREDICCIÓN *****/
		bool predicted = true;

		/** Obtener desplazamiento realizado */
		double deltaDistance = 0.0, deltaAngle = 0.0;
		gn->getIncrementPosition(&deltaDistance, &deltaAngle);
printf("1.\n");
		// Si hay movimiento se predice, si no, no
		if(deltaDistance != 0 || deltaAngle != 0) // Hay que hacerlo aquí si no siempre le sumaría el BIAS
		{
			/** Predicción del estado */	
			xk_pred_sup(0,0) = x_sup_1(0,0) + deltaDistance * std::cos(x_sup_1(2,0) + deltaAngle / 2.0) + centro_odom_dist_sup;
			xk_pred_sup(1,0) = x_sup_1(1,0) + deltaDistance * std::sin(x_sup_1(2,0) + deltaAngle / 2.0) + centro_odom_dist_sup;
			xk_pred_sup(2,0) =RNUtils::fixAngleRad( x_sup_1(2,0) + deltaAngle + centro_odom_angl_sup );

			xk_pred_inf(0,0) = x_inf_1(0,0) + deltaDistance * std::cos(x_inf_1(2,0) + deltaAngle / 2.0) + centro_odom_dist_inf;
			xk_pred_inf(1,0) = x_inf_1(1,0) + deltaDistance * std::sin(x_inf_1(2,0) + deltaAngle / 2.0) + centro_odom_dist_inf;
			xk_pred_inf(2,0) =RNUtils::fixAngleRad( x_inf_1(2,0) + deltaAngle + centro_odom_angl_inf );

			/** Calculos para fiabilidad */
			// Centro de gravedad
			CG(2,0) = (2*x_sup_1(2,0)*std::sqrt(3*P_sup_1(2,2)) + 2*x_inf_1(2,0)*std::sqrt(3*P_inf_1(2,2)) + x_sup_1(2,0)*std::sqrt(3*P_inf_1(2,2)) + x_inf_1(2,0)*std::sqrt(3*P_sup_1(2,2)) ) / ( 3*(sqrt(3*P_sup_1(2,2)) + sqrt(3*P_inf_1(2,2))) );
			CG(2,0) =RNUtils::fixAngleRad(CG(2,0));

			// Matriz A 
			// Ak ya ha sido definida como una matriz diagonal
			Ak(0, 2) = -deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);
			Ak(1, 2) = deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);

			// Matriz B 
			Bk(0, 0) = std::cos(CG(2, 0) + deltaAngle/2.0);
			Bk(0, 1) = (-0.5) * deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);
			Bk(1, 0) = std::sin(CG(2, 0) + deltaAngle/2.0);
			Bk(1, 1) = 0.5 * deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);
			Bk(2, 0) = 0.0;
			Bk(2, 1) = 1.0;

			/** Fiabilidad */
			Pk_pred_sup = (Ak * P_sup_1 * ~Ak) + (Bk * Q_sup * ~Bk);
			Pk_pred_inf = (Ak * P_inf_1 * ~Ak) + (Bk * Q_inf * ~Bk);
		}
		else
		{
			xk_pred_sup = x_sup_1;
			xk_pred_inf = x_inf_1;

			Pk_pred_sup = P_sup_1;
			Pk_pred_inf = P_inf_1;

			predicted = false;
		}
printf("2.\n");		



		/****** 2º MEDIDAS PARA CORRECCIÓN ******/
		bool corrected = false;


		/** Balizas detectadas */
		gn->lockLaserLandmarks();
		gn->lockVisualLandmarks();

		int totalLandmarksDetected = 0;
		int laserLandmarksDetected = 0;
		int cameraLandmarksDetected = 0;
		if(gn->isLaserSensorActivated())
		{
			laserLandmarksDetected = gn->getLaserLandmarks()->size();
			totalLandmarksDetected += laserLandmarksDetected;
		}
		if(gn->isCameraSensorActivated())
		{
			cameraLandmarksDetected = gn->getVisualLandmarks()->size();
			totalLandmarksDetected += cameraLandmarksDetected;
		}
		printf("Balizas detectadas: %d, Láser: %d, Camara: %d. \n", totalLandmarksDetected, laserLandmarksDetected, cameraLandmarksDetected);

		int totalLandmarksAccepted = 0;
		int laserLandmarksAccepted = 0;
		int cameraLandmarksAccepted = 0;
	
		/** Si se han detectado balizas se va a corregir, si no no*/
		if(totalLandmarksDetected > 0)
		{
printf("3.\n");			
			/****** 2ºa MATRIZ DE OBSERVACIÓN E INNOVACIÓN PARA TODAS LAS BALIZAS EXISTENTES ******/
			// Número de medidas totales -> Algunos sensores dan más de una medida por baliza
			int num_possibleObservations = 0;
			int num_possibleLandmarks = 0;
			if(gn->isLaserSensorActivated())
			{
				num_possibleObservations += 2*laserLandmarksCount;
				num_possibleLandmarks += laserLandmarksCount;
			}
			if(gn->isCameraSensorActivated())
			{
				num_possibleObservations += cameraLandmarksCount;
				num_possibleLandmarks += cameraLandmarksCount;
			}
	
			if(num_possibleObservations > 0)
			{
				/** Centro de gravedad*/
				CG(0,0) = (2*xk_pred_sup(0,0)*std::sqrt(3*Pk_pred_sup(0,0)) + 2*xk_pred_inf(0,0)*std::sqrt(3*Pk_pred_inf(0,0)) + xk_pred_sup(0,0)*std::sqrt(3*Pk_pred_inf(0,0)) + xk_pred_inf(0,0)*std::sqrt(3*Pk_pred_sup(0,0)) ) / ( 3*(sqrt(3*Pk_pred_sup(0,0)) + sqrt(3*Pk_pred_inf(0,0))) );
				CG(1,0) = (2*xk_pred_sup(1,0)*std::sqrt(3*Pk_pred_sup(1,1)) + 2*xk_pred_inf(1,0)*std::sqrt(3*Pk_pred_inf(1,1)) + xk_pred_sup(1,0)*std::sqrt(3*Pk_pred_inf(1,1)) + xk_pred_inf(1,0)*std::sqrt(3*Pk_pred_sup(1,1)) ) / ( 3*(sqrt(3*Pk_pred_sup(1,1)) + sqrt(3*Pk_pred_inf(1,1))) );
				CG(2,0) = (2*xk_pred_sup(2,0)*std::sqrt(3*Pk_pred_sup(2,2)) + 2*xk_pred_inf(2,0)*std::sqrt(3*Pk_pred_inf(2,2)) + xk_pred_sup(2,0)*std::sqrt(3*Pk_pred_inf(2,2)) + xk_pred_inf(2,0)*std::sqrt(3*Pk_pred_sup(2,2)) ) / ( 3*(sqrt(3*Pk_pred_sup(2,2)) + sqrt(3*Pk_pred_inf(2,2))) );
				CG(2,0) = RNUtils::fixAngleRad(CG(2,0));

				/** Matrices H y innovación completas */
				double distance, angle, d;
				int i_laser = 0, i_camera = 0;
				if(gn->isLaserSensorActivated())
				{
					i_camera = laserLandmarksCount;
				}

				Matrix completeObservations_sup = Matrix(num_possibleObservations,1);
				Matrix completeObservations_inf = Matrix(num_possibleObservations,1);
				Matrix completeHk = Matrix(num_possibleObservations, 3);
				Matrix disp = Matrix(2,1); // Matriz de rotación del sensor

				/** Rellenar matrices completas -> todas las balizas posibles */
				for(int i = 0; i < num_possibleLandmarks; i++)
				{
					// Baliza teórica
					s_landmark* teoricalLandmark = currentSector->landmarkAt(i);

					if(teoricalLandmark->type == XML_SENSOR_TYPE_LASER_STR)
					{
						if(gn->isLaserSensorActivated())
						{
							disp = Matrix(2,1); // Rotación cero

							/** Observaciones */
							landmarkObservation(xk_pred_sup, disp, teoricalLandmark, distance, angle);
							completeObservations_sup(i_laser*2, 0) = distance + centro_laser_dist_sup;
							completeObservations_sup(i_laser*2+1, 0) = RNUtils::fixAngleRad(angle + centro_laser_angl_sup);

							landmarkObservation(xk_pred_inf, disp, teoricalLandmark, distance, angle);
							completeObservations_inf(i_laser*2, 0) = distance + centro_laser_dist_inf;
							completeObservations_inf(i_laser*2+1, 0) = RNUtils::fixAngleRad(angle + centro_laser_angl_inf);

							/** Matriz H */
							d = std::sqrt( std::pow(teoricalLandmark->xpos - (CG(0,0) + disp(0,0)), 2) + std::pow(teoricalLandmark->ypos - (CG(1,0) + disp(1,0)), 2) );
							completeHk(i_laser*2,0) = -(teoricalLandmark->xpos - (CG(0,0) + disp(0,0))) / d;
							completeHk(i_laser*2,1) = -(teoricalLandmark->ypos - (CG(1,0) + disp(1,0))) / d;
							completeHk(i_laser*2,2) = 0.0;
							completeHk(i_laser*2+1,0) = (teoricalLandmark->ypos - (CG(1,0) + disp(1,0))) / std::pow(d,2);
							completeHk(i_laser*2+1,1) = (-1) * (teoricalLandmark->xpos - (CG(0,0) + disp(0,0))) / std::pow(d, 2);
							completeHk(i_laser*2+1,2) = -1.0;	

						i_laser ++;
						}
					}
					else if(teoricalLandmark->type == XML_SENSOR_TYPE_CAMERA_STR)
					{
						if(gn->isCameraSensorActivated())
						{							
							/** Observaciones */
							// Matriz de rotación respecto a la posición superior
							disp = Matrix(2, 1);   double nrx, nry;								
							RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk_pred_sup(2, 0), &nrx, &nry);
							//RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, CG(2, 0), &nrx, &nry);
							disp(0, 0) = nrx;   disp(1, 0) = nry;
							// Medidas superiores
							landmarkObservation(xk_pred_sup, disp, teoricalLandmark, distance, angle);
							completeObservations_sup(i_camera, 0) = RNUtils::fixAngleRad(angle + centro_camera_angl_sup);
							
							// Matriz de rotación respecto a la posición inferior
							disp = Matrix(2, 1);							
							RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, xk_pred_inf(2, 0), &nrx, &nry);
							//RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, CG(2, 0), &nrx, &nry);
							disp(0, 0) = nrx;   disp(1, 0) = nry;
							// Medidas inferiores
							landmarkObservation(xk_pred_inf, disp, teoricalLandmark, distance, angle);
							completeObservations_inf(i_camera, 0) = RNUtils::fixAngleRad(angle + centro_camera_angl_inf);

							/** Matriz H */
							// Matriz de rotación respecto al centro de gravedad
							disp = Matrix(2, 1);							
							RNUtils::rotate(CAMERA_ERROR_POSITION_X, CAMERA_ERROR_POSITION_Y, CG(2, 0), &nrx, &nry);
							disp(0, 0) = nrx;   disp(1, 0) = nry;
							d = std::sqrt( std::pow(teoricalLandmark->xpos - (CG(0,0) + disp(0,0)), 2) + std::pow(teoricalLandmark->ypos - (CG(1,0) + disp(1,0)), 2) );
							completeHk(i_camera,0) = (teoricalLandmark->ypos - (CG(1,0) + disp(1,0))) / std::pow(d,2);
							completeHk(i_camera,1) = (-1) * (teoricalLandmark->xpos - (CG(0,0) + disp(0,0))) / std::pow(d, 2);
							completeHk(i_camera,2) = -1.0;

						i_camera ++;
						}
					}
					else
					{
						printf("CUIDADITO ESTA BALIZA NO TIENE TIPO LÁSER NI CÁMARA\n");
					}
				}// for para rellenar con todas las balizas posibles
				
printf("4.\n");


				
				/****** 2ºb MATRIZ DE OBSERVACIÓN E INNOVACIÓN DE LAS BALIZAS DETECTADAS ******/		

				/** Primero las del láser luego añadiremos las de la cámara */
				Matrix laserRealObservations = Matrix(2,1);
				Matrix laserInnovation_sup = Matrix(2,1);
				Matrix laserInnovation_inf = Matrix(2,1);
				Matrix laserHk = Matrix(2,3);
				Matrix laserR_inf = Matrix(2,2);


				Matrix cameraRealObservations = Matrix(1,1);
				Matrix cameraInnovation_sup = Matrix(1,1);
				Matrix cameraInnovation_inf = Matrix(1,1);
				Matrix cameraHk = Matrix(1,3);
				Matrix cameraR_inf = Matrix(1,1);

				laserR_inf(0,0) = std::pow(incertidumbre_laser_dist_inf, 2) / 3.0;
				laserR_inf(1,1) = std::pow(incertidumbre_laser_angl_inf, 2) / 3.0;
				int i_ref_md = -1;
				double min_mahalanobisDistance = 1000.0;
				vector <int> i_ref_fuzzy;
				double z1_dist, z2_dist, z3_dist, z4_dist, z1_angl, z2_angl, z3_angl, z4_angl;

				laserLandmarksAccepted = 0;
				cameraLandmarksAccepted = 0;

				for(int i = 0; i < laserLandmarksDetected; i++)
				{
					if(laserLandmarksCount != 0)
					{
						RNLandmark* laserLandmark = gn->getLaserLandmarks()->at(i);
						laserRealObservations(0,0) = laserLandmark->getPointsXMean();
						laserRealObservations(1,0) = RNUtils::fixAngleRad(laserLandmark->getPointsYMean());

						/** IDENTIFICACIÓN. Las balizas láser no sabemos a cuál corresponden realmente */
						/** La identificación se va a hacer en función de la mínima distancia de Mahalanobis comparando con la estimación inferior */
						i_ref_md = -1;
						min_mahalanobisDistance = 1000.0;
						i_ref_fuzzy.clear();

						for(int j = 0; j < laserLandmarksCount; j ++)
						{
							laserInnovation_sup(0,0) = laserRealObservations(0,0) - completeObservations_sup(j*2,0);
							laserInnovation_sup(1,0) = RNUtils::fixAngleRad( laserRealObservations(1,0) - completeObservations_sup(j*2+1,0) );

							laserInnovation_inf(0,0) = laserRealObservations(0,0) - completeObservations_inf(j*2,0);
							laserInnovation_inf(1,0) = RNUtils::fixAngleRad( laserRealObservations(1,0) - completeObservations_inf(j*2+1,0) );

							laserHk(0, 0) = completeHk(2*j, 0);
							laserHk(0, 1) = completeHk(2*j, 1);
							laserHk(0, 2) = completeHk(2*j, 2);
							laserHk(1, 0) = completeHk(2*j+1, 0);
							laserHk(1, 1) = completeHk(2*j+1, 1);
							laserHk(1, 2) = completeHk(2*j+1, 2);

							// Mahalanobis
							Matrix omega_inf = laserHk * Pk_pred_inf * ~laserHk + laserR_inf;
							Matrix mdk_inf = ~laserInnovation_inf* !omega_inf * laserInnovation_inf;
							double md_inf = std::sqrt(mdk_inf(0,0));

							if(md_inf < min_mahalanobisDistance)
							{
								min_mahalanobisDistance = md_inf;
								i_ref_md = j;
							}

							Matrix P_Z_sup = laserHk * Pk_pred_sup * ~laserHk;
							Matrix P_Z_inf = laserHk * Pk_pred_inf * ~laserHk;

							/** Identificación borrosa */
							z1_dist = completeObservations_inf(j*2,0) - std::sqrt(3 * P_Z_inf(0,0));
							z2_dist = completeObservations_sup(j*2,0) - std::sqrt(3 * P_Z_sup(0,0));
							z3_dist = completeObservations_sup(j*2,0) + std::sqrt(3 * P_Z_sup(0,0));
							z4_dist = completeObservations_inf(j*2,0) + std::sqrt(3 * P_Z_inf(0,0));

							z1_angl = completeObservations_inf(j*2+1,0) - std::sqrt(3 * P_Z_inf(1,1));
							z2_angl = completeObservations_sup(j*2+1,0) - std::sqrt(3 * P_Z_sup(1,1));
							z3_angl = completeObservations_sup(j*2+1,0) + std::sqrt(3 * P_Z_sup(1,1));
							z4_angl = completeObservations_inf(j*2+1,0) + std::sqrt(3 * P_Z_inf(1,1));

							if(laserRealObservations(0,0) > z1_dist and laserRealObservations(0,0) < z4_dist)
							{
								if(laserRealObservations(1,0) > z1_angl and laserRealObservations(1,0) < z4_angl)
								{
									i_ref_fuzzy.push_back(j); 
								}
							}

						}

						/** Resultados detección borrosa */
						/*if(i_ref_fuzzy.size() > 0)
						{
							printf("La identificación borrosa ha encontrado %d posibles resultados para la baliza %d.\tBalizas propuestas: ", i_ref_fuzzy.size(), i);
							for (int k = 0; k < i_ref_fuzzy.size(); k++)
							{
								printf("%d ", i_ref_fuzzy.at(k));
							}
							printf("\n");
						}*/


						// Baliza identificada
						if( (min_mahalanobisDistance < singleMahalanobisLimit) and (i_ref_md >= 0) )
						{
							printf("Baliza detectada  nº %d identificada como %d\n", i, i_ref_md);

							if(i_ref_fuzzy.size() == 1)
								fprintf(test3, "%d\t%d\n", i_ref_md, i_ref_fuzzy.back());
							else
								fprintf(test3, "%d\t%d\n", i_ref_md, -1);


							laserLandmarksAccepted ++;
							
							// Datos aceptados
							laserHk(0, 0) = completeHk(2*i_ref_md, 0);
							laserHk(0, 1) = completeHk(2*i_ref_md, 1);
							laserHk(0, 2) = completeHk(2*i_ref_md, 2);
							laserHk(1, 0) = completeHk(2*i_ref_md+1, 0);
							laserHk(1, 1) = completeHk(2*i_ref_md+1, 1);
							laserHk(1, 2) = completeHk(2*i_ref_md+1, 2);

							laserInnovation_sup(0,0) = laserRealObservations(0,0) - completeObservations_sup(i_ref_md*2,0);
							laserInnovation_sup(1,0) = RNUtils::fixAngleRad( laserRealObservations(1,0) - completeObservations_sup(i_ref_md*2+1,0) );

							laserInnovation_inf(0,0) = laserRealObservations(0,0) - completeObservations_inf(i_ref_md*2,0);
							laserInnovation_inf(1,0) = RNUtils::fixAngleRad( laserRealObservations(1,0) - completeObservations_inf(i_ref_md*2+1,0) );

							v_Hk.push_back(laserHk);
							v_innovation_sup.push_back(laserInnovation_sup);
							v_innovation_inf.push_back(laserInnovation_inf);
						}
						else
						{
							printf("Baliza descartada por Mahalanobis: %f > %f\n", min_mahalanobisDistance, singleMahalanobisLimit);
						}
					}// for todas las balizas LASER detectadas
				}
				if(cameraLandmarksCount)
				{
					printf("5.\n");				
					/** Ahora añadimos la cámara */
					cameraR_inf(0,0) = std::pow(incertidumbre_camera_angl_inf, 2) / 3.0;
					bool validQR = true;
					cameraLandmarksAccepted = 0;

					if(cameraLandmarksDetected > 1) // UNA SOLA OBSERVACIÓN, NO ME VALE NI PARA CÁLCULOS NI ME FIO DE SU MEDIDA
					{
						for(int i = 0; i < cameraLandmarksDetected; i++)
						{

		printf("5b. %d\n",i);	
							RNLandmark* cameraLandmark = gn->getVisualLandmarks()->at(i);

							validQR = true;
							// Primer filtro -> Que se haya detectado toda la información de la baliza
							if(cameraLandmark->getMapId() > RN_NONE and cameraLandmark->getSectorId() > RN_NONE && cameraLandmark->getMarkerId() > RN_NONE)
							{
								// Segundo filtro -> Compara que los datos no sean los de otra baliza
								for(int j = 0; j < cameraLandmarksDetected and validQR; j++)
								{
									if(j != i)
									{
										if(cameraLandmark->getMarkerId() == gn->getVisualLandmarks()->at(j)->getMarkerId())
										{
											validQR = false;
										}
									}
								}
							}
							else
							{
								validQR = false;
							}

							if(validQR)
							{
								// Tercer filtro -> Si coincide el mapa y el sector
								if( (cameraLandmark->getMapId() == gn->getCurrentSector()->getMapId()) and (cameraLandmark->getSectorId() == gn->getCurrentSector()->getId()) )
								{
									// Buscamos si alguna de las balizas de este sector coincide el QR con el detectado
									bool landmark_used = false;

									for(int j = 2*laserLandmarksCount; j < (2*laserLandmarksCount + cameraLandmarksCount) and !landmark_used; j++)
									{

										if(cameraLandmark->getMarkerId() == gn->getCurrentSector()->landmarkAt(j)->id)
										{
											printf("Baliza detectada. ID: %d compara con la baliza %d con ID: %d\n", cameraLandmark->getMarkerId(), j, gn->getCurrentSector()->landmarkAt(j)->id);
											landmark_used = true;

											cameraRealObservations(0,0) = RNUtils::fixAngleRad(cameraLandmark->getPointsYMean());

											cameraHk(0, 0) = completeHk(j, 0);
											cameraHk(0, 1) = completeHk(j, 1);
											cameraHk(0, 2) = completeHk(j, 2);

											cameraInnovation_sup(0,0) = RNUtils::fixAngleRad( cameraRealObservations(0,0) - completeObservations_sup(j,0) );
											cameraInnovation_inf(0,0) = RNUtils::fixAngleRad( cameraRealObservations(0,0) - completeObservations_inf(j,0) );

		printf("5b. innovacion_sup = %f\n",cameraInnovation_sup(0,0));

											Matrix smallS_inf = cameraHk * Pk_pred_inf * ~cameraHk + cameraR_inf;
											double mahalanobisCamera_inf = std::sqrt( cameraInnovation_inf(0,0) * pow(smallS_inf(0,0), -1) * cameraInnovation_inf(0,0) );

											if(mahalanobisCamera_inf < camera_mahalanobis_limit)
											{
												v_Hk.push_back(cameraHk);
												v_innovation_sup.push_back(cameraInnovation_sup);
												v_innovation_inf.push_back(cameraInnovation_inf);

												cameraLandmarksAccepted ++;
											}
										}
									}
								}
							}
						}// for todas las balizas VISUALES detectadas
					}			
				}

printf("6.\n");
				totalLandmarksAccepted = laserLandmarksAccepted + cameraLandmarksAccepted;
				
				// Escribir información en un fichero
				int cont_landmarks = 0;
				for(int k = 0; k < gn->getLaserLandmarks()->size(); k++)
				{
					RNLandmark* lndmrk = gn->getLaserLandmarks()->at(k);
					fprintf(test2, "%d\t%.10lf\t%.10lf\t", -1, lndmrk->getPointsXMean(), RNUtils::fixAngleRad(lndmrk->getPointsYMean()));
					cont_landmarks ++;
				}
				for(int k = 0; k < gn->getVisualLandmarks()->size(); k++)
				{
					RNLandmark* lndmrk = gn->getVisualLandmarks()->at(k);
					fprintf(test2, "%d\t%.10lf\t%.10lf\t", lndmrk->getMarkerId(), lndmrk->getPointsXMean(), RNUtils::fixAngleRad(lndmrk->getPointsYMean()));
					cont_landmarks ++;
				}
				for(int k = cont_landmarks; k < (laserLandmarksCount + cameraLandmarksCount); k++)
				{
					fprintf(test2, "%d\t%.10lf\t%.10lf\t", 0, 0.0, 0.0);				
				}
				fprintf(test2, "\n");




				/***** 3º CORRECCIÓN *****/
				// Si tenemos información disponible para corregir, pues corregimos
				if(laserLandmarksAccepted > 0 || cameraLandmarksAccepted > 1) // Con 1 sola baliza visual (1 medida de ángulo) no funciona bien
				{
					/** Matrices para la corrección solo con las dimensiones de las balizas aceptadas */
					int sizeMatrix = 2*laserLandmarksAccepted + cameraLandmarksAccepted;
					Matrix Hk = Matrix(sizeMatrix, stateLenght);
					Matrix innovation_sup = Matrix(sizeMatrix, 1);
					Matrix innovation_inf = Matrix(sizeMatrix, 1);
					Matrix R_sup = Matrix(sizeMatrix, sizeMatrix);
					Matrix R_inf = Matrix(sizeMatrix, sizeMatrix);
printf("7.\n");					
					for(int i = 0; i < laserLandmarksAccepted; i++)
					{
						// Leer info guardada
						laserHk = v_Hk.at(i);
						laserInnovation_sup = v_innovation_sup.at(i);
						laserInnovation_inf = v_innovation_inf.at(i);
						
						// Construir matrices finales
						Hk(2*i,0) = laserHk(0,0);
						Hk(2*i,1) = laserHk(0,1);
						Hk(2*i,2) = laserHk(0,2);
						Hk(2*i+1,0) = laserHk(1,0);
						Hk(2*i+1,1) = laserHk(1,1);
						Hk(2*i+1,2) = laserHk(1,2);

						innovation_sup(2*i,0) = laserInnovation_sup(0,0);
						innovation_sup(2*i+1,0) = laserInnovation_sup(1,0);

						innovation_inf(2*i,0) = laserInnovation_inf(0,0);
						innovation_inf(2*i+1,0) = laserInnovation_inf(1,0);

						R_sup(2*i, 2*i) = std::pow(incertidumbre_laser_dist_sup, 2) / 3.0;
						R_sup(2*i+1, 2*i+1) = std::pow(incertidumbre_laser_angl_sup, 2) / 3.0;

						R_inf(2*i, 2*i) = std::pow(incertidumbre_laser_dist_inf, 2) / 3.0;
						R_inf(2*i+1, 2*i+1) = std::pow(incertidumbre_laser_angl_inf, 2) / 3.0;
					}
	
	printf("laser accepted: %d. camera accepted: %d\n",laserLandmarksAccepted, cameraLandmarksAccepted);
					for(int i = 2*laserLandmarksAccepted; i < (2*laserLandmarksAccepted + cameraLandmarksAccepted); i++)
					{					
						cameraHk = v_Hk.at(i);
						cameraInnovation_sup = v_innovation_sup.at(i);
						cameraInnovation_inf = v_innovation_inf.at(i);
						
						// Construir matrices finales
						Hk(i,0) = cameraHk(0,0);
						Hk(i,1) = cameraHk(0,1);
						Hk(i,2) = cameraHk(0,2);

						innovation_sup(i,0) = cameraInnovation_sup(0,0);

						innovation_inf(i,0) = cameraInnovation_inf(0,0);

						R_sup(i, i) = std::pow(incertidumbre_camera_angl_sup, 2) / 3.0;

						R_inf(i, i) = std::pow(incertidumbre_camera_angl_inf, 2) / 3.0;
					}
	

					/** AHORA SÍ CORRECCIÓN DE EKF */
					Matrix Sk_sup = Hk * Pk_pred_sup * ~Hk + R_sup;
					Matrix Sk_inf = Hk * Pk_pred_inf * ~Hk + R_inf;


					/** Último descarte de valores atípicos por mahalanobis */
					Matrix mdk_inf = ~innovation_inf * !Sk_inf * innovation_inf;
					double md = std::sqrt(mdk_inf(0,0));

					if(md < fullMahalanobisLimit)
					{
						// Ganancia de Kalman
						Matrix Wk_sup = Pk_pred_sup * ~Hk * !Sk_sup;
						Matrix Wk_inf = Pk_pred_inf * ~Hk * !Sk_inf;	
						
						// Corrección de la estimación
						xk_sup = xk_pred_sup + Wk_sup * innovation_sup;
						xk_sup(2, 0) = RNUtils::fixAngleRad(xk_sup(2, 0));

						xk_inf = xk_pred_inf + Wk_inf * innovation_inf;
						xk_inf(2, 0) = RNUtils::fixAngleRad(xk_inf(2, 0));

						// Fiabilidad de la estimación corregida
						Pk_sup = Pk_pred_sup - Wk_sup*Sk_sup*~Wk_sup;
						Pk_inf = Pk_pred_inf - Wk_inf*Sk_inf*~Wk_inf;

						corrected = true;		





						if(std::isnan(Pk_sup(0,0)) or (Pk_sup(0,0) == 0.0))
						{
							corrected = false;
							printf("ESTÁN SALIENDO VALORES NAN \n ESTÁN SALIENDO VALORES NAN \n");

							printf("Balizas aceptadas %d. Tamaño matrices: %d\n", totalLandmarksAccepted, sizeMatrix);
							printf("Innovacion: \n");
							innovation_inf.print();
							printf("P pred\n");
							Pk_pred_inf.print();
							printf("Sk: \n");
							Sk_inf.print();
							printf("R\n");
							R_inf.print();
						}
					}
					else
					{
						printf("Corrección COMPLETA descartada por Mahalanobis: %lf > %lf\n", md, singleMahalanobisLimit);
					}
				}
			}
		}

printf("Landmarks accepted: %d. Laser: %d, camera: %d\n",totalLandmarksAccepted, laserLandmarksAccepted, cameraLandmarksAccepted);

printf("8.\n");
		gn->getVisualLandmarks()->clear();
		gn->unlockVisualLandmarks();
		gn->unlockLaserLandmarks();


		/** Si no hemos corregido mantenemos la predicción*/
		if(!corrected)
		{
			xk_sup = xk_pred_sup;
			xk_inf = xk_pred_inf;

			Pk_sup = Pk_pred_sup;
			Pk_inf = Pk_pred_inf;	

			printf("     no se corrige\n");

			for(int k = 0; k < (laserLandmarksCount + cameraLandmarksCount); k++)
			{
				fprintf(test2, "%d\t%.10lf\t%.10lf\t", 0, 0.0, 0.0);				
			}
			fprintf(test2, "\n");
		}
		else
		{
			printf("     SI se corrige\n");
		}


		if(Pk_sup(0,0) < 0 || Pk_sup(1,1) < 0 || Pk_sup(2,2) < 0 || Pk_inf(0,0) < 0 || Pk_inf(1,1) < 0 || Pk_inf(2,2) < 0)
		{
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
			printf("\t\tVALORES DE FIABILIDAD NEGATIVOS \n");
		}



		/** Liberar los vectores */
		v_Hk.clear();
		v_innovation_sup.clear();
		v_innovation_inf.clear();
		
printf("9.\n");
		/** ***** GUARDAR INFO ***** **/
		/** Info estimación */
		char buffer_Pk_pred_sup[256], buffer_Pk_sup[256];
		char buffer_Pk_pred_inf[256], buffer_Pk_inf[256];

		sprintf(buffer_Pk_pred_sup, "%.15lf\t%.15lf\t%.15lf", Pk_pred_sup(0, 0), Pk_pred_sup(1, 1), Pk_pred_sup(2, 2));
		sprintf(buffer_Pk_pred_inf, "%.15lf\t%.15lf\t%.15lf", Pk_pred_inf(0, 0), Pk_pred_inf(1, 1), Pk_pred_inf(2, 2));
		
		sprintf(buffer_Pk_sup, "%.15lf\t%.15lf\t%.15lf", Pk_sup(0, 0), Pk_sup(1, 1), Pk_sup(2, 2));
		sprintf(buffer_Pk_inf, "%.15lf\t%.15lf\t%.15lf", Pk_inf(0, 0), Pk_inf(1, 1), Pk_inf(2, 2));

		char buffer_xk_pred_sup[256], buffer_xk_sup[256];
		char buffer_xk_pred_inf[256], buffer_xk_inf[256];

		sprintf(buffer_xk_pred_sup, "%.15lf\t%.15lf\t%.15lf", xk_pred_sup(0, 0), xk_pred_sup(1, 0), xk_pred_sup(2, 0));
		sprintf(buffer_xk_pred_inf, "%.15lf\t%.15lf\t%.15lf", xk_pred_inf(0, 0), xk_pred_inf(1, 0), xk_pred_inf(2, 0));

		sprintf(buffer_xk_sup, "%.15lf\t%.15lf\t%.15lf", xk_sup(0, 0), xk_sup(1, 0), xk_sup(2, 0));
		sprintf(buffer_xk_inf, "%.15lf\t%.15lf\t%.15lf", xk_inf(0, 0), xk_inf(1, 0), xk_inf(2, 0));

		char buffer[1024];
		sprintf(buffer, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%d\t%d\t%lf\t%lf\n", buffer_xk_pred_sup, buffer_xk_pred_inf, buffer_xk_sup, buffer_xk_inf, buffer_Pk_pred_sup, buffer_Pk_pred_inf, buffer_Pk_sup, buffer_Pk_inf, laserLandmarksAccepted, cameraLandmarksAccepted, deltaDistance, deltaAngle);
		if(test != NULL)
		{
			fprintf(test, "%s", buffer);

		}



		/** ***** PUBLICACIÓN DE UNA POSICIÓN CRISP PARA UN POSIBLE CONTROLADOR ***** **/
		CG(0, 0) = (2 * xk_sup(0, 0) * std::sqrt(3*Pk_sup(0, 0)) + 2 * xk_inf(0, 0) * std::sqrt(3*Pk_inf(0, 0)) + xk_sup(0, 0) * std::sqrt(3*Pk_inf(0, 0)) + xk_inf(0, 0) * std::sqrt(3*Pk_sup(0, 0))) / (3 * (std::sqrt(3*Pk_sup(0, 0)) + std::sqrt(3*Pk_inf(0, 0))));
        CG(1, 0) = (2 * xk_sup(1, 0) * std::sqrt(3*Pk_sup(1, 1)) + 2 * xk_inf(1, 0) * std::sqrt(3*Pk_inf(1, 1)) + xk_sup(1, 0) * std::sqrt(3*Pk_inf(1, 1)) + xk_inf(1, 0) * std::sqrt(3*Pk_sup(1, 1))) / (3 * (std::sqrt(3*Pk_sup(1, 1)) + std::sqrt(3*Pk_inf(1, 1))));
		CG(2, 0) = (2 * xk_sup(2, 0) * std::sqrt(3*Pk_sup(2, 2)) + 2 * xk_inf(2, 0) * std::sqrt(3*Pk_inf(2, 2)) + xk_sup(2, 0) * std::sqrt(3*Pk_inf(2, 2)) + xk_inf(2, 0) * std::sqrt(3*Pk_sup(2, 2))) / (3 * (std::sqrt(3*Pk_sup(2, 2)) + std::sqrt(3*Pk_inf(2, 2))));
		CG(2, 0) = RNUtils::fixAngleRad(CG(2, 0));
				
		gn->setAltPose(ArPose(CG(0, 0), CG(1, 0), CG(2, 0)*180/M_PI));
printf("10.\n");


/*
		// Anterior
		printf("Pos anterior sup: %.8f, %.8f, %.8f\n", x_sup_1(0, 0), x_sup_1(1, 0), x_sup_1(2, 0));
		printf("Pos anterior inf: %.8f, %.8f, %.8f\n", x_inf_1(0, 0), x_inf_1(1, 0), x_inf_1(2, 0));
		printf("Fiabilidad anterior sup: %.8f, %.8f, %.8f\n", P_sup_1(0, 0), P_sup_1(1, 1), P_sup_1(2, 2));
		printf("Fiabilidad anterior inf: %.8f, %.8f, %.8f\n", P_inf_1(0, 0), P_inf_1(1, 1), P_inf_1(2, 2));
		// Predicha
		printf("Pos predicha sup: %.8f, %.8f, %.8f\n", xk_pred_sup(0, 0), xk_pred_sup(1, 0), xk_pred_sup(2, 0));
		printf("Pos predicha inf: %.8f, %.8f, %.8f\n", xk_pred_inf(0, 0), xk_pred_inf(1, 0), xk_pred_inf(2, 0));
		printf("Fiabilidad predicha sup: %.8f, %.8f, %.8f\n", Pk_pred_sup(0, 0), Pk_pred_sup(1, 1), Pk_pred_sup(2, 2));
		printf("Fiabilidad predicha inf: %.8f, %.8f, %.8f\n", Pk_pred_inf(0, 0), Pk_pred_inf(1, 1), Pk_pred_inf(2, 2));
		*/
		// Corregida
		printf("Pos corregida sup: %.8f, %.8f, %.8f\n", xk_sup(0, 0), xk_sup(1, 0), xk_sup(2, 0));
		printf("Pos corregida inf: %.8f, %.8f, %.8f\n", xk_inf(0, 0), xk_inf(1, 0), xk_inf(2, 0));
		printf("Fiabilidad corregida sup: %.8f, %.8f, %.8f\n", Pk_sup(0, 0), Pk_sup(1, 1), Pk_sup(2, 2));
		printf("Fiabilidad corregida inf: %.8f, %.8f, %.8f\n", Pk_inf(0, 0), Pk_inf(1, 1), Pk_inf(2, 2));
		
		// Desfase entre superior e inferior
		//printf("Desfase centros superior e inferior: %.8f, %.8f, %.8f\n", xk_sup(0, 0) - xk_inf(0, 0), xk_sup(1, 0) - xk_inf(1, 0), xk_sup(2, 0) - xk_inf(2, 0));
		


		/** COMPROBAR TRAPECIOS DEFORMES **/
		
		/** Arreglo para intentar que todos los trapecios del ángulo queden en la misma versión de positivo o negativo */
		if(xk_sup(2,0) >= xk_inf(2,0) + M_PI)
			xk_sup(2,0) -= 2*M_PI;
		if(xk_sup(2,0) <= xk_inf(2,0) - M_PI)
			xk_sup(2,0) += 2*M_PI;

		/** Trapecios */
		float x1 = xk_inf(0, 0) - std::sqrt(3*Pk_inf(0, 0));
		float x2 = xk_sup(0, 0) - std::sqrt(3*Pk_sup(0, 0));
		float x3 = xk_sup(0, 0) + std::sqrt(3*Pk_sup(0, 0));
		float x4 = xk_inf(0, 0) + std::sqrt(3*Pk_inf(0, 0));

		float y1 = xk_inf(1, 0) - std::sqrt(3*Pk_inf(1, 1));
		float y2 = xk_sup(1, 0) - std::sqrt(3*Pk_sup(1, 1));
		float y3 = xk_sup(1, 0) + std::sqrt(3*Pk_sup(1, 1));
		float y4 = xk_inf(1, 0) + std::sqrt(3*Pk_inf(1, 1));

		float th1 = xk_inf(2, 0) - std::sqrt(3*Pk_inf(2, 2));
		float th2 = xk_sup(2, 0) - std::sqrt(3*Pk_sup(2, 2));
		float th3 = xk_sup(2, 0) + std::sqrt(3*Pk_sup(2, 2));
		float th4 = xk_inf(2, 0) + std::sqrt(3*Pk_inf(2, 2));

		/*
			printf("trapecio X: %.4f, %.4f, %.4f %.4f\n", x1, x2, x3, x4);
			printf("trapecio Y: %.4f, %.4f, %.4f %.4f\n", y1, y2, y3, y4);
			printf("trapecio Th (grados): %.3f, %.3f, %.3f %.3f\n", th1*180/M_PI, th2*180/M_PI, th3*180/M_PI, th4*180/M_PI);
		*/


		if(corrected or predicted)
		{
			if(x1 > x2 || x3 > x4)
			{
				//printf("    TRAPECIO DEFORME X: %.6f, %.6f, %.6f, %.6f\n", x1, x2, x3, x4);
				cont_deforme_x ++; 
			}
			if(y1 > y2 || y3 > y4)
			{
				//printf("    TRAPECIO DEFORME Y: %.6f, %.6f, %.6f, %.6f\n", y1, y2, y3, y4);
				cont_deforme_y ++;
			}
			if(th1 > th2 || th3 > th4)
			{
				//printf("    TRAPECIO DEFORME TH: %.6f, %.6f, %.6f, %.6f\n", th1, th2, th3, th4);
				cont_deforme_th ++;		
			}
		}

		
		printf("Contadores trapecios deformes:                                                x = %d, y = %d, th = %d\n", cont_deforme_x, cont_deforme_y, cont_deforme_th);

		
		printf("CG: (%.8f, %.8f, %.8f)\n\n\n", CG(0, 0), CG(1, 0), CG(2, 0) * 180.0/M_PI);

	} 
	else 
	{
		init();
	}
	RNUtils::sleep(10);


printf("11.\n");
}


/** Calcula las observaciones de la baliza en base a la posición
* @param & Matriz Xk (Posición del robot en cuestión)
* @param & Matriz disp (Matriz de rotación para relacionar la posición de la cámara con la posición del robot)
* @param estructura baliza (Baliza en cuestión)
* @param & distancia calculada
* @param & ángulo calculado
*/
void RNPKalmanLocalizationTask::landmarkObservation(const Matrix& Xk, const Matrix& disp, s_landmark* landmark, double& distance, double& angle)
{
	distance = std::sqrt(std::pow(landmark->xpos - (Xk(0, 0) + disp(0, 0)), 2) + std::pow(landmark->ypos - (Xk(1, 0) + disp(1, 0)), 2));

	angle = std::atan2(landmark->ypos - (Xk(1, 0) + disp(1, 0)), landmark->xpos - (Xk(0, 0) + disp(0, 0))) - Xk(2, 0);

	angle = RNUtils::fixAngleRad(angle);
}


void RNPKalmanLocalizationTask::OnMessageReceivedWithData(unsigned char* cad, int length){
	gn->lockVisualLandmarks();
	RNLandmarkList* markers = gn->getVisualLandmarks();
	markers->initializeFromString((char*)cad);
	gn->unlockVisualLandmarks();
}