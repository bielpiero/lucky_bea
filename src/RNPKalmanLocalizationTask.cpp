

/** VERSIÓN QUE USA SOLO EL LÁSER PARA CORREGIR 
 *  El láser no es capaz de identificar las balizas, por lo que hay que hacer una identificación
 */



#include "RNPKalmanLocalizationTask.h"

const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_X = -0.25;
const double RNPKalmanLocalizationTask::CAMERA_ERROR_POSITION_Y = -0.014;

//const float RNPKalmanLocalizationTask::MULTIPLIER_FACTOR = 4;

const float RNPKalmanLocalizationTask::incertidumbre_odom_dist_sup = 0.045; 
const float RNPKalmanLocalizationTask::incertidumbre_odom_dist_inf = 0.075;
const float RNPKalmanLocalizationTask::incertidumbre_odom_angl_sup = 0.065; 
const float RNPKalmanLocalizationTask::incertidumbre_odom_angl_inf = 0.115;

const float RNPKalmanLocalizationTask::incertidumbre_laser_dist_sup = 0.017; 
const float RNPKalmanLocalizationTask::incertidumbre_laser_dist_inf = 0.040;
const float RNPKalmanLocalizationTask::incertidumbre_laser_angl_sup = 0.025; 
const float RNPKalmanLocalizationTask::incertidumbre_laser_angl_inf = 0.065;

const float mahalanobis_limit = 5.9915;

RNPKalmanLocalizationTask::RNPKalmanLocalizationTask(const GeneralController* gn, const char* name, const char* description) : RNLocalizationTask(gn, name, description), UDPServer(22500){
	enableLocalization = false;
	test = std::fopen("laser_camera_pkalman.txt","w+");
	test2 = std::fopen("medidas_sensores_pkalman.txt","w+");
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
		Pk_sup(0,0) = std::pow(0.10, 2) / 3.0;   Pk_sup(1,1) = std::pow(0.10, 2) / 3.0;   Pk_sup(2,2) = std::pow(2*M_PI/180.0, 2) / 3.0;
		Pk_inf(0,0) = std::pow(0.15, 2) / 3.0;   Pk_inf(1,1) = std::pow(0.15, 2) / 3.0;   Pk_inf(2,2) = std::pow(4*M_PI/180.0, 2) / 3.0;

		// Balizas láser existentes en este sector
		laserLandmarksCount = gn->getCurrentSector()->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);

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

void RNPKalmanLocalizationTask::task(){
	if(enableLocalization)
	{
		printf("1.\n");
		/** Guardar valores de la última estimación */
		x_sup_1 = xk_sup;
		x_inf_1 = xk_inf;
		
		P_sup_1 = Pk_sup;
		P_inf_1 = Pk_inf;


		/** Odometría */
		double deltaDistance = 0.0, deltaAngle = 0.0;
		
		// Obtener desplazamiento realizado
		gn->getIncrementPosition(&deltaDistance, &deltaAngle);

		/** ***** PREDICCIÓN ***** **/	
		printf("Movimiento lineal: %.4f, angular %.4f\n", deltaDistance, deltaAngle);
		// Actualización de las estimaciones de la posición según la odometría (pos anterior + incremento)
		RNUtils::getOdometryPose(x_sup_1, deltaDistance, deltaAngle, xk_pred_sup);
		RNUtils::getOdometryPose(x_inf_1, deltaDistance, deltaAngle, xk_pred_inf);
		xk_pred_sup(2,0) = RNUtils::fixAngleRad(xk_pred_sup(2,0));
		xk_pred_inf(2,0) = RNUtils::fixAngleRad(xk_pred_inf(2,0));
		/* Lo que hace RNUtils
			xk1 = xk + deltaDistance * std::cos(thk + (deltaDegrees / 2.0));
    		yk1 = yk + deltaDistance * std::sin(thk + (deltaDegrees / 2.0));
    		thk1 = thk + deltaDegrees */
				
		/** Centro gravedad del ángulo */
		CG(2, 0) = (2 * x_sup_1(2, 0) * std::sqrt(3*P_sup_1(2, 2)) + 2 * x_inf_1(2, 0) * std::sqrt(3*P_inf_1(2, 2)) + x_sup_1(2, 0) * std::sqrt(3*P_inf_1(2, 2)) + x_inf_1(2, 0) * std::sqrt(3*P_sup_1(2, 2))) / (3 * (std::sqrt(3*P_sup_1(2, 2)) + sqrt(3*P_inf_1(2, 2))));
		CG(2, 0) = RNUtils::fixAngleRad(CG(2, 0));
			
		/** Matrices jacobianas */
		// Ak ya está inicializada como una matriz I -> Solo hay que cambiar dos valores
		Ak(0, 2) = -deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);
		Ak(1, 2) = deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);

		Bk(0, 0) = std::cos(CG(2, 0) + deltaAngle/2.0);
		Bk(0, 1) = -0.5 * deltaDistance * std::sin(CG(2, 0) + deltaAngle/2.0);
		Bk(1, 0) = std::sin(CG(2, 0) + deltaAngle/2.0);
		Bk(1, 1) = 0.5 * deltaDistance * std::cos(CG(2, 0) + deltaAngle/2.0);
		Bk(2, 0) = 0.0;
		Bk(2, 1) = 1.0;

		/** Matriz ruido desplazamiento */	
		if(deltaDistance == 0.0 and deltaAngle == 0.0) //< Si hay movimiento predecimos
		{
			Q_sup(0, 0) = 0.0;
			Q_sup(1, 1) = 0.0;

			Q_inf(0, 0) = 0.0;
			Q_inf(1, 1) = 0.0;
		}
		else
		{
			Q_sup(0, 0) = std::pow(incertidumbre_odom_dist_sup, 2) / 3.0;
			Q_sup(1, 1) = std::pow(incertidumbre_odom_angl_sup, 2) / 3.0;
		
			Q_inf(0, 0) = std::pow(incertidumbre_odom_dist_inf, 2) / 3.0;
			Q_inf(1, 1) = std::pow(incertidumbre_odom_angl_inf, 2) / 3.0;
		}	

		/** Fiabilidad de la estimación por odometría */
		Pk_pred_sup = (Ak * P_sup_1 * ~Ak) + (Bk * Q_sup * ~Bk);
		Pk_pred_inf = (Ak * P_inf_1 * ~Ak) + (Bk * Q_inf * ~Bk);

		


		printf("2.\n");

		/** ***** CORRECCIÓN ***** **/
		/* En esta versión solo hay corrección de la cámara */

		// Variables 
		int vsize = 0; //< contador de balizas encontradas
		int vsize_used = 0; //< contador usadas
		Matrix disp = Matrix(2, 1); //< Matriz de rotación
		bool landmark_used = false; //< Flag para detectar si hemos usado la baliza	

		Matrix real_observations = Matrix(2,1);
		Matrix observations_sup = Matrix(2*laserLandmarksCount,1);
		Matrix observations_inf = Matrix(2*laserLandmarksCount,1);
		double distance = 0.0, angle = 0.0;

		Matrix single_inno_sup = Matrix(2,1);
		Matrix single_inno_inf = Matrix(2,1);
		
		Matrix fullHk = Matrix(2*laserLandmarksCount, 3); 
		Matrix singleHk = Matrix(2,3); // 2 -> distancia y ángulo
		double landmarkDistance = 0.0; //< Es el denominador de la matriz H (corresponde a la distancia)

		Matrix smallR_inf = Matrix(2,2);
		smallR_inf(0,0) = std::pow(incertidumbre_laser_dist_inf, 2) / 3.0;
		smallR_inf(1,1) = std::pow(incertidumbre_laser_angl_inf, 2) / 3.0;

		double mahalanobis_distance = 1000.0;
		int i_mahalanobis = -1;
		Matrix smallObservations_inf(2, 1);



		/** Centro de Gravedad*/
		CG(0, 0) = (2 * xk_pred_sup(0, 0) * std::sqrt(3*Pk_pred_sup(0, 0)) + 2 * xk_pred_inf(0, 0) * std::sqrt(3*Pk_pred_inf(0, 0)) + xk_pred_sup(0, 0) * std::sqrt(3*Pk_pred_inf(0, 0)) + xk_pred_inf(0, 0) * std::sqrt(3*Pk_pred_sup(0, 0))) / (3 * (std::sqrt(3*Pk_pred_sup(0, 0)) + sqrt(3*Pk_pred_inf(0, 0))));
        CG(1, 0) = (2 * xk_pred_sup(1, 0) * std::sqrt(3*Pk_pred_sup(1, 1)) + 2 * xk_pred_inf(1, 0) * std::sqrt(3*Pk_pred_inf(1, 1)) + xk_pred_sup(1, 0) * std::sqrt(3*Pk_pred_inf(1, 1)) + xk_pred_inf(1, 0) * std::sqrt(3*Pk_pred_sup(1, 1))) / (3 * (std::sqrt(3*Pk_pred_sup(1, 1)) + sqrt(3*Pk_pred_inf(1, 1))));
		

        /** PROCESO EN EL QUE SE OBTIENE LA INFORMACIÓN DE LA MATRIZ H Y LAS MATRICES DE INNOVACIÓN */
		if(gn->isLaserSensorActivated())
		{
			/** Balizas detectadas */
			gn->lockLaserLandmarks(); // Bloqueamos detección

			// Balizas detectadas
			vsize = gn->getLaserLandmarks()->size();


			/** Cálculo de todas las observaciones hechas por la estimación superior e inferior
				Cálculo de H completo */
			for(int i = 0; i < laserLandmarksCount; i++)
			{
				s_landmark* landmark = gn -> getCurrentSector() -> landmarkAt(i);

				// Observaciones superiores
				landmarkObservation(xk_pred_sup, disp, landmark, distance, angle);
				angle = RNUtils::fixAngleRad(angle);
				observations_sup(i*2, 0) = distance;
				observations_sup(i*2 + 1, 0) = angle;

				// Observaciones inferiores
				landmarkObservation(xk_pred_inf, disp, landmark, distance, angle);
				angle = RNUtils::fixAngleRad(angle);
				observations_inf(i*2, 0) = distance;
				observations_inf(i*2 + 1, 0) = angle;

				// Matriz H
				landmarkDistance = RNUtils::distanceTo(landmark->xpos, landmark->ypos, (CG(0, 0) + disp(0, 0)), (CG(1, 0) + disp(1, 0)));
				
				fullHk(i*2, 0) = -(landmark->xpos - (CG(0, 0) + disp(0, 0)))/landmarkDistance;
				fullHk(i*2, 1) = -(landmark->ypos - (CG(1, 0) + disp(1, 0)))/landmarkDistance;
				fullHk(i*2, 2) = 0.0;

				fullHk(i*2+1, 0) = (landmark->ypos - (CG(1, 0) + disp(1, 0)))/std::pow(landmarkDistance, 2);
				fullHk(i*2+1, 1) = -(landmark->xpos - (CG(0, 0) + disp(0, 0)))/std::pow(landmarkDistance, 2);
				fullHk(i*2+1, 2) = -1.0;
			}

			/** Obtención de las observaciones tomadas
				Cálculo de la innovación 
				Cálculo de la matriz H final (solo tiene las balizas detectadas) */
			for(int i = 0; i < vsize; i++)
			{

				// Puntero apuntando a la baliza actual
				RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

				// Medidas de la baliza detectada
				real_observations(0,0) = lndmrk->getPointsXMean();
				real_observations(1,0) = RNUtils::fixAngleRad(lndmrk->getPointsYMean());
				printf("Baliza detectada. Distancia = %.3f, angulo = %.3f\n", real_observations(0,0), real_observations(1,0)*180.0/M_PI);

				mahalanobis_distance = 1000.0;
				i_mahalanobis = -1;

				// Comparación con cada una de las existentes en el mapa
				for(int j = 0; j < laserLandmarksCount; j++)
				{
					singleHk(0, 0) = fullHk(2*j, 0);
					singleHk(0, 1) = fullHk(2*j, 1);
					singleHk(0, 2) = fullHk(2*j, 2);
					singleHk(1, 0) = fullHk(2*j+1, 0);
					singleHk(1, 1) = fullHk(2*j+1, 1);
					singleHk(1, 2) = fullHk(2*j+1, 2);
					
					// Fiabilidad de medida_tomada - medida_estimada
					Matrix omega_inf = singleHk * Pk_pred_inf * ~singleHk + smallR_inf;
					
					// Medida predicha según la posición estimada por odometría
					smallObservations_inf(0, 0) = observations_inf(2*j, 0);
					smallObservations_inf(1, 0) = observations_inf(2*j+1, 0);
					
					// Distancia de Mahalanobis
					Matrix mdk = ~(real_observations - smallObservations_inf) * !omega_inf * (real_observations - smallObservations_inf);
					float md = std::sqrt(mdk(0, 0));
					//printf("  Mahalanobis -> %f > %f\n", md, mahalanobis_limit);
					// Guardar la más pequeña
					if(md < mahalanobis_distance)
					{
						mahalanobis_distance = md;
						i_mahalanobis = j*2;
					}
				}
				printf("Distancia mahalanobis: %f\n", mahalanobis_distance);
				if(i_mahalanobis == -1 || mahalanobis_distance > mahalanobis_limit)
				{
					if(i_mahalanobis == -1)
						printf("WOoooW i_mahal = -1\n");
					if(mahalanobis_distance > mahalanobis_limit)
						printf("Descartada por mahalanobis -> %f > %f\n", mahalanobis_distance, mahalanobis_limit);
					continue;
				}	

				// Una vez se ha identificado la baliza a la que corresponde (i_mahalanobis) se calcula la innovación y la H que se va a usar
				singleHk(0, 0) = fullHk(i_mahalanobis, 0);
				singleHk(0, 1) = fullHk(i_mahalanobis, 1);
				singleHk(0, 2) = fullHk(i_mahalanobis, 2);
				singleHk(1, 0) = fullHk(i_mahalanobis+1, 0);
				singleHk(1, 1) = fullHk(i_mahalanobis+1, 1);
				singleHk(1, 2) = fullHk(i_mahalanobis+1, 2);
				v_single_H.push_back(singleHk);
				printf("Baliza sup elegida. Distanc = %.3f, angulo = %.3f\n", observations_sup(i_mahalanobis, 0), observations_sup(i_mahalanobis+1, 0)*180/M_PI);
				printf("Baliza inf elegida. Distanc = %.3f, angulo = %.3f\n", observations_inf(i_mahalanobis, 0), observations_inf(i_mahalanobis+1, 0)*180/M_PI);


				single_inno_sup(0, 0) = real_observations(0,0) - observations_sup(i_mahalanobis, 0);
				single_inno_sup(1, 0) = real_observations(1,0) - observations_sup(i_mahalanobis+1, 0);
				single_inno_sup(1, 0) = RNUtils::fixAngleRad(single_inno_sup(1, 0));
				printf("Inno sup: dist = %f, ang = %f\n", single_inno_sup(0,0), single_inno_sup(1,0)*180/M_PI);
							
				single_inno_inf(0, 0) = real_observations(0,0) - observations_inf(i_mahalanobis, 0);
				single_inno_inf(1, 0) = real_observations(1,0) - observations_inf(i_mahalanobis+1, 0); 
				single_inno_inf(1, 0) = RNUtils::fixAngleRad(single_inno_inf(1, 0));
				printf("Inno inf: dist = %f, ang = %f\n", single_inno_inf(0,0), single_inno_inf(1,0)*180/M_PI);

				v_single_inno_sup.push_back(single_inno_sup);
				v_single_inno_inf.push_back(single_inno_inf);		

				// Baliza usada
				vsize_used ++;		
			}
		} // if(gn->isLaserSensorActivated())


		printf("Balizas detectadas = %d.   Balizas usadas = %d\n", vsize, vsize_used);
		gn->unlockLaserLandmarks();
		printf("3.\n");
		printf("4.\n");


		/** CORRECCIÓN final*/
		// Si se han obtenido observaciones se corrige, si no, no
		if(vsize_used > 0)
		{
			// Ahora sí vamos a construir las matrices enteras

			// Inicializar tamaño matrices
			R_sup = Matrix(2*vsize_used, 2*vsize_used);
			R_inf = Matrix(2*vsize_used, 2*vsize_used);

			Matrix Hk = Matrix(2*vsize_used,3);
			Matrix innovacion_sup = Matrix(2*vsize_used,1);
			Matrix innovacion_inf = Matrix(2*vsize_used,1);
			
			// Rellenamos
			for(int i = 0; i < vsize_used; i++)
			{
				if(i == 0)
					printf("5.\n");

				// Matrices de ruido
				R_sup(2*i, 2*i) = std::pow(incertidumbre_laser_dist_sup, 2) / 3.0;
				R_sup(2*i+1, 2*i+1) = std::pow(incertidumbre_laser_angl_sup, 2) / 3.0;

				R_inf(2*i, 2*i) = std::pow(incertidumbre_laser_dist_inf, 2) / 3.0;
				R_inf(2*i+1, 2*i+1) = std::pow(incertidumbre_laser_angl_inf, 2) / 3.0;

				// Matriz de observación
				singleHk = v_single_H.at(i);
				Hk(2*i,0) = singleHk(0,0);
				Hk(2*i,1) = singleHk(0,1);
				Hk(2*i,2) = singleHk(0,2);
				Hk(2*i+1,0) = singleHk(1,0);
				Hk(2*i+1,1) = singleHk(1,1);
				Hk(2*i+1,2) = singleHk(1,2);

				// Matrices de innovacion
				single_inno_sup = v_single_inno_sup.at(i);
				single_inno_inf = v_single_inno_inf.at(i);

				innovacion_sup(2*i,0) = single_inno_sup(0,0);
				innovacion_sup(2*i+1,0) = single_inno_sup(1,0);

				innovacion_inf(2*i,0) = single_inno_inf(0,0);
				innovacion_inf(2*i+1,0) = single_inno_inf(1,0);
			}

			// AHORA SÍ CORRECCIÓN DE EKF
			Matrix Sk_sup = Hk * Pk_pred_sup * ~Hk + R_sup;
			Matrix Sk_inf = Hk * Pk_pred_inf * ~Hk + R_inf;


			// NOTA: Hay que tratar el caso de 1 sola observación a parte, porque la clase Matrix no permite algunos cálculos de matriz(1,1)
			Matrix Wk_sup, Wk_inf;
			try
			{
				Wk_sup = Pk_pred_sup * ~Hk * !Sk_sup;
				Wk_inf = Pk_pred_inf * ~Hk * !Sk_inf;
			}
			catch(const std::exception& e)
			{
				printf("Error. Hay %d medidas \n", vsize_used);	
			}


			Pk_sup = Pk_pred_sup - Wk_sup*Sk_sup*~Wk_sup;
			Pk_inf = Pk_pred_inf - Wk_inf*Sk_inf*~Wk_inf;


			try
			{

			Matrix correccion_sup = Wk_sup * innovacion_sup;
			Matrix correccion_inf = Wk_inf * innovacion_inf;
			correccion_sup.print();
			correccion_inf.print();

			}catch(const std::exception& e)
			{
				printf("Algo raro con print matriz correcion");	
			}
			


			xk_sup = xk_pred_sup + Wk_sup * innovacion_sup;
			xk_sup(2, 0) = RNUtils::fixAngleRad(xk_sup(2, 0));

			xk_inf = xk_pred_inf + Wk_inf * innovacion_inf;
			xk_inf(2, 0) = RNUtils::fixAngleRad(xk_inf(2, 0));
		}
		else
		{
			printf("No se corrige\n");
			xk_sup = xk_pred_sup;
			xk_inf = xk_pred_inf;
			Pk_sup = Pk_pred_sup;
			Pk_inf = Pk_pred_inf;
		}
		printf("6.\n");
		

		/** Liberar los vectores */
		v_single_H.clear();
		v_single_inno_sup.clear();
		v_single_inno_inf.clear();
		

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
		sprintf(buffer, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%d\t%d\n", buffer_xk_pred_sup, buffer_xk_pred_inf, buffer_xk_sup, buffer_xk_inf, buffer_Pk_pred_sup, buffer_Pk_pred_inf, buffer_Pk_sup, buffer_Pk_inf, vsize_used, vsize_used);
		if(test != NULL)
		{
			fprintf(test, "%s", buffer);

		}
		printf("7.\n");


		/** Info balizas detectadas*/
		/*for(int i = 0; i < gn->getLaserLandmarks()->size(); i++)
		{
			printf("7. %d\n", i);
			// Puntero apuntando a la baliza actual
			RNLandmark* lndmrk = gn->getLaserLandmarks()->at(i);

			// Medidas de la baliza detectada
			float dist = lndmrk->getPointsXMean();
			float ang = RNUtils::fixAngleRad(lndmrk->getPointsYMean());
			fprintf(test2, "%f\t %f\t", dist, ang);
		}
		printf("7.b\n");
		// Como no siempre vamos a tener el mismo número de medidas, para escribir en el fichero relleno todas las que faltan con 0
		for(int i = gn->getLaserLandmarks()->size(); i < laserLandmarksCount; i++)
		{
			fprintf(test2, "%f\t %f\t", 0.0, 0.0);
		}*/
		printf("8.\n");


		/** ***** PUBLICACIÓN DE UNA POSICIÓN CRISP PARA UN POSIBLE CONTROLADOR ***** **/
		CG(0, 0) = (2 * xk_sup(0, 0) * std::sqrt(3*Pk_sup(0, 0)) + 2 * xk_inf(0, 0) * std::sqrt(3*Pk_inf(0, 0)) + xk_sup(0, 0) * std::sqrt(3*Pk_inf(0, 0)) + xk_inf(0, 0) * std::sqrt(3*Pk_sup(0, 0))) / (3 * (std::sqrt(3*Pk_sup(0, 0)) + sqrt(3*Pk_inf(0, 0))));
        CG(1, 0) = (2 * xk_sup(1, 0) * std::sqrt(3*Pk_sup(1, 1)) + 2 * xk_inf(1, 0) * std::sqrt(3*Pk_inf(1, 1)) + xk_sup(1, 0) * std::sqrt(3*Pk_inf(1, 1)) + xk_inf(1, 0) * std::sqrt(3*Pk_sup(1, 1))) / (3 * (std::sqrt(3*Pk_sup(1, 1)) + sqrt(3*Pk_inf(1, 1))));
		CG(2, 0) = (2 * xk_sup(2, 0) * std::sqrt(3*Pk_sup(2, 2)) + 2 * xk_inf(2, 0) * std::sqrt(3*Pk_inf(2, 2)) + xk_sup(2, 0) * std::sqrt(3*Pk_inf(2, 2)) + xk_inf(2, 0) * std::sqrt(3*Pk_sup(2, 2))) / (3 * (std::sqrt(3*Pk_sup(2, 2)) + sqrt(3*Pk_inf(2, 2))));
		CG(2, 0) = RNUtils::fixAngleRad(CG(2, 0));
		printf("9.\n");
				
		gn->setAltPose(ArPose(CG(0, 0), CG(1, 0), CG(2, 0)*180/M_PI));
		printf("10.\n");


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
		// Corregida
		printf("Pos corregida sup: %.8f, %.8f, %.8f\n", xk_sup(0, 0), xk_sup(1, 0), xk_sup(2, 0));
		printf("Pos corregida inf: %.8f, %.8f, %.8f\n", xk_inf(0, 0), xk_inf(1, 0), xk_inf(2, 0));
		printf("Fiabilidad corregida sup: %.8f, %.8f, %.8f\n", Pk_sup(0, 0), Pk_sup(1, 1), Pk_sup(2, 2));
		printf("Fiabilidad corregida inf: %.8f, %.8f, %.8f\n", Pk_inf(0, 0), Pk_inf(1, 1), Pk_inf(2, 2));

		/** COMPROBAR TRAPECIOS DEFORMES **/
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

			printf("trapecio X: %.4f, %.4f, %.4f %.4f\n", x1, x2, x3, x4);
			printf("trapecio Y: %.4f, %.4f, %.4f %.4f\n", y1, y2, y3, y4);
			printf("trapecio Th (grados): %.3f, %.3f, %.3f %.3f\n", th1*180/M_PI, th2*180/M_PI, th3*180/M_PI, th4*180/M_PI);

		static int cont_deforme_x = 0;
		static int cont_deforme_y = 0;
		static int cont_deforme_th = 0;

		if(x1 > x2 || x3 > x4)
		{
			printf(" TRAPECIO DEFORME X: %.6f, %.6f, %.6f %.6f\n", x1, x2, x3, x4);
			cont_deforme_x ++;
		}
		if(y1 > y2 || y3 > y4)
		{
			printf(" TRAPECIO DEFORME Y: %.6f, %.6f, %.6f %.6f\n", y1, y2, y3, y4);
			cont_deforme_y ++;
		}
		if(th1 > th2 || th3 > th4)
		{
			printf(" TRAPECIO DEFORME TH: %.6f, %.6f, %.6f %.6f\n", th1, th2, th3, th4);
			cont_deforme_th ++;			
		}
		printf("Contadores trapecios deformes: x = %d, y = %d, th = %d\n", cont_deforme_x, cont_deforme_y, cont_deforme_th);

		printf("11.\n");
		printf("CG: (%.8f, %.8f, %.8f)\n\n\n", CG(0, 0), CG(1, 0), CG(2, 0) * 180.0/M_PI);


	} 
	else 
	{
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

/** Calcula las observaciones de la baliza en base a la posición
* @param & Matriz Xk (Posición del robot en cuestión)
* @param & Matriz disp (Matriz de rotación para relacionar la posición de la cámara con la posición del robot)
* @param estructura baliza (Baliza en cuestión)
* @param & distancia calculada
* @param & ángulo calculado
*/
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