
#include <pruebaClase/pruebaClase.h>

//------------------------------------------------------------------------
//  Metodos privados de la clase
//------------------------------------------------------------------------

void jointState_callback(const sensor_msgs::JointStatePtr& msg)
{
  patas_joint_state = *msg;
//  ROS_INFO("Lectura de la posicion de las patas, son %d patas", patas_joint_state.name.size());
//  ROS_INFO("n joints %d", patas_joint_state.name.size());
  for (int i = 0; i < patas_joint_state.position.size(); i++)
  {
//    ROS_INFO("joint %d name %s, position %.2f, effort %.2f", i, patas_joint_state.name.at(i).c_str(), patas_joint_state.position.at(i), patas_joint_state.effort.at(i));
//    ROS_INFO("joint %d position  %.2f", i, patas_joint_state.position.at(i));
    patas_posicion_actual.at(i) = patas_joint_state.position.at(i);
//    patas_posicion_actual.push_back(msg->position.at(i));
//    ROS_INFO("joint %d position  %.2f", i, patas_posicion_actual[i]);
//    std::cout << patas_posicion_actual[i];
    patas_lectura_flag = true;
  }
}

/*
* Esta funcion levanta al robot desde el suelo.
* Se le manda un vector de consignas de posicion para cada pata,
* por lo normal suele ser: -0.260, 0.26, 0.260, -0.260, -0.260, 0.260
* Con este vector se queda en tripode alterno nada más erguirse.
*
* Dudas: 
* - El vector patas_posicion_actual viene de la lectura del joint_state. 
*	  Aqui tiene el mismo valor hoy hay que definirlo de nuevo
*
*/
bool stand_up(std::vector<double> patas_consignas_vector, std::vector<double> patas_posicion_actual)
{
  for(int i = 0; i < 6; i++)
  {
  	if(patas_posicion_origen.at(i) == false){
        float error = (patas_posicion_actual.at(i) - patas_consignas_vector.at(i));
        float velocity = -2.5 * error;
//        ROS_INFO("joint dentro del while %d position  %.2f", i, patas_posicion_actual[i]);
        pata[i]->setVelocity(velocity);

      if(abs(velocity) < 0.1)
      {
//        ROS_INFO("Estoy en el 0");
//        pata[i]->setPosition(-0.02);
        patas_posicion_origen.at(i) = true;
        pata[i]->setVelocity(0.0);
        pata[i]->setPosition(patas_consignas_vector.at(i));
      }
  	}
  }
      
  if(std::all_of(patas_posicion_origen.begin(), patas_posicion_origen.end(), [](bool posicion_origen) {return posicion_origen;}))
  {
  	ROS_INFO("Posicion origen = TRUE");
//        posicion_origen = true;float velocity_phase_1, velocity_flight;
    for(int i = 0; i < 6; i++)
    {
      patas_posicion_origen.at(i) = false;
    }
    return (true);
  }

  else
  	return (false);
}

/*
* Movimiento de las patas para el tripode alterno
*
*/

//Matriz de que las patas han llegado a su consigna
static std::vector<bool> leg_in_position_vector = {false, false, false, false, false, false};
static bool leg_in_position[tripods_quantity][tripods_length] = {{false, false, false},{false, false, false}};

bool movement(bool start_movement, int phase, int phase_mod)
{
	if(start_movement)
	{
		for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                if (leg_in_position[i][j] == false)
                {
                    if(phase_mod == 0)
                    {	
                    	if(phase == 0)
                    	{
	                    	ROS_INFO("Fase inicial");
	                    	// En la fase inicial, el tripode del suelo tiene que ir aún más lento 
	                    	// para que el el tripode que vuela pueda llegar a tiempo. 
	                    	// Esto es si la posicion de origen es 0, 0, 0, 0, 0, 0.
	                        pata[robot_legs_configuration[0][j]]->setVelocity(phase_one_velocity / 2);
	                        pata[robot_legs_configuration[1][j]]->setVelocity(phase_two_velocity);
	                    }
	                	else
	                  	{
	                    	ROS_INFO("Fase 1");
	                    	pata[robot_legs_configuration[0][j]]->setVelocity(phase_one_velocity);
	                    	pata[robot_legs_configuration[1][j]]->setVelocity(phase_two_velocity);
	                  	}
                	}

	                else
	                {
	                    ROS_INFO("Fase 2");
	                    pata[robot_legs_configuration[0][j]]->setVelocity(phase_two_velocity);
	                    pata[robot_legs_configuration[1][j]]->setVelocity(phase_one_velocity);
	                }
	            }
	        }
	    }
	}

	else
	{
		for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                //float tripod_one_angle, tripod_two_angle;
                float distance;
                float leg_actual_pos = fmod(patas_posicion_actual.at(robot_legs_configuration[i][j]), 2 * PI);
                static float angle;
//                ROS_INFO("phase module = %d", phase_mod);
                if(phase_mod == 0)
                {
                    if(i == 0)
                        angle = take_off_angle; //Angulo para tripode_1 y phase_0

                    else //(i == 1)
                        angle = take_land_angle;    //Angulo para el tripode_1 y phase_1
//                    ROS_INFO("angulo %.2f", angle);
                }
                else
                {
                    if(i == 0)
                        angle = take_land_angle;
                    else //(i = 1)
                        angle = take_off_angle;
                }

                distance = fabs(angle - leg_actual_pos);
//                ROS_INFO("Fase %.2f -- Pata %d -- Posicion %.2f -- Error %.2f -- Velocidad %.2f", phase_mod, robot_legs_configuration[i][j], leg_actual_pos, distance, pata[robot_legs_configuration[i][j]]->getVel());
                if(distance < 0.07)
                {
//                  ROS_INFO("Distancia correcta para la pata %d", robot_legs_configuration[i][j]);
                    leg_in_position[i][j] = true;
                    pata[robot_legs_configuration[i][j]]->setVelocity(0.0);

                    // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
                    if(i == 0)
                        leg_in_position_vector.at(j) = true;
                    else
                        leg_in_position_vector.at(j+3) = true;
                }
            }
        }
	}

	/* COMPROBACION DE QUE LOS TRIPODES ESTAN EN LA POSICION DESEADA */
    // Si estan todas las patas en posicion, entonces se resetea el estado de sus valores
    // Y tambien se devuelve el estado de la variable FLAG de la maquina de estados
    bool position_reached = std::all_of(leg_in_position_vector.begin(), leg_in_position_vector.end(), [](bool reached) {return reached;});

    if(position_reached)
//    if(leg_in_position[1][0] && leg_in_position[1][1] && leg_in_position[1][2])
    {
      ROS_INFO("Se ha alcanzado la consigna de todas las patas");
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                leg_in_position[i][j] = false;
                pata[robot_legs_configuration[i][j]]->setVelocity(0.0);

                if(i == 0)
                    leg_in_position_vector.at(j) = false;
                else
                    leg_in_position_vector.at(j+3) = false;
        }
      }
        phase++;
        phase_mod = phase % 2;
        ROS_INFO("phase = %d -- module = %.2f", phase, phase_mod);

        return true;
    }
    else
        return false;

}

void hexapodo::hexapodo()
{
	const int tripods_quantity = 2; 
	const int tripods_length = 3;

	std::vector<int> tripod_one = {0, 3, 4}; 
	std::vector<int> tripod_two = {1, 2, 5}; 

	robot_configuration [tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, 
															  {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

	std::vector<bool> patas_posicion_origen = {false, false, false, false, false, false};

	// Angulos
	float total_angle = 45;
	float total_angle_rads = total_angle * PI / 180;
	float take_off_angle = total_angle_rads / 2;
	float take_land_angle = 2 * PI - (total_angle_rads / 2);

}

