#include <altern_tripod_fem/altern_tripod_fem.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <c_leg/c_leg.h>


/******************************************************
 * Setup patas
 ******************************************************/
//Funcion setup de las patas
void altern_tripod_fem::setup_legs(ros::NodeHandle &nh){
  ROS_INFO("Entro en setup_patas");
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
    ROS_INFO("Pata numero %d ha sido inicializada", i);
  }

  return;
}

/******************************************************
 * Init subscriber
 ******************************************************/
//Metodos que inicializan los subscriber
void altern_tripod_fem::init_joint_state_sub()
{
    //Inicializacion de los subscriber
    joint_state_subs = nh.subscribe("/hexapodo/joint_states", 1, &altern_tripod_fem::joint_state_callback, this);
    ROS_INFO("Suscriptor iniciado");
    return;
}

/******************************************************
 * Joint States
 ******************************************************/
void altern_tripod_fem::joint_state_callback(const sensor_msgs::JointStatePtr& msg)
{

  legs_joint_state = *msg;
//  ROS_INFO("Lectura de la posicion de las patas, son %d patas", legs_joint_state.name.size());
//  ROS_INFO("n joints %d", patas_joint_state.name.size());
  legs_actual_position.clear();
  for (int i = 0; i < legs_joint_state.name.size(); i++)
  {
//    ROS_INFO("joint %d name %s, position %.2f, effort %.2f", i+1, legs_joint_state.name.at(i).c_str(), legs_joint_state.position.at(i), legs_joint_state.effort.at(i));
//    ROS_INFO("joint %d position  %.2f", i, legs_joint_state.position.at(i));
//    legs_actual_position.at(i) = legs_joint_state.position.at(i);
    legs_actual_position.push_back(msg->position.at(i));
//    ROS_INFO("vector de tamaño  %d",  legs_actual_position.size());
//    ROS_INFO("joint %d position  %.2f", i, legs_actual_position.at(i));
//    std::cout << patas_posicion_actual[i];

  }

  if(legs_joint_state.name.size() == ROBOT_LEG_NUM)
  {
    legs_joint_state_flag = true;
    return;
  }
}

/******************************************************
 * Stand UP
 ******************************************************/
bool altern_tripod_fem::stand_up()
{
  ROS_INFO("Stand UP");
  //vector con la consigna de posicion de las patas.
  // En este caso, las patas las queremos situar en la posición absoluta 0.0
  std::vector<double> legs_setting_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int posicion_origen_int = stand_up_position; //ROS_INFO("posicion origen = %d", posicion_origen_int);
  int patas_lectura_flag_int = legs_joint_state_flag; ROS_INFO("posicion origen = %d -- patas_lectura_flag = %d", posicion_origen_int, patas_lectura_flag_int);

  // Solo se entra en el bucle si se ha leido, al menos una vez el joint_states y el flag de que el robot este
  // en la posicion de origen no se ha activado.
  if(!stand_up_position && legs_joint_state_flag)
  {
    ROS_INFO("Dentro del while");
    for(int i = 0; i < ROBOT_LEG_NUM; i++)
    {
      ROS_INFO("Dentro del for");
      // Si la pata no se encuentra en la posicion 0.0
      // se calcula el error y con ello la velocidad en un controlador P.
      // La velocidad es negativa para que las patas giren en sentido antihorario. Pero es el giro correcto.
      if(legs_origin_position.at(i) == false){
        ROS_INFO("Dentro del if, actual_position tamanyo = %d", legs_actual_position.size());
        float error = (legs_actual_position.at(i) - legs_setting_vector.at(i));
        ROS_INFO("Error = %.2f", error);
        float velocity = (-2.5) * error;
        ROS_INFO("joint dentro del while %d position  %.2f", i, legs_actual_position[i]);
        pata[i]->setVelocity(velocity);
        ROS_INFO("Velocidad asignada a la pata %d con una velocidad de %.2f", i, velocity);

        if(abs(velocity) < 0.1)
        {
          ROS_INFO("Pata %d Estoy en el 0", i);
  //        pata[i]->setPosition(-0.02);
          legs_origin_position.at(i) = true;
          pata[i]->setPosition(0.0);
        }
      }
    }
  }

  if(std::all_of(legs_origin_position.begin(), legs_origin_position.end(), [](bool posicion_origen) {return posicion_origen;})){
    ROS_INFO("Posicion origen = TRUE");
//        posicion_origen = true;float velocity_phase_1, velocity_flight;
    for(int i = 0; i < 6; i++)
    {
      legs_origin_position.at(i) = false;
    }
    stand_up_position = true;
    return (true);
    }
  else
  {
    ROS_INFO("Else");
    stand_up_position = false;
    return (false);
  }
}

/******************************************************
 * Calculos de velocidades
 ******************************************************/

void altern_tripod_fem::vel_calcs(float robot_speed)
{
    const float leg_diameter = 0.200;
    float alpha = total_angle_degrees;
  	float lStep = (2 * PI * alpha * leg_diameter) / 360;
    float totSteps = (1/robot_speed) / lStep;
  	float steps_per_second = 1 / totSteps;
  	float alpha_rads = alpha * PI / 180;
    phase_one_vel = alpha_rads * (steps_per_second * 100);
//    ROS_INFO("phase one velocity = %.2f, robot speed = %.2f", phase_one_vel, robot_speed);

  	// Calculo de la velocidad de vuelo segun el porcentaje de seguridad
    const float phase_two_vel_security = 0.9;
    phase_two_vel = (2 * PI - alpha_rads) * (steps_per_second * phase_two_vel_security * 100);
  //  phase_one_vel = ground_vel_rads;
  //  phase_two_vel = fly_vel_rads;
    if ((phase_one_vel != phase_one_vel_old) || (phase_two_vel != phase_two_vel_old))
  	{
      phase_one_vel_old = phase_one_vel;
      phase_two_vel_old = phase_two_vel;
//      ROS_INFO("phase 1 velocity = %.2f  --- phase 2 velocity = %.2f", phase_one_vel, phase_two_vel);
  	}

//    ROS_INFO("Velocidad Calculada");

    return;
}

/******************************************************
 * Movimiento de las patas
 ******************************************************/

bool altern_tripod_fem::move_legs(bool start_movement)
{
  int robot_configuration[groups_quantity][groups_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

  int start_movement_int = start_movement; int start_movement_old_int = start_movement_old;
  ROS_INFO("Start movement = %d -- Start movement old = %d", start_movement_int, start_movement_old_int);

  if(start_movement != start_movement_old)
    {
      ROS_INFO("START_MOVEMENT");
        /* SET VELOCITIES */
        for(int i = 0; i < groups_quantity; i++)
        {
            for(int j = 0; j < groups_length; j++)
            {
                if (leg_in_position[i][j] == false)
                {
                    if(phase_module == 0)
                    {
                        //Si estamos en el primer ciclo, el tripode 1 va por el suelo y el tripode 2 por el aire
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[0][j], phase_one_vel);
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[1][j], phase_two_vel);
                      if(phase == 0)
                      {
                        ROS_INFO("Fase inicial");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel / 2);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
                      }

                      else{
                        ROS_INFO("Fase 1");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
                      }
                    }
                    else
                    {
                        ROS_INFO("Fase 2");
                        pata[robot_configuration[0][j]]->setVelocity(phase_two_vel);
                        pata[robot_configuration[1][j]]->setVelocity(phase_one_vel);

                    }
                }
            }
            start_movement_old = start_movement;

        }
//        ROS_INFO("Fase = %.2f -- Velocidad fase 1 = %.2f -- Velocidad fase 2 = %.2f", phase_module, phase_one_vel, phase_two_vel);
    }


    /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
    else
    {
      ROS_INFO("ELSE");
        for(int i = 0; i < groups_quantity; i++)
        {
            for(int j = 0; j < groups_length; j++)
            {
                //float tripod_one_angle, tripod_two_angle;
                float distance;
                float leg_actual_pos = fmod(legs_actual_position.at(robot_configuration[i][j]), 2 * PI);
                static float angle;
//                ROS_INFO("phase module = %d", phase_module);
                if(phase_module == 0)
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
                    else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};

                        angle = take_off_angle;
                }

                distance = fabs(angle - leg_actual_pos);
//                ROS_INFO("Fase %.2f -- Pata %d -- Posicion %.2f -- Error %.2f -- Velocidad %.2f", phase_module, robot_configuration[i][j], leg_actual_pos, distance, pata[robot_configuration[i][j]]->getVel());
                if(distance < 0.07)
                {
//                  ROS_INFO("Distancia correcta para la pata %d", robot_configuration[i][j]);
                    leg_in_position[i][j] = true;
                    pata[robot_configuration[i][j]]->setVelocity(0.0);

                    // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
                    if(i == 0)
                        leg_in_position_vector.at(j) = true;
                    else
                        leg_in_position_vector.at(j+3) = true;
                }
            }
        }
//        ROS_INFO("FIN de un un ciclo");
    }

    /* COMPROBACION DE QUE LOS TRIPODES ESTAN EN LA POSICION DESEADA */
    // Si estan todas las patas en posicion, entonces se resetea el estado de sus valores
    // Y tambien se devuelve el estado de la variable FLAG de la maquina de estados
    bool position_reached = std::all_of(leg_in_position_vector.begin(), leg_in_position_vector.end(), [](bool reached) {return reached;});

    if(position_reached)
//    if(leg_in_position[1][0] && leg_in_position[1][1] && leg_in_position[1][2])
    {
      ROS_INFO("Se ha alcanzado la consigna de todas las patas");
        for(int i = 0; i < groups_quantity; i++)
        {
            for(int j = 0; j < groups_length; j++)
            {
                leg_in_position[i][j] = false;
                pata[robot_configuration[i][j]]->setVelocity(0.0);

                if(i == 0)
                    leg_in_position_vector.at(j) = false;
                else
                    leg_in_position_vector.at(j+3) = false;
        }
      }
        phase++;
        phase_module = phase % 2;
        ROS_INFO("phase = %d -- module = %.2f", phase, phase_module);
        start_movement_old = false;
        return true;
    }
    else
        return false;

}

/******************************************************
 * Maquina de estados
 ******************************************************/

bool altern_tripod_fem::fem(bool exit)
{
  enum leg_state {origin, phase_one, phase_two, transition};
  leg_state state_actual = origin;
  leg_state state_last;
  leg_state state_next;

  int robot_configuration[groups_quantity][groups_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)}, {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

	// Matriz que crea la estructura de las patas del robot

    bool flag;

    switch(state_actual)
    {
        case origin:
            if(!exit)
            {
              ROS_INFO("Origin");
                flag = this->move_legs(true);
                state_actual = transition;
                state_next = phase_one;
                state_last = origin;
            }
            if(exit)
            {
              ROS_INFO("origin exit");
              for(int i = 0; i < groups_quantity; i++)
              {
                  for(int j = 0; j < groups_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_configuration[0][j]]->setPosition(take_off_angle);
                    }
                    else
                      pata[robot_configuration[1][j]]->setPosition(take_land_angle);
                  }
              }
              flag = true;
            }
            return flag;

        break;

        case phase_one:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_1");
                flag = this->move_legs(true);
                state_actual = transition;
                state_next = phase_two;
                state_last = phase_one;
                ROS_INFO("Move to TRANSITION");
            }
            if(exit)
            {
              ROS_INFO("phase_1 exit");
              for(int i = 0; i < groups_quantity; i++)
              {
                  for(int j = 0; j < groups_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_configuration[0][j]]->setPosition(take_off_angle);
                    }
                    else
                      pata[robot_configuration[1][j]]->setPosition(take_land_angle);
                  }
              }
            }

        break;

        case phase_two:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_2");
                flag = this->move_legs(true);
                state_actual = transition;
                state_next = phase_one;
                state_last = phase_two;
            }
            if(exit)
            {
              ROS_INFO("phase_2_exit");
              for(int i = 0; i < groups_quantity; i++)
              {
                  for(int j = 0; j < groups_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_configuration[0][j]]->setPosition(take_land_angle);
                    }
                    else
                      pata[robot_configuration[1][j]]->setPosition(take_off_angle);
                  }
              }
            }

        break;

        case transition:
            if(state_last == origin && state_next == phase_one){
                ROS_INFO("transition from origin");
                flag = this->move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from origin");
                    state_actual = phase_one;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_one && state_next == phase_two){
              ROS_INFO("transition from phase 1");
                flag = this->move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_two;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_two && state_next == phase_one){
              ROS_INFO("transition from phase 2");
                flag = this->move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_one;
                    state_next = transition;
                    state_last = transition;
                }
            }


        break;

    }	
}

/******************************************************
 * Constructor
 ******************************************************/

altern_tripod_fem::altern_tripod_fem(bool exit, float total_angle, float robot_speed)
{
  //Patas del robot RHex
  CLeg *pata[ROBOT_LEG_NUM];

  // Variables de move_legs
  start_movement_old = false;

  // Flag que indica que se ha leido el joint states al menos una vez
  legs_joint_state_flag = false;

  flag = false; //flag que indica que ha terminado una transicion

  groups_quantity = 2;
  groups_length = 3;

  phase = 0;
  phase_module = 0.0;

  leg_in_position_vector = {false, false, false, false, false, false};

  phase_one_vel = 0.0;
  phase_one_vel_old = 0.0;
  phase_two_vel = 0.0;
  phase_two_vel_old = 0.0;

  total_angle_degrees = total_angle;
  total_angle_rads = total_angle * PI / 180;
  take_off_angle = total_angle_rads / 2;
  take_land_angle = 2 * PI - (total_angle_rads / 2);

  tripod_one = {0, 3, 4};
  tripod_two = {1, 2, 5};

  // joint_state_subscriber
//  legs_actual_position(6);

  // stand up

  stand_up_position = false;
  legs_origin_position = {false, false, false, false, false, false};

  // Inicializacion del subscriber
  init_joint_state_sub();
  ROS_INFO("Voy a setup_legs");
  setup_legs(nh);

//  if(legs_joint_state_flag)
//  {
//    ROS_INFO("Entro");
//    fem(exit);
//  }


  ROS_INFO("Constructor iniciado");

}



