#include "hexapodo_fsm.h"

/******************************************************
 * Callbacks
 ******************************************************/
void hexapodo_fsm::init_subscribers()
{
  velocity_subs = nh.subscribe("/cmd_vel", 1, &hexapodo_fsm::cmd_vel_callback, this);
}

void hexapodo_fsm::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &vel)
{
  robot_vel_linear_ = vel->linear.x;
}

/******************************************************
 * Movimiento de las patas
 ******************************************************/
bool hexapodo_fsm::move_legs(bool start_movement)
{
  //static bool position_reached = false;
  const int tripods_quantity = 2;
  const int tripods_length = 3;
  std::vector<int> tripod_one = {0, 3, 4};
  std::vector<int> tripod_two = {1, 2, 5};
  int robot_configuration[tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)},
                                                               {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

  std::vector<bool> leg_in_position = {false, false, false, false, false, false};
  std::vector<bool> leg_in_position_vector = {false, false, false, false, false, false};
  static bool start_movement_old;

  float phase_one_vel = 0.93;//(hexapod_common_methods_.phase_one_vel());
  float phase_two_vel = 5.83;//(hexapod_common_methods_.phase_two_vel());

  int phase = 0;
  float phase_module = fmod(phase, 2);



  /*int start_movement_int = start_movement; int start_movement_old_int = start_movement_old;
  ROS_INFO("Start movement = %d -- Start movement old = %d", start_movement_int, start_movement_old_int);*/

  if(start_movement)
    {
      ROS_INFO("START_MOVEMENT");
        /* SET VELOCITIES */
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                if (!leg_in_position.at(robot_configuration[i][j]))
                {
                    if(phase_module == 0)
                    {
                        //Si estamos en el primer ciclo, el tripode 1 va por el suelo y el tripode 2 por el aire
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[0][j], phase_one_vel);
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[1][j], phase_two_vel);
                      /*if(phase == 0)
                      {
                        ROS_INFO("Fase inicial");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel / 2);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
                      }

                      else{*/
//                        ROS_INFO("Fase 1");
                        hexapod_common_methods_.pata[robot_configuration[0][j]]->setVelocity(phase_two_vel);
                        hexapod_common_methods_.pata[robot_configuration[1][j]]->setVelocity(phase_one_vel);
                      //}
                    }
                    else
                    {
//                        ROS_INFO("Fase 2");
                        hexapod_common_methods_.pata[robot_configuration[0][j]]->setVelocity(phase_one_vel);
                        hexapod_common_methods_.pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);

                    }
                }
            }
            start_movement_old = start_movement;

        }
        ROS_INFO("Fase = %.2f -- Velocidad fase 1 = %.2f -- Velocidad fase 2 = %.2f", phase_module, phase_one_vel, phase_two_vel);
        start_movement = false;
    }

    /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
    else if(!start_movement)
    {
//      ROS_INFO("ELSE");
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
              ROS_INFO("Indice i+j = %d", 3*i+j);
              if(!leg_in_position_vector.at(robot_configuration[i][j]))
              {
                int leg_in_position_vector_int = leg_in_position_vector.at(3*i+j);
                ROS_INFO("Pata %d", leg_in_position_vector_int);
                //float tripod_one_angle, tripod_two_angle;
                float leg_actual_pos = fmod(hexapod_common_methods_.pata[robot_configuration[i][j]]->getPos(), 2 * PI);
                static float angle;

                float total_angle_rads = 0.79; //0.79 rads = 45 degrees; hexapod_common_methods_.total_angle_rads_;
                float take_off_angle = total_angle_rads / 2;
                float take_land_angle = 2 * M_PI - total_angle_rads / 2;

//                ROS_INFO("phase module = %d", phase_module);
                if(phase_module == 0)
                {
                    if(i == 0)
                        angle = take_land_angle; //Angulo para tripode_1 y phase_0

                    else //(i == 1)
                        angle = take_off_angle;    //Angulo para el tripode_1 y phase_1
//                    ROS_INFO("angulo %.2f", angle);
                }
                else
                {
                    if(i == 0)
                        angle = take_off_angle;
                    else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};

                        angle = take_land_angle;
                }

                float distance = fabs(angle - leg_actual_pos);
//                ROS_INFO("Fase %.2f -- Pata %d -- Posicion %.2f -- Error %.2f -- Velocidad %.2f", phase_module, robot_configuration[i][j], leg_actual_pos, distance, hexapod_common_methods_.pata[robot_configuration[i][j]]->getVel());
                if(distance < 0.07)
                {
                  ROS_INFO("Distancia correcta para la pata %d", robot_configuration[i][j]);
//                    leg_in_position.at(3 * i + j) = true;
//                    pata[robot_configuration[i][j]]->setVelocity(0.0);
                    hexapod_common_methods_.pata[robot_configuration[i][j]]->setPosition(angle);//hexapod_common_methods_.pata[robot_configuration[i][j]]->getPos());

                    // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
                    if(i == 0)
                        leg_in_position_vector.at(j) = true;
                    else if (i == 1)
                        leg_in_position_vector.at(j+3) = true;
                }
              } // leg_in_position_vector
            }
        }
        //ROS_INFO("FIN de un un ciclo");
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
                leg_in_position.at(3 * i + j) = false;
                hexapod_common_methods_.pata[robot_configuration[i][j]]->setPosition(hexapod_common_methods_.pata[robot_configuration[i][j]]->getPos());

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
bool hexapodo_fsm::fsm(float vel_linear)
{
  enum leg_state {origin, phase_one, phase_two, transition};  // Fases de la maquina de estados
  /*
   *
   * Origin: Es la posicion inicial del robot, debe estar en tripode alterno por defecto. Con el tripode 1 para despegar y el tripode 2 en aterrizaje
   * Phase_one: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de despegue y el tripode 2 desde el angulo de aterrizaje
   * Phase_two: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de aterrizaje y el tripode 2 desde el angulo de despegue
   * Transition: son los estados en los que las patas se encuentran circulando
   *
   * */

  // Se cargan los estados por defecto de la fsm
  static leg_state state_actual = phase_one;
  static leg_state state_last;
  static leg_state state_next = transition;

  while(vel_linear > 0)
  {
    switch(state_actual)
    {
      case phase_one:
      {
        vel_linear = robot_vel_linear_;
        ROS_INFO("vel_linear = %.2f", vel_linear);
//        eStop = hexapod_common_methods_.eStop_;
//        advance = hexapod_common_methods_.advance();
//        turn_left = hexapod_common_methods_.turn_left();
//        turn_right = hexapod_common_methods_.turn_right();
        static bool flag;

        // Se suelta R1 y por tanto se sale
        //if(!turn_left && !turn_right && !advance)
        if(vel_linear <= 0)
        {
          ROS_INFO("Phase 1 --> Exit");
          state_actual = phase_one;
          state_next = transition;
          return true;

          break;
        }

        else if((vel_linear > 0) && (state_next == transition))
        {
          ROS_INFO("Phase 1 --> Start movement");
          flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_two;
          state_last = phase_one;
          ROS_INFO("Phase 1 --> Transition");

          break;
        }

      } // Phase one

      case phase_two:
      {
        ROS_INFO("Entro en fase 2");
        vel_linear = robot_vel_linear_;
        /*eStop = hexapod_common_methods_.eStop_;
        advance = hexapod_common_methods_.advance();
        turn_left = hexapod_common_methods_.turn_left();
        turn_right = hexapod_common_methods_.turn_right();*/
        static bool flag;
        if(vel_linear <= 0)
        {
          ROS_INFO("Phase 2 --> Exit");
          state_actual = phase_two;
          state_next = transition;

          return true;
        }

        if((vel_linear > 0) && (state_next == transition))
        {
          ROS_INFO("Phase 2 --> Start movement");
          flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = phase_two;
          ROS_INFO("Phase 2 --> Transition");
        }

        break;
      } // Phase two

      case transition:
      {
        static bool flag;
        // Phase 1 --> transition --> Phase 2
        if(state_last == phase_one && state_next == phase_two)
        {
          //ROS_INFO("transition from phase 1");
          flag = this->move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 1");
            state_actual = phase_two;
            state_next = transition;
            state_last = transition;
          }
        }

        // Phase 2 --> transition --> Phase 1
        else if(state_last == phase_two && state_next == phase_one)
        {
          //ROS_INFO("transition from phase 2");
          flag = this->move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 2");
            state_actual = phase_one;
            state_next = transition;
            state_last = transition;
          }
        }

        break;
        } // Transition
    }
  } // while(vel_linear > 0)

}

/******************************************************
 * Init
 ******************************************************/


/******************************************************
 * Constructor
 ******************************************************/
hexapodo_fsm::hexapodo_fsm()
{
  init_subscribers();
}


