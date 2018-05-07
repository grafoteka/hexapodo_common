#include <altern_tripod/altern_tripod.h>

/******************************************************
 * Callbacks
 ******************************************************/
void altern_tripod::init_subscribers()
{
  walking_mode_subs = nh.subscribe("/walking_mode", 1, &altern_tripod::walking_mode_callback, this);
}

void altern_tripod::walking_mode_callback(const std_msgs::String::ConstPtr &msg)
{
  walking_mode_readed_ = msg->data;
}
/******************************************************
 * Movimiento de las patas
 ******************************************************/
bool altern_tripod::move_legs(bool start_movement)
{
  static bool position_reached = false;
  const int tripods_quantity = 2;
  const int tripods_length = 3;
  std::vector<int> tripod_one = {0, 3, 4};
  std::vector<int> tripod_two = {1, 2, 5};
  int robot_configuration[tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)},
                                                                 {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

  std::vector<bool> leg_in_position = {false, false, false, false, false, false};
  static bool start_movement_old;

  float phase_one_vel = (hexapod_common_methods_.phase_one_vel());
  float phase_two_vel = -(hexapod_common_methods_.phase_two_vel());

  int phase = 0;
  float phase_module = fmod(phase, 2);

  float total_angle_rads = hexapod_common_methods_.total_angle_rads_;
  float take_off_angle = total_angle_rads / 2;
  float take_land_angle = M_PI - total_angle_rads / 2;

  if(start_movement)
  {
    ROS_INFO("START_MOVEMENT");
    /* SET VELOCITIES */
    for(int i = 0; i < tripods_quantity; i++)
    {
      for(int j = 0; j < tripods_length; j++)
      {
        if (leg_in_position.at(robot_configuration[i][j]) == false)
        {
          if(phase_module == 0)
          {
            ROS_INFO("Fase 1");
            pata[robot_configuration[0][j]]->setVelocity(phase_one_vel);
            pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
          }
          else
          {
            ROS_INFO("Fase 2");
            pata[robot_configuration[0][j]]->setVelocity(phase_two_vel);
            pata[robot_configuration[1][j]]->setVelocity(phase_one_vel);
          }
        }
      }
    }
    start_movement = false;
  //        ROS_INFO("Fase = %.2f -- Velocidad fase 1 = %.2f -- Velocidad fase 2 = %.2f", phase_module, phase_one_vel, phase_two_vel);
  }

  /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
  if(!start_movement)//else
  {

    /* COMPROBACION DE QUE LOS TRIPODES ESTAN EN LA POSICION DESEADA */
    // Si estan todas las patas en posicion, entonces se resetea el estado de sus valores
    // Y tambien se devuelve el estado de la variable FLAG de la maquina de estados
    position_reached = std::all_of(leg_in_position.begin(), leg_in_position.end(), [](bool reached) {return reached;});

    ROS_INFO("ELSE");
    while (!position_reached){
      for(int i = 0; i < tripods_quantity; i++)
      {
        for(int j = 0; j < tripods_length; j++)
        {
          //float tripod_one_angle, tripod_two_angle;
          float distance;
          float leg_actual_pos = fmod(pata[robot_configuration[i][j]]->getPos(), 2 * PI);
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
          else  // phase_module =! 0
          {
            if(i == 0)
              angle = take_land_angle;
            else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};
              angle = take_off_angle;
          }

          distance = fabs(angle - leg_actual_pos);
          //ROS_INFO("Fase = %d -- modulo = %.2f -- Pata %d -- Posicion = %.2f -- Error = %.2f -- Velocidad = %.2f", phase, phase_module, robot_configuration[i][j], leg_actual_pos, distance, pata[robot_configuration[i][j]]->getVel());
          if(distance < 0.07)
          {
            //ROS_INFO("Distancia correcta para la pata %d", robot_configuration[i][j]);
            leg_in_position.at(robot_configuration[i][j]) = true;
            pata[robot_configuration[i][j]]->setPosition(leg_actual_pos);
            // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
            if(i == 0)
              leg_in_position.at(j) = true;
            else
              leg_in_position.at(j+3) = true;
          }
        }
      }
      position_reached = std::all_of(leg_in_position.begin(), leg_in_position.end(), [](bool reached) {return reached;});
      int position_reached_int = position_reached ? 1 : 0;
      //ROS_INFO("Posicion alcanzada = %d", position_reached_int);
    }
        ROS_INFO("FIN de un un ciclo");
  }


  if(position_reached)
//    if(leg_in_position[1][0] && leg_in_position[1][1] && leg_in_position[1][2])
  {
    ROS_INFO("Se ha alcanzado la consigna de todas las patas");
    for(int i = 0; i < tripods_quantity; i++)
    {
      for(int j = 0; j < tripods_length; j++)
      {
        leg_in_position.at(robot_configuration[i][j]) = false;
        pata[robot_configuration[i][j]]->setVelocity(0.0);

        if(i == 0)
          leg_in_position.at(j) = false;
        else
          leg_in_position.at(j+3) = false;
      }
    }

    phase++;
    phase_module = phase % 2;
    ROS_INFO("phase = %d -- module = %.2f", phase, phase_module);
    start_movement_old = false;

    position_reached = false;

    return true;
  }

  else
    return false;

}

/******************************************************
 * Maquina de estados
 ******************************************************/
bool altern_tripod::fsm()
{
  enum leg_state {origin, phase_one, phase_two, transition};  // Fases de la maquina de estados
  /*
   *
   * Origin:
   * Phase_one: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de despegue y el tripode 2 desde el angulo de aterrizaje
   * Phase_two: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de aterrizaje y el tripode 2 desde el angulo de despegue
   * Transition: son los estados en los que las patas se encuentran circulando
   *
   * */

  //

  // Y axis != 0 -> advance = TRUE
  // X axis != 0 -> turn = TRUE
  bool advance = hexapod_common_methods_.advance();
  bool turn_left = hexapod_common_methods_.turn_left();
  bool turn_right = hexapod_common_methods_.turn_right();
  bool reverse = hexapod_common_methods_.reverse();

  // R1 pulsado o no
  // R1 pulsado -> eStop = FALSE
  // R1 NO pulsado -> eStop = TRUE
  bool eStop =! hexapod_common_methods_.eStop_;

  static leg_state state_actual = phase_one;
  static leg_state state_last;
  static leg_state state_next = transition;

  // Si se da una consigna de movimiento y esta pulsado R1 se entra en la fsm de movimiento
  if((turn_left || turn_right || advance) && (eStop))
  {
    //ROS_INFO("Entro en la fsm");
    while(eStop && (advance))//turn_left || turn_right || advance))
    {
    switch(state_actual)
    {
      case phase_one:
      {
      ROS_INFO("Entro en el caso 1");
//        eStop = hexapod_common_methods_.eStop_;
//        advance = hexapod_common_methods_.advance();
//        turn_left = hexapod_common_methods_.turn_left();
//        turn_right = hexapod_common_methods_.turn_right();
        static bool flag;

        // Se suelta R1 y por tanto se sale
        //if(!turn_left && !turn_right && !advance)
        if(!eStop)
        {
          ROS_INFO("Phase 1 --> Exit");
          state_actual = phase_one;
          state_next = transition;
          return true;

          break;
        }

        if((advance/*(turn_left || turn_right || advance*/) && eStop && state_next == transition)
        {
          ROS_INFO("Phase 1 --> Start movement");
          flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_two;
          state_last = phase_one;
          ROS_INFO("Phase 1 --> Transition");

          break;
        }

      }

      case phase_two:
      {
        eStop = hexapod_common_methods_.eStop_;
        advance = hexapod_common_methods_.advance();
        turn_left = hexapod_common_methods_.turn_left();
        turn_right = hexapod_common_methods_.turn_right();
        static bool flag;
        if(!turn_left && turn_right && !advance)
        {
          ROS_INFO("Phase 2 --> Exit");
          state_actual = phase_two;
          state_next = transition;

          return true;
        }

        if((turn_left || turn_right || advance) && !eStop && state_next == transition)
        {
          ROS_INFO("Phase 2 --> Start movement");
          flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = phase_two;
          ROS_INFO("Phase 2 --> Transition");
        }

        break;
      }

      case transition:
      {
        static bool flag;
        // Phase 1 --> transition --> Phase 2
        if(state_last == phase_one && state_next == phase_two)
        {
          //ROS_INFO("transition from phase 1");
          //flag = this->move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 1");
            state_actual = phase_two;
            state_next = transition;
            state_last = transition;
          }
        }

        // Phase 2 --> transition --> Phase 1
        if(state_last == phase_two && state_next == phase_one)
        {
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

    }
  }
}

/******************************************************
 * Init
 ******************************************************/
void altern_tripod::init()
{
  ROS_INFO("Entro en el init");


  // Legs_in_position significa que todas las patas estan en posicion
  // En este caso significa que el robot se ha levantado y estan los
  // dos tripodes en posiciones estaticas. +- touch_angle (common_methods.total_angle_degrees = 60)
  legs_in_position_ = hexapod_stand_by_.getGrounded();
//  int legs_in_position_int_ = legs_in_position_ ? 1 : 0;
//  ROS_INFO("%d", legs_in_position_int_);

//  bool eStop_ = hexapod_common_methods_.eStop_;
//  int eStop_int_ = eStop_ ? 1 : 0;
//  ROS_INFO("%d", eStop_int_);
  std::string walking_mode_to_compare = "altern_tripod";
  static bool string_comparison; // Variable que guarda la comparacion de la lectura del modo de marcha
  if(walking_mode_readed_ == walking_mode_to_compare)
    string_comparison = true;
  else
    string_comparison = false;

  //ROS_INFO("Mensaje leido -> %s \n Mensaje para comparar -> %s", walking_mode_readed_.c_str(), walking_mode_to_compare.c_str());

  // Este bucle se usa para que se quede en la fsm siempre y cuando se este en el modo de tripode alterno
  while(string_comparison){
    //ROS_INFO("Comparacion correcta");

    if(legs_in_position_ && !hexapod_common_methods_.eStop_)
    {
      static bool flag = false;
      //ROS_INFO("Las patas estan en posicion");
      // Podemos ir a la fsm
      while(!flag)
      {
        flag = this->fsm();
      }
      flag = false;
    }

    if(!legs_in_position_ && !hexapod_common_methods_.eStop_)
    {
      ROS_INFO("Las patas no estan en posicion, llamamos a Stand UP");
      hexapod_stand_by_.stand_up();
    }


    if(walking_mode_readed_ == walking_mode_to_compare)
      string_comparison = true;
    else
      string_comparison = false;
  }


}


/******************************************************
 * Constructor
 ******************************************************/
altern_tripod::altern_tripod()
{
  ROS_INFO("Constructor creado altern_tripod");
  init_subscribers();

}



