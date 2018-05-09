#include <stand_by/stand_by.h>

bool stand_by::getGrounded() const
{
  return grounded_;
}

/******************************************************
 * Altern tripod pose
 * Esta funcion pone al robot en tripode alterno según
 * se levanta del suelo
 * Primero coloca el tripode 2 y luego el tripode 1
 ******************************************************/
bool stand_by::altern_tripod_pose()
{
  ROS_INFO("Altern tripod pose");

  float tripod_one_position = 0.39; //22.5 grados //0.52; // 30 grados
  float tripod_two_position = 2 * M_PI - 0.39; //5.89 rads --> 337.5 grados //2 * M_PI - 0.52; // 5.76 rads --> 330 grados

  static std::vector<float> leg_actual_pos(6);
  static std::vector<float> leg_actual_error(6);
  static std::vector<float> leg_desired_pos         = {tripod_one_position, tripod_two_position, tripod_two_position,
                                                       tripod_one_position, tripod_one_position, tripod_two_position};
  static std::vector<bool>  leg_reached_pos         = {false, false, false, false, false, false};
  static std::vector<bool>  tripod_one_reached_pos  ={false, false, false};
  const  std::vector<int>   tripod_one_legs_number  = {0, 3, 4};
  static std::vector<int>   tripod_one_safe_counter = {0, 0, 0};

  static std::vector<bool>  tripod_two_reached_pos  = {false, false, false};
  const  std::vector<int>   tripod_two_legs_number  = {1, 2, 5};
  static std::vector<int>   tripod_two_safe_counter = {0, 0, 0};
  static std::vector<int>   leg_safe_counter        = {0, 0, 0, 0, 0, 0};

  static bool pose = false;
  static bool pose_tripod_one = false;
  static bool pose_tripod_two = false;

  while(!pose)
  {
    while(!pose_tripod_two)
    {
      // TRIPOD 2
      for(int i = 0; i < 3; i++)
      {
        if(!tripod_two_reached_pos.at(i))
        {
          float leg_position = fmod(hexapodo_common_methods.pata[tripod_two_legs_number.at(i)]->getPos(), 2 * M_PI);
          float error = fabs(leg_position - tripod_two_position);

          //ROS_INFO("Posicion de la pata %d -- %.2f -- Error -- %.2f", tripod_one_legs_number.at(i), hexapodo_common_methods.pata[i]->getPos(), error);

          float velocity = 0.5 * error;
          if(tripod_two_safe_counter.at(i) > 100)
          {
            velocity = error + 0.01 * tripod_two_safe_counter.at(i);
          }

          hexapodo_common_methods.pata[tripod_two_legs_number.at(i)]->setVelocity(velocity);

          //ROS_INFO("Velocidad de la pata %d -- %.2f", tripod_one_legs_number.at(i), velocity);

          if (fabs(error) < 0.05)
          {
            hexapodo_common_methods.pata[tripod_two_legs_number.at(i)]->setPosition(tripod_two_position);
            ROS_INFO("Posicion de la pata %d alcanzada", tripod_two_legs_number.at(i));
            tripod_two_reached_pos.at(i) = true;
            tripod_two_safe_counter.at(i) = 0;
          }

          else
          {
            tripod_two_safe_counter.at(i)++;
          }
        } //if(!tripod_one_reached_pos.at(i))

        if(!pose_tripod_two)
        {
          if(std::all_of(tripod_two_reached_pos.begin(), tripod_two_reached_pos.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
          {
            ROS_INFO("Tripode 2 en posicion");
            pose_tripod_two = true;
          }
        }
      }
    } //Tripod 2

    // TRIPOD ONE
    while(!pose_tripod_one)
    {
      for(int i = 0; i < 3; i++)
      {
        if(!tripod_one_reached_pos.at(i))
        {
          float leg_position = fmod(hexapodo_common_methods.pata[tripod_one_legs_number.at(i)]->getPos(), 2 * M_PI);
          float error = fabs(leg_position - tripod_one_position);

          //ROS_INFO("Posicion de la pata %d -- %.2f -- Error -- %.2f", tripod_one_legs_number.at(i), hexapodo_common_methods.pata[i]->getPos(), error);

          float velocity = 0.5 * error;
          if(tripod_one_safe_counter.at(i) > 100)
          {
            velocity = error + 0.01 * tripod_one_safe_counter.at(i);
          }

          hexapodo_common_methods.pata[tripod_one_legs_number.at(i)]->setVelocity(velocity);

          //ROS_INFO("Velocidad de la pata %d -- %.2f", tripod_one_legs_number.at(i), velocity);

          if (fabs(error) < 0.05)
          {
            hexapodo_common_methods.pata[tripod_one_legs_number.at(i)]->setPosition(tripod_one_position);
            ROS_INFO("Posicion de la pata %d alcanzada", tripod_one_legs_number.at(i));
            tripod_one_reached_pos.at(i) = true;
            tripod_one_safe_counter.at(i) = 0;
          }

          else
          {
            tripod_one_safe_counter.at(i)++;
          }
        } //if(!tripod_one_reached_pos.at(i))

        if(!pose_tripod_one)
        {
          if(std::all_of(tripod_one_reached_pos.begin(), tripod_one_reached_pos.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
          {
            ROS_INFO("Tripode 1 en posicion");
            pose_tripod_one = true;
          }
        }
      } //for(int i = 0; i < 3; i++

    } //while(!pose_tripod_one)

    if(pose_tripod_one && pose_tripod_two)
    {
      pose = pose;
      return true;
    }

    else
    {
      return false;
    }

  } //while(!pose)

}

/******************************************************
 * Set Position
 ******************************************************/
bool stand_by::set_position(float tripod_one_desired_position, float tripod_two_desired_position)
{
  // Posicion de las patas en el suelo

  // Vector que verifica que todas las posiciones han llegado a su posicion
  static std::vector<bool> legs_in_position = {false, false, false, false, false, false};

  // Definicion de la configuracion del robot como una matriz de 2X3 -> i = 2; j = 3
  const int tripod_quantity = 2;
  const int tripod_length = 3;
  int robot_configuration[tripod_quantity][tripod_length] = {{tripod_one_.at(0), tripod_one_.at(1), tripod_one_.at(2)},
                                                             {tripod_two_.at(0), tripod_two_.at(1), tripod_two_.at(2)}};

    /*************************************************************************************************
     * Si el robot se encuentra en el suelo:
     * TRIPOD ONE: position -> stand up take_off_rads
     * TRIPOD TWO: position -> stand up take_land_rads
     * El tripod one, tiene que recorrer mas distancia, por eso tiene una velocidad un poco mayor.
    *************************************************************************************************/
    if(grounded_)
    {
      //double tripod_one_desired_position = 2 * M_PI - 0.52;
      //double tripod_two_desired_position = 0.52;
      // Contador de seguridad por si la pata no puede alcanzar la posicion objetivo
      static int safe_counter = 0;
      static int safe_counter_qty = 10000000;

      ROS_INFO("Stand UP bucle");
      for(int i = 0; i < tripod_quantity; i++)
      {
        for(int j = 0; j < tripod_length; j++)
        {
          //ROS_INFO("El robot esta en el suelo, empieza el movimiento");
          // First tripod
          if(i == 0)
          {
            if(!legs_in_position.at(j))
            {
              float error = fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_one_desired_position);
              ROS_INFO("Error de la pata %d --> %.2f", j, error);
              float velocity = fabs(3.5 * error);
//              if(safe_counter > safe_counter_qty)
//              {
//                velocity = velocity * 1.5;
//              }
//              ROS_INFO("Velocity tripod 1 = %.2f", velocity);
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.05)
              {
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setPosition(tripod_one_desired_position);
                ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                legs_in_position.at(j) = true;
                safe_counter = 0;
              }
            //hexapodo_common_methods.pata[robot_configuration[0][0]]->setVelocity(50.0);

            }
          }
          // Second tripod
          else
          {
            if(!legs_in_position.at(j + tripod_length))
            {
              float error = abs(fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position));
              float velocity = (3.50 * error);
//              if(safe_counter > safe_counter_qty)
//              {
//                velocity = velocity * 1.5;
//              }
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.05)
              {
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setPosition(tripod_two_desired_position);
                ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                legs_in_position.at(j + tripod_length) = true;
                safe_counter = 0;
              }
            }
          }
        }
      }
      safe_counter++;
    }
    
    /************************************************************
     * GROUND == FALSE
     * El robot esta erguido y se quiere volver al suelo:
     * TRIPOD ONE: position -> fmod(-4.5 rads)
     * TRIPOD TWO: position -> fmod(-4.5 rads)
    *************************************************************/
    /*else{
      const float ground_velocity = 0.0;
      static int safe_counter = 0;
      //ROS_INFO("Get DOWN");
      for(int i = 0; i < tripod_quantity; i++)
      {
        for(int j = 0; j < tripod_length; j++)
        {
          //hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(0.0);
          // First tripod
          if(i == 0)
          {
            if(!legs_in_position.at(j))
            {
              ROS_INFO("Posicion objetivo T1 --> %.2f", tripod_one_desired_position);
              float error = abs((fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_one_desired_position)));
//              float error = tripod_one_desired_position - hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos();
              float velocity = (ground_velocity);
//              if(safe_counter > 200)
//              {
//                velocity = 4.5 * error;
//              }
//              ROS_INFO("Error pata %d = %.2f -- velocidad = %.2f -- Posicion deseada = %.2f -- Posicion actual = %.2f", robot_configuration[i][j], error, velocity, tripod_one_desired_position, hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos());
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.1)
              {
                ROS_INFO("Error = %.2f", error);
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(0.0);
                ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                legs_in_position.at(j) = true;
                safe_counter = 0;
              }
            }
          }
            // Second tripod
            else
            {
              if(!legs_in_position.at(j + tripod_length))
              {
                float error = abs(fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position));
//                float error = tripod_two_desired_position - hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos();
                float velocity = -(ground_velocity);
//                if(safe_counter > 200)
//                {
//                  velocity = 4.5 * error;
//                }
//                ROS_INFO("Error pata %d = %.2f -- velocidad = %.2f -- Posicion deseada = %.2f -- Posicion actual = %.2f", robot_configuration[i][j], error, velocity, tripod_one_desired_position, hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos());
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
                if (fabs(error) < 0.05)
                {
                  hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(0.0);
                  ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                  legs_in_position.at(j + tripod_length) = true;
                  safe_counter = 0;
                }
              }
            }
          }
        }
      safe_counter++;

      }*/


    // Ahora toca chequear las condiciones de salida.
    if(std::all_of(legs_in_position.begin(), legs_in_position.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
    {
      for(int i = 0; i < 6; i++)
      {
        legs_in_position.at(i) = false;
      }
      ROS_INFO ("All legs in position");
      if(grounded_)
      {
        ROS_INFO("Grounded = false");
        grounded_ = !grounded_;
      }
      else
      {
        ROS_INFO("Grounded = true");
        grounded_ = true;
      }
      return true;
    }
    else
      return false;
//  }
}

/******************************************************
 * Metodo para volver al suelo
 ******************************************************/
void stand_by::get_down()
{
  ROS_INFO("Volvemos al suelo");
  const float legs_position_ground = M_PI_2; //1.78;
  static bool flag = false;
  while(!flag)
  {
    float tripod_one_position = legs_position_ground;
    float tripod_two_position = legs_position_ground;
    flag = this->set_position(tripod_one_position, tripod_two_position);
  }
  flag = false;
}


/******************************************************
 * Metodo para levantarse
 ******************************************************/
void stand_by::stand_up()
{
  ROS_INFO("Nos levantamos");
  static bool flag = false;
  const float take_off_angle_rads = 2 * M_PI + hexapodo_common_methods.total_angle_rads_ / 2;
  const float take_land_angle_rads = 2 * M_PI - hexapodo_common_methods.total_angle_rads_ / 2;

  static std::vector<float> leg_actual_pos(6);
  static std::vector<float> leg_actual_error(6);
  static std::vector<float> leg_desired_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static std::vector<bool>  leg_reached_pos = {false, false, false, false, false, false};
  static std::vector<int>   leg_safe_counter = {0, 0, 0, 0, 0, 0};

  static bool grounded = false;

  /*for(int i = 0; i < 6; i++){
    leg_actual_error.at(i) = fabs(hexapodo_common_methods.pata[i]->getPos() - leg_desired_pos.at(i));
  }*/

  while(!grounded)
  {
    for(int i = 0; i < 6; i++)
    {
      if(!leg_reached_pos.at(i))
      {
        float velocity;
        float error = fabs(hexapodo_common_methods.pata[i]->getPos() - leg_desired_pos.at(i));
        //ROS_INFO("Error de la pata %d -- %.2f", i, error);
        if (error > 1.9)
        {
          velocity = 5.0;
          hexapodo_common_methods.pata[i]->setVelocity(velocity);
        }
        else if ((0.5 < error) && (error <= 1.9) )
        {
          if(leg_safe_counter.at(i) > 100)
          {
            velocity = 2.0;
          }
          else
          {
            velocity = 1.0;
          }
          //ROS_INFO("Mierda");
          hexapodo_common_methods.pata[i]->setVelocity(velocity);
        }
        else if ((0.1 < error) && (error <= 0.5))
        {
          if(leg_safe_counter.at(i) > 100)
          {
            velocity = 1.0;
          }
          else
          {
            velocity = 0.4;
          }
          //ROS_INFO("Casi estamos");
          hexapodo_common_methods.pata[i]->setVelocity(velocity);
        }
        else if (error <= 0.05)
        {
          hexapodo_common_methods.pata[i]->setPosition(0.0);
          leg_reached_pos.at(i) = true;
          ROS_INFO("Leg %d position reached", i);
        }
        leg_safe_counter.at(i)++;
      }
    }
    if(std::all_of(leg_reached_pos.begin(), leg_reached_pos.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
    {
      ROS_INFO("Todas en posicion");
      grounded = true;
      //flag = false;
    }
  }

  /*if(flag)
  {

  }

  while(1)
  {}*/

  // Este era el codigo original que habia
  while(!flag)
  {
    flag = this ->altern_tripod_pose();
    //float tripod_one_position = take_off_angle_rads;
    //float tripod_two_position = take_land_angle_rads;
    //flag = this->set_position(tripod_one_position, tripod_two_position);
  }
  flag = false;
}

/******************************************************
 * FSM del metodo
 ******************************************************/
void stand_by::stand_by_fsm()
{
  if(!hexapodo_common_methods.eStop_ && grounded_)
  {
    ROS_INFO("Stand UP");
//    if(grounded_)
      this->stand_up();
//    else
  }
  if(!hexapodo_common_methods.eStop_ && !grounded_)  {
      this->get_down();
    }
}

/******************************************************
 * Constructor
 ******************************************************/
stand_by::stand_by()
{
  total_angle_degrees = 60;
  total_angle_rads = total_angle_degrees * M_PI / 180;

  tripod_one_ = {0, 3, 4};
  tripod_two_ = {1, 2, 5};

  grounded_ = true;  // Proviene del origen

  ROS_INFO("STAND BY Constructor creado");

}
