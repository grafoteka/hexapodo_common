#include <stand_by/stand_by.h>


/******************************************************
 * Set Position
 ******************************************************/
bool stand_by::getGrounded() const
{
  return grounded_;
}

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

    // Si el robot se encuentra en el suelo:
    // TRIPOD ONE: position -> stand up take_off_rads
    // TRIPOD TWO: position -> stand up take_land_rads
    // El tripod one, tiene que recorrer mas distancia, por eso tiene una velocidad un poco mayor.
    if(grounded_)
    {
      // Contador de seguridad por si la pata no puede alcanzar la posicion objetivo
      static int safe_counter = 0;

      ROS_INFO("Stand UP bucle");
      for(int i = 0; i < tripod_quantity; i++)
      {
        for(int j = 0; j < tripod_length; j++)
        {
//          ROS_INFO("El robot esta en el suelo, empieza el movimiento");
          // First tripod
          if(i == 0)
          {
            if(!legs_in_position.at(j))
            {
              float error = fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_one_desired_position);
              float velocity = -(5.0 * error);
              if(safe_counter > 200)
              {
                velocity = velocity * 1.5;
              }
//              ROS_INFO("Velocity tripod 1 = %.2f", velocity);
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.01)
              {
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setPosition(tripod_one_desired_position);
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
              float error = fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position);
              float velocity = -(4.5 * error);
              if(safe_counter > 200)
              {
                velocity = velocity * 1.5;
              }
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.01)
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
    
    // El robot esta erguido y se quiere volver al suelo:
    // TRIPOD ONE: position -> fmod(-4.5 rads)
    // TRIPOD TWO: position -> fmod(-4.5 rads)
    else{
      const float ground_velocity = 3.0;

      ROS_INFO("Get DOWN");
      for(int i = 0; i < tripod_quantity; i++)
      {
        for(int j = 0; j < tripod_length; j++)
        {
          // First tripod
          if(i == 0)
          {
            if(!legs_in_position.at(j))
            {
              float error = (fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position));
//              float error = tripod_one_desired_position - hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos();
              float velocity = (ground_velocity);
//              ROS_INFO("Error pata %d = %.2f -- velocidad = %.2f -- Posicion deseada = %.2f -- Posicion actual = %.2f", robot_configuration[i][j], error, velocity, tripod_one_desired_position, hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos());
              hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
              if (fabs(error) < 0.1)
              {
                ROS_INFO("Error = %.2f", error);
                hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(0.0);
                ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                legs_in_position.at(j) = true;
              }
            }
          }
            // Second tripod
            else
            {
              if(!legs_in_position.at(j + tripod_length))
              {
                float error = fmod(hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position);
//                float error = tripod_two_desired_position - hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos();
                float velocity = -(ground_velocity);
//                ROS_INFO("Error pata %d = %.2f -- velocidad = %.2f -- Posicion deseada = %.2f -- Posicion actual = %.2f", robot_configuration[i][j], error, velocity, tripod_one_desired_position, hexapodo_common_methods.pata[robot_configuration[i][j]]->getPos());

                hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(velocity);
                if (fabs(error) < 0.05)
                {
                  hexapodo_common_methods.pata[robot_configuration[i][j]]->setVelocity(0.0);
                  ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                  legs_in_position.at(j + tripod_length) = true;
                }
              }
            }
          }
        }
      }


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
  while(!flag)
  {
    float tripod_one_position = take_off_angle_rads;
    float tripod_two_position = take_land_angle_rads;
    flag = this->set_position(tripod_one_position, tripod_two_position);
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
