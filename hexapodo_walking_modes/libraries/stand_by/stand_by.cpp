#include <stand_by/stand_by.h>


void stand_by::get_position()
{
  static std::vector<bool> legs_in_position = {false, false, false, false, false, false};

  ROS_INFO("Entro en el metodo");

  while(grounded)
  {
    static bool legs_read_flag = hexapodo_stand_by.legs_readed();
//    ROS_INFO("Valor varibale = %d", legs_read_flag);
    if(legs_read_flag)
    {
      // TRIPOD TWO
      for(int i = 0; i < 3; i++)
      {
        if(!legs_in_position.at(tripod_two.at(i)))
        {
  //        float error = (hexapodo_stand_by.pata[tripod_two.at(i)]->getPos() - (0 - total_angle_rads / 2));
          float error = fmod(hexapodo_stand_by.pata[tripod_two.at(i)]->getPos(), 2 * M_PI - total_angle_rads / 2);
          float velocity = -2.5 * error;
          hexapodo_stand_by.pata[tripod_two.at(i)]->setVelocity(velocity);
          ROS_INFO("Error de la pata %d = %.2f", i, error);
          if (abs(velocity) < 0.1)
          {
            hexapodo_stand_by.pata[tripod_two.at(i)]->setPosition(0 - (total_angle_rads / 2));
            ROS_INFO("Posicion de la pata %d alcanzada", i);
            legs_in_position.at(tripod_two.at(i)) = true;
          }
        }
      }

      // TRIPOD ONE
      for(int i = 0; i < 3; i++)
      {
        if(!legs_in_position.at(tripod_one.at(i)))
        {
          float error = abs(hexapodo_stand_by.pata[tripod_one.at(i)]->getPos() - total_angle_rads / 2);
          float velocity = 2.5 * error;
          hexapodo_stand_by.pata[tripod_one.at(i)]->setVelocity(velocity);
          ROS_INFO("Error de la pata %d = %.2f", i, error);

          if (error < 0.1)
          {
            hexapodo_stand_by.pata[tripod_one.at(i)]->setPosition(total_angle_rads / 2);
            ROS_INFO("Posicion de la pata %d alcanzada", i);
            legs_in_position.at(tripod_one.at(i)) = true;
          }
        }
      }
    }

    else
    {
      legs_read_flag = hexapodo_stand_by.legs_readed();
    }

    if(std::all_of(legs_in_position.begin(), legs_in_position.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
    {
      ROS_INFO ("All legs in position");
      grounded = false;
    }

  }
}

/******************************************************
 * Constructor
 ******************************************************/
stand_by::stand_by()
{
  total_angle_degrees = 60;
  total_angle_rads = total_angle_degrees * M_PI / 180;

  tripod_one = {0, 3, 4};
  tripod_two = {1, 2, 5};

  grounded = true;  // Proviene del origen

  ROS_INFO("STAND BY Constructor creado");

}
