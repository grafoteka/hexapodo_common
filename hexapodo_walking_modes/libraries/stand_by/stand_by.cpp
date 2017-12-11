#include <stand_by/stand_by.h>


void stand_by::move_legs()
{
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
        float error = (hexapodo_stand_by.pata[tripod_two.at(i)]->getPos() - (0 - total_angle_rads / 2));
        float velocity = -2.5 * error;
        hexapodo_stand_by.pata[tripod_two.at(i)]->setVelocity(velocity);
        ROS_INFO("Error de la pata %d = %.2f", i, error);
        if (abs(velocity) < 0.1)
        {
          hexapodo_stand_by.pata[tripod_two.at(i)]->setPosition(0 - (total_angle_rads / 2));
          ROS_INFO("Posicion de la pata %d alcanzada", i);
        }
      }

      // TRIPOD ONE
      for(int i = 0; i < 3; i++)
      {
        float error = abs(hexapodo_stand_by.pata[tripod_one.at(i)]->getPos() - total_angle_rads / 2);
        float velocity = 2.5 * error;
        hexapodo_stand_by.pata[tripod_one.at(i)]->setVelocity(velocity);
        ROS_INFO("Error de la pata %d = %.2f", i, error);

        if (error < 0.1)
        {
          hexapodo_stand_by.pata[tripod_one.at(i)]->setPosition(total_angle_rads / 2);
          ROS_INFO("Posicion de la pata %d alcanzada", i);
        }
      }
    }

    else
    {
      legs_read_flag = hexapodo_stand_by.legs_readed();
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
