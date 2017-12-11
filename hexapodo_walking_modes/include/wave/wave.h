#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class wave_mode
{
private:

#define ROBOT_LEG_NUM 6 // Numero de patas del robot

  void movimiento(float);

  void vel_calc(double);  // Funcion para el calculo de las velocidades
  double legs_velocity;


public:
  CLeg *pata[ROBOT_LEG_NUM];

  wave_mode();

};
