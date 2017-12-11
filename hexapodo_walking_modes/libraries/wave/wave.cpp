#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <wave/wave.h>
#include <robot_configuration/robot_configuration.h>

/******************************************************
 * Movimiento
 ******************************************************/
void wave_mode::movimiento(float robot_velocity)
{
  for(int i = 0; i < ROBOT_LEG_NUM; i++)
  {
    pata[i]->setVelocity(legs_velocity);
  }
}

/******************************************************
 * Calculo velocidad de las patas
 ******************************************************/
void wave_mode::vel_calc(double)
{
  // Habria que calcular el desplazamiento de la pata desde que toca el suelo
  // Pero de momento pongo solo una velocidad constante
  legs_velocity = 10.0;
}

/******************************************************
 * Constructor
 ******************************************************/
wave_mode::wave_mode()
{
  ROS_INFO("Clase creada wave");
}

