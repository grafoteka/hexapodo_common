#ifndef HEXAPODO_FSM_H
#define HEXAPODO_FSM_H

//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  date: 9/may/2018
//  version: 1.0
//  Archivo: hexapodo_fsm.h
//  Descripcion: Archivo de cabezera para la maquina de estados de tripode alterno
//               para la marcha de un robot hexapodo.
//====================================================================================

#include <ros/ros.h>
#include <common_methods/hexapod_common_methods.h>
#include <geometry_msgs/Twist.h>

class hexapodo_fsm
{

private:

  float robot_vel_linear_;

  // ROS
  ros::NodeHandle nh;  //Node handle
  ros::Subscriber velocity_subs;  // Subscriptor al twist_cmd_vel

  // Callbacks
  void init_subscribers();  // Metodo que inicia los subscriber
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel); // Callback para el twist del joystick

  bool move_legs(bool start_movement);

  common_methods hexapod_common_methods_;

public:
  hexapodo_fsm();

  bool fsm(float vel_linear);
};

#endif // HEXAPODO_FSM_H
