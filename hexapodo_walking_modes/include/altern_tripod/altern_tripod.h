#ifndef ALTERN_TRIPOD_H
#define ALTERN_TRIPOD_H

//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  date: 12/december/2017
//  version: 1.0
//  Archivo: altern_tripod.h
//  Descripcion: Archivo de cabezera para describir la funcionalidad de un modo de
//               marcha en tripode alterno para un robot hexapodo.
//====================================================================================

#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <common_methods/hexapod_common_methods.h>
#include <stand_by/stand_by.h>

class altern_tripod : public common_methods
{
private:

  // Variable que lee el estado actual del modo de marcha.
  std::string walking_mode_readed_;
  //std::string walking_mode_to_compare_ = "altern_tripod"; // Variable para comparar que es el modo de marcha correcto

  // Variable que chequea si el robot se encuentra en stand_by
  bool legs_in_position_;

  common_methods hexapod_common_methods_;
  stand_by hexapod_stand_by_;

  // ROS
  ros::NodeHandle nh;  //Node handle
  // Subscriptor a walking_mode -> Es para realizar la comprobacion de si se sigue en el estado Altern_tripod
  // En caso de que se cambie el modo se sale de la funcion del modo altern tripod
  ros::Subscriber walking_mode_subs;

  // Callbacks
  void init_subscribers();  // Metodo que inicia los subscriber
  void walking_mode_callback(const std_msgs::String::ConstPtr& msg);

  // vectores de las patas
  std::vector<int> tripod_one_ = {0, 3, 4};
  std::vector<int> tripod_two_ = {1, 2, 5};

  int tripods_quantity_ = 2;
  int tripods_length_ = 3;

  bool move_legs(bool); // Funcion que mueve las patas

  bool fsm(); // Funcion que es la maquina virtual de las fases

public:
  altern_tripod();  // Constructor
  void init();

};

#endif // ALTERN_TRIPOD_H
