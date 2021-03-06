//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  Archivo: tripode_alterno.cpp
//  Descripcion: Modo de marcha en tripode alterno para el robot hexapodo
//====================================================================================


//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <robot_configuration/robot_configuration.h>

//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

//sensor_msgs::JointState

//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

static int modo = 0;

//------------------------------------------------------------------------------------
//  Funciones de configuracion
//------------------------------------------------------------------------------------


/*********************************************
    CALLBACKS
**********************************************/

/******************************************************
 * Joystick.
 ******************************************************/

// Callback para el joystick.
// Lo que hacemos es leer el boton circulo del mando para ir cambiando de modo:
// stand_by_suelo, stand_by_up, tripode_alterno, wave, complete, ...
void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  static int boton_modo;
  static int boton_modo_old;
  // Si se pulsa el circulo
  if(joy->buttons[2] == 1)
  {
    boton_modo++;
  }
  if(boton_modo != boton_modo_old)
  {
    boton_modo_old = boton_modo;
    modo = (boton_modo % 5);
//    ROS_INFO("Numero de modo: %d", modo);
  }
}


//####################################################################################
//  Programa principal
//####################################################################################

int main(int argc, char **argv)
{

  //---------------------------------------------------------------------------------
  //  Setup
  //----------------------------------------------------------= new robot_configuration()-----------------------

  //Inicio del nodo en ROS y el node handle
  ros::init(argc, argv, "fsm_template");
  ros::NodeHandle nh;

  ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_callback);

  ros::Rate rate(10.0);

  ros::AsyncSpinner spinner(0); // Use 4 threads
  spinner.start();


  //---------------------------------------------------------------------------------
  //  Rutinas de una sola ejecucion
  //---------------------------------------------------------------------------------

  ROS_INFO("Antes de la clase");
  bool flag = true;
  robot_configuration hexapodo();

//  ROS_INFO("He creado la clase");
  //---------------------------------------------------------------------------------
  //  Bucle del programa principal
  //---------------------------------------------------------------------------------


  while(ros::ok()){

  static int modo_old;

  if(modo != modo_old)
  {
    ROS_INFO("Llamo a la fsm");
//    send_event(call);
    modo_old = modo;
  }

/*  switch(modo)
  {
  case 1:
    ROS_INFO("STAND_BY");
    break;
  case 2:
    ROS_INFO("TRIPODE ALTERNO");
    break;
  case 3:
    ROS_INFO("WAVE");
    break;
  case 4:
    ROS_INFO("COMPLETE");
    break;
  deafult:
    ROS_INFO("DEFAULT");
    break;

  }; */




    rate.sleep();

//    hexapodo.stand_up();
  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}
