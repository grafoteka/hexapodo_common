//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  Archivo: hexapodo_fsm.cpp
//  Descripcion: Programa que lee del joystick el boton del circulo y selecciona el
//  modo de marcha segun el valor de este.
//====================================================================================


//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

static int modo = -1;
//enum operation_mode {stand_by, altern_tripod, tetrapod, wave};
//operation_mode operation_mode_actual;
//operation_mode operation_mode_last;
//operation_mode operation_mode_next;


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
  const int modos_totales = 5;
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
    modo = (boton_modo % modos_totales);
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
  //---------------------------------------------------------------------------------

  //Inicio del nodo en ROS y el node handle
  ros::init(argc, argv, "walking_modes");
  ros::NodeHandle nh;

  ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_callback);
  ros::Publisher walking_mode = nh.advertise<std_msgs::String>("walking_mode", 1000);

  ros::Rate rate(10.0);

  //---------------------------------------------------------------------------------
  //  Rutinas de una sola ejecucion
  //---------------------------------------------------------------------------------

  //---------------------------------------------------------------------------------
  //  Bucle del programa principal
  //---------------------------------------------------------------------------------

  ROS_INFO("Entro en el while");
  while(ros::ok()){

    static int modo_old = -1;
    static std_msgs::String msg; // Variable para el mensaje que se publica

  if(modo != modo_old)
  {
    modo_old = modo;


    switch(modo){

    // Stand by
    case 0:
        ROS_INFO("Modo Stand By");
        msg.data = "stand_by";
//      operation_mode_actual = stand_by;
      break;

    // Altern tripod
    case 1:
    {
      ROS_INFO("Modo Altern Tripod");
      msg.data = "altern_tripod";
      // Cuando las patas ya están en posicion
    }
//      operation_mode_actual = altern_tripod;
      break;

    // Tetrapod
    case 2:
      ROS_INFO("Modo Tetrapod");
      msg.data = "tetrapod";
//      operation_mode_actual = wave;
      break;

    // Wave
    case 3:
      ROS_INFO("Modo Wave");
      msg.data = "wave";
//      operation_mode_actual = tetrapod;
      break;

    case 4:
      ROS_INFO("Modo Ground");
      msg.data = "ground";
      break;

//    default:
//      ROS_INFO("Default");
//      msg.data = "default";
    }
  }

    walking_mode.publish(msg);

    rate.sleep();
    ros::spinOnce();

  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}

