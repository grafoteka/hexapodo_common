//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  Fecha: 13/diciembre/2017
//  Archivo: hexapodo_fsm_listener.cpp
//  Descripcion: Programa que lee el mensaje de la maquina de estados de los modos de marcha
//====================================================================================


//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/spinner.h>
#include <string>
#include <stand_by/stand_by.h>
#include <altern_tripod/altern_tripod.h>

//---------------------------------------------------------------------------------
//  Variables
//---------------------------------------------------------------------------------
std::string walking_mode;

// Este enum se utiliza para mapear los strings recibidos de los modos de marcha
// y poder emplear el switch para seleccionar los modos de marcha
enum modes {
  stand_by_mode,
  altern_tripod_mode,
  tetrapod_mode,
  wave_mode,
  ground_mode,
  error_mode
};

modes mode_actual;
modes mode_last;

//---------------------------------------------------------------------------------
//  Funciones
//---------------------------------------------------------------------------------
modes mode_read(std::string input) {
  if( input == "stand_by" ) return stand_by_mode;
  if( input == "altern_tripod" ) return altern_tripod_mode;
  if( input == "tetrapod" ) return tetrapod_mode;
  if( input == "wave" ) return wave_mode;
  if( input == "ground" ) return ground_mode;

  return error_mode;
};


//---------------------------------------------------------------------------------
//  Callbacks
//---------------------------------------------------------------------------------
void hexapodo_fsm_callback(const std_msgs::String::ConstPtr& msg)
{
  walking_mode = msg->data;
//  ROS_INFO("I heard: %s", walking_mode.c_str());
  mode_read(walking_mode);
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
  ros::init(argc, argv, "hexapodo_fsm_listener");
  ros::NodeHandle nh;

  ros::Subscriber hexapodo_fsm = nh.subscribe("/walking_mode", 1, hexapodo_fsm_callback);

  ros::Rate rate(10.0);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  stand_by hexapod_stand_by;
  altern_tripod hexapod_altern_tripod;

  //---------------------------------------------------------------------------------
  //  Rutinas de una sola ejecucion
  //---------------------------------------------------------------------------------

  //---------------------------------------------------------------------------------
  //  Bucle del programa principal
  //---------------------------------------------------------------------------------

  while(ros::ok()){
    
    switch(mode_read(walking_mode))
    {
    case stand_by_mode:
    {
      mode_actual = stand_by_mode;
      if(mode_actual != mode_last){
        ROS_INFO("Stand by");
        mode_last = mode_actual;
      }
      hexapod_stand_by.stand_by_fsm();
      break;
    }

    case altern_tripod_mode:
      mode_actual = altern_tripod_mode;
      if(mode_actual != mode_last){
        ROS_INFO("Altern tripod");
        mode_last = mode_actual;
      }

      // Llamamos a la fsm de altern_tripod, desde ella comprobamos la posicion de los tripodes
      hexapod_altern_tripod.init();
      break;

    case tetrapod_mode:
      mode_actual = tetrapod_mode;
      if(mode_actual != mode_last){
        ROS_INFO("Tetrapod");
        mode_last = mode_actual;
      }
      break;

    case wave_mode:
      mode_actual = wave_mode;
      if(mode_actual != mode_last){
        ROS_INFO("Wave");
        mode_last = mode_actual;
      }
      break;

    case ground_mode:
      mode_actual = ground_mode;
      if(mode_actual != mode_last){
        ROS_INFO("Ground");
        mode_last = mode_actual;
      }
      hexapod_stand_by.stand_by_fsm();
      break;

    case error_mode:
      ROS_INFO("Error");
      break;

    }

  }
//---------------------------------------------------------------------------------
//  Rutinas de salida
//---------------------------------------------------------------------------------

//deletePatas();


return 0;

}
