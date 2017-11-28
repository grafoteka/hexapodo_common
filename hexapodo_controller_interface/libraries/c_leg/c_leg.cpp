//====================================================================================
//  Autor: Raul Cebolla Arroyo
//  Matricula: 13069
//  Archivo: c_leg.h
//  Descripcion: Archivo de cabezera para describir la funcionalidad de una pata c de
//               un robot rhex y asi establecer su funcionalidad
//====================================================================================

//----------------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------------

#include "c_leg/c_leg.h"
#include <math.h>
#include <cmath>
#include <string>
#include <hexapodo_controller_interface/CLegCommand.h>

//----------------------------------------------------------------------------
//  Definicion de macros
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  Clase pata C
//----------------------------------------------------------------------------

//------------------------------------------------------------------------
//  Metodos privados de la clase
//------------------------------------------------------------------------

//Metodo que inicializan los clientes
void CLeg::inicControllerInterfaceClient(){
  controller_interface_command_client = nh->serviceClient<hexapodo_controller_interface::CLegCommand>("/hexapodo/" + nom + "_controller_interface/c_leg_command");
  return;
}

//Metodos que inicializan los subscriber
void CLeg::inicJointStateSub(){
    //Inicializacion de los subscriber
    joint_state_subs = nh->subscribe("/hexapodo/joint_states", 1, &CLeg::parseJointState, this);
    return;
}

//Metodo que actualiza los campos correspondientes a las variables de estado de la junta
void CLeg::parseJointState(const sensor_msgs::JointState &message){
    double msg_pos = message.position[num_pata-1];
    if (msg_pos > 0){
      pos = msg_pos - trunc(msg_pos/(2*PI))*(2*PI);
    }else{
      pos = msg_pos + std::fabs(trunc(msg_pos/(2*PI))*(2*PI));
    }
    vel = message.velocity[num_pata-1];
    eff = message.effort[num_pata-1];
    return;
}

//Metodo que envia una request de comando hacia el controller interface
bool CLeg::requestCommand (float value, bool position_command){

  //Servicio para commandar una pata
  hexapodo_controller_interface::CLegCommand command;

  //Se distingue si el comando es de posicion o de velocidad
  if(position_command){
    command.request.position_control = true;
  }else{
    command.request.position_control = false;
  }

  //Se completa el resto del comando
  if (position_command){
    command.request.value = value;
  }else{
    command.request.value = value;
  }


  //Se envia el comando
  controller_interface_command_client.call(command);

  //Se comprueba si ha existido algun tipo de problema en la llamada
  if (command.response.ok){
    return true;
  }else{
    return false;
  }

}

//------------------------------------------------------------------------
//  Constructor de la clase
//------------------------------------------------------------------------

//Constructor por defecto -- no hay
//CLeg();

//Constructor de la clase
CLeg::CLeg(unsigned int numero_pata, ros::NodeHandle &n){

    //Se asigna el handler del nodo
    nh = &n;

    //Asignacion del numero de pata y el nombre en consecuencia
    num_pata = numero_pata;
    nom = "pata" + std::to_string(num_pata);

    //Inicializacion de los clientes
    inicControllerInterfaceClient();

    //Inicializacion de subscribers
    inicJointStateSub();

    //El control de velocidad se inicia por defecto
    control_posicion = false;

    ros::spinOnce();

    return;
}

//------------------------------------------------------------------------
//  Metodos publicos
//------------------------------------------------------------------------

//Metodo que actualiza el estado de las patas
void CLeg::actualizarEstado(){
    ros::spinOnce();
    return;
}

//Metodos GET
float CLeg::getPos(){
    ros::spinOnce();
    return pos;
}

float CLeg::getVel(){
    ros::spinOnce();
    return vel;
}

float CLeg::getEff(){
    ros::spinOnce();
    return eff;
}

bool CLeg::getEstControl(){
    return control_posicion;
}

//Metodos que mandan los comandos de velocidad y posicion
void CLeg::setPosition(float value){
  if(requestCommand(value, true)){
    ROS_INFO("Leg %d commanded adequately", num_pata);
  }else{
    ROS_ERROR("Could not command leg %d properly", num_pata);
  }
}

void CLeg::setVelocity(float value){
  if(requestCommand(value, false)){
    ROS_INFO("Leg %d commanded adequately", num_pata);
  }else{
    ROS_ERROR("Could not command leg %d properly", num_pata);
  }
}
