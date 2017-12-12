#include <common_methods/hexapod_common_methods.h>
#include <c_leg/c_leg.h>
#include <ros/ros.h>

/******************************************************
 * Setup patas
 ******************************************************/
//Funcion setup de las patas
void common_methods::setup_legs(ros::NodeHandle &nh){
//  ROS_INFO("Entro en setup_patas");
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
    ROS_INFO("Pata numero %d ha sido inicializada", i);
  }

  return;
}

//Funcion que libera el espacio en memoria de las patas
void common_methods::delete_legs (){
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    delete pata[i];
  }
  return;
}

/******************************************************
 * Confirmacion de la lectura de las patas
 ******************************************************/
// Esta funcion devuelve:
// TRUE -> si las patas ya han sido leidas en la libreria CLeg
// FALSE -> si no han sido leido todavia. En cuyo caso las vuelve a leer
bool common_methods::legs_readed(){
  if((pata[0]->joint_state_read_flag) &&
     (pata[1]->joint_state_read_flag) &&
     (pata[2]->joint_state_read_flag) &&
     (pata[3]->joint_state_read_flag) &&
     (pata[4]->joint_state_read_flag) &&
     (pata[5]->joint_state_read_flag))
  {
//    ROS_INFO("Patas inicializadas correctamente");
    for(int i = 0; i < ROBOT_LEG_NUM; i++)
    {
//      ROS_INFO("Posicion de la pata %d -- %.2f", i, pata[i]->getPos());
    }

    return true;
  }

  else{
    for(int i = 0; i < ROBOT_LEG_NUM; i++){
      pata[i]->getPos();
    }
    return false;
  }

}


/******************************************************
 * Constructor
 ******************************************************/
common_methods::common_methods()
{
  ROS_INFO("Entro en el constructor");

  setup_legs(nh);

  legs_readed_flag = false;

  legs_readed_flag = legs_readed();


  ROS_INFO("Constructor iniciado");

}

/******************************************************
 * Destructor
 ******************************************************/
//common_methods::~common_methods()
//{
//  delete_legs();

//}
