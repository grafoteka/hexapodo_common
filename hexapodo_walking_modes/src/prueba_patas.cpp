#include <ros/ros.h>
#include "c_leg/c_leg.h"

ros::Rate *loop_rate;
PataC *pata[6];

bool waitFor(unsigned int seconds, void (*callback) ()){
  int i=0;
  while(i < seconds/0.1){
    i++;
    callback();
    loop_rate->sleep();
  }
  return true;
}

void cero(){

  for(int i=0; i<6; i++){
    ROS_INFO("Pata %d a posicion 0", i+1);
    pata[i]->setPosRef(0);
  }

  return;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "prueba_patas");

  ros::NodeHandle nh;

  loop_rate = new ros::Rate(10);

  ROS_INFO("Se inicializan las patas");
  for(int i=0; i<6; i++){
      ROS_INFO("Pata %d inicializada", i+1);
      pata[i] = new PataC(i+1, nh);
  }

  //Core loop
  while(ros::ok()){

      waitFor(2, cero);

      ROS_INFO("Pata 1 a posicion PI");
      pata[0]->setPosRef(PI);
      ROS_INFO("Pata 4 a posicion PI");
      pata[3]->setPosRef(PI);
      ROS_INFO("Pata 5 a posicion PI");
      pata[4]->setPosRef(PI);

      //waitFor(2);

      ROS_INFO("Pata 2 cambiada a control de velocidad");
      pata[1]->activarControlVelocidad();
      ROS_INFO("Pata 3 cambiada a control de velocidad");
      pata[2]->activarControlVelocidad();
      ROS_INFO("Pata 6 cambiada a control de velocidad");
      pata[5]->activarControlVelocidad();

      pata[1]->setVelRef(10.472);
      pata[2]->setVelRef(10.472);
      pata[5]->setVelRef(10.472);

      while(ros::ok()){
        continue;
      }
  }

  for(int i=0; i<6; i++){
    pata[i]->activarControlPosicion();
    pata[i]->setPosRef(0);
  }

  for(int i=0; i<6; i++){
      delete pata[i];
  }
}
