#ifndef COMMON_METHODS_H
#define COMMON_METHODS_H

#include <c_leg/c_leg.h>
#include <ros/ros.h>

//#include <sensor_msgs/JointState.h>

class common_methods
{
private:

#define ROBOT_LEG_NUM 6 // Numero de patas del robot

  // libreria CLegs
  void setup_legs(ros::NodeHandle &nh);  // Inicializacion de las patas
  void delete_legs(); // Eliminacion de las patas




  // ROS
  ros::NodeHandle nh;  //Node handle

public:

  // Patas del robot
  CLeg *pata[ROBOT_LEG_NUM];

  common_methods(); // Constructor
//  ~common_methods();  // Destructor

//  std::vector<float> legs_actual_position; // Vector que almacena la posicion actual de cada pata -- su joint

  // Legs readed first time
  bool legs_readed();
  bool legs_readed_flag;

  int npatas() const;
  void setNpatas(int npatas);
};

#endif // COMMON_METHODS_H
