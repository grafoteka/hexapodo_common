#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define ROBOT_LEG_NUM  6  //Numero de patas del robot RHEX

class wave
{
private:
  float robot_speed;	// Velocidad de avance del cuerpo del robot [m/s]

  // ROS
  ros::NodeHandle nh;  //Node handle
  ros::Subscriber joint_state_subs; //Joint state subscriber

  void init_joint_state_sub(); //Metodos que inicializan los subscriber
  void joint_state_callback(const sensor_msgs::JointStatePtr& msg); //Metodo que actualiza los campos correspondientes a las variables de estado de la junta

  // Joint State
  sensor_msgs::JointState legs_joint_state; // Vector que almacena la posicion de las patas con el mensaje del jointState
  std::vector<float> legs_actual_position; // Vector que almacena la posicion actual de cada pata -- su joint
  bool legs_joint_state_flag;  // Bandera que indica que si es TRUE se ha leido al menos una vez el joint_states

  // libreria CLegs

public:
  // Patas del robot
  CLeg *pata[ROBOT_LEG_NUM];

  // Constructor
  wave();
  ~wave();

	
};
