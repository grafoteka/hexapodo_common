#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_configuration/robot_configuration.h>

/******************************************************
 * Setup patas
 ******************************************************/
//Funcion setup de las patas
void robot_configuration::setup_legs(ros::NodeHandle &nh){
//  ROS_INFO("Entro en setup_patas");
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
//    ROS_INFO("Pata numero %d ha sido inicializada", i);
  }

  return;
}

////Funcion que libera el espacio en memoria de las patas
//void robot_configuration::delete_legs (){
//  for (int i = 0; i < ROBOT_LEG_NUM; i++){
//    delete pata[i];
//  }
//  return;
//}

/******************************************************
 * Init subscriber
 ******************************************************/
//Metodos que inicializan los subscriber
void robot_configuration::init_joint_state_sub()
{
    //Inicializacion de los subscriber
    joint_state_subs = nh.subscribe("/hexapodo/joint_states", 1, &robot_configuration::joint_state_callback, this);
//    ROS_INFO("Suscriptor iniciado");
    return;
}

void robot_configuration::joint_state_callback(const sensor_msgs::JointStatePtr &msg)
{
  legs_joint_state = *msg;
  legs_actual_position.clear();
  for (int i = 0; i < legs_joint_state.name.size(); i++)
  {
    legs_actual_position.push_back(msg->position.at(i));
  }

  if(legs_joint_state.name.size() == ROBOT_LEG_NUM)
  {
    legs_joint_state_flag = true;
    return;
  }
}

int robot_configuration::npatas() const
{
  return npatas_;
}

void robot_configuration::setNpatas(int npatas)
{
  npatas_ = npatas;
}

void robot_configuration::walking_mode_fsm(int modo)
{
  switch(modo)
  {
  case 0:
    ROS_INFO("STAND BY");
    walking_mode_actual = stand_by;
    break;

  case 1:
    ROS_INFO("TRIPODE ALTERNO");

    break;

  case 2:
    ROS_INFO("ALL");

    break;

  case 3:
    ROS_INFO("WAVE");

    break;

  case 4:
    ROS_INFO("QUADRUPED");

    break;

  case 5:
    ROS_INFO("PRONKING");

    break;

  default:
    ROS_INFO("SOMETHING WRONG");

    break;
  }
}

/******************************************************
 * Constructor
 ******************************************************/
robot_configuration::robot_configuration()
{
  ROS_INFO("Entro en el constructor");

//  CLeg *pata[ROBOT_LEG_NUM];
  legs_joint_state_flag = false;

  init_joint_state_sub();
  setup_legs(nh);


  ROS_INFO("Constructor iniciado");

}

/******************************************************
 * Destructor
 ******************************************************/
//robot_configuration::~robot_configuration()
//{
//  delete_legs();
//  ROS_INFO("Destruido");

//  return;
//}
