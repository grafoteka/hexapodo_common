#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <altern_tripod_fem/wave.h>


/******************************************************
 * Joint States
 ******************************************************/
void altern_tripod_fem::joint_state_callback(const sensor_msgs::JointStatePtr& msg)
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

/******************************************************
 * Constructor
 ******************************************************/
wave::wave()
{
  legs_joint_state_flag = false;

  init_joint_state_sub();
  ROS_INFO("Constructor iniciado");
};

/******************************************************
 * Destructor
 ******************************************************/
wave::~wave()
{
  ROS_INFO("Destruido");
}
