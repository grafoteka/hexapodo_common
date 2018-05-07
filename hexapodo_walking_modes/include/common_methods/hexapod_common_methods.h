#ifndef COMMON_METHODS_H
#define COMMON_METHODS_H

#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//#include <sensor_msgs/JointState.h>

class common_methods
{
private:
  // Variables para

  // Metodo velocidades
  bool advance_;
  bool reverse_;
  bool turn_left_;
  bool turn_right_;
  float velocity_;
  float velocity_robot_;

  // Metodo calculo velocidades
  void velocity_calcule();
  float phase_one_vel_;
  float phase_two_vel_;
  float total_angle_degrees_ = 45;

#define ROBOT_LEG_NUM 6 // Numero de patas del robot

  // libreria CLegs
  void setup_legs(ros::NodeHandle &nh);  // Inicializacion de las patas
  void delete_legs(); // Eliminacion de las patas


  // ROS
  ros::NodeHandle nh;  //Node handle
  ros::Subscriber velocity_subs;  // Subscriptor al twist_cmd_vel
  ros::Subscriber joystick_subs;  // Subscriptor al /joy

  // Callbacks
  void init_subscribers();  // Metodo que inicia los subscriber
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel); // Callback para el twist del joystick
  void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);  // Callback para los botones del joystick


public:

  // Patas del robot
  CLeg *pata[ROBOT_LEG_NUM];

  common_methods(); // Constructor
//  ~common_methods();  // Destructor

//  std::vector<float> legs_actual_position; // Vector que almacena la posicion actual de cada pata -- su joint

  // Legs readed first time
  bool legs_readed();
  bool legs_readed_flag;

  float total_angle_rads_ = total_angle_degrees_* M_PI / 180;

  bool eStop_;
  bool exit_button_;


  // Getters
  bool advance() const;
  bool reverse() const;
  bool turn_left() const;
  bool turn_right() const;

  float phase_one_vel() const;
  float phase_two_vel() const;


};

#endif // COMMON_METHODS_H
