//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  Archivo: tripode_alterno.cpp
//  Descripcion: Modo de marcha en tripode alterno para el robot hexapodo
//====================================================================================


//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <c_leg/c_leg.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <altern_tripod_fem/altern_tripod_fem.h>

//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

#define RHEX_LEG_NUM  6             //Numero de patas del robot RHEX
sensor_msgs::JointState patas_joint_state;  //Vector que lee los datos del JointState
geometry_msgs::Twist new_vel;   //Variable para guardar las velocidades del joystick
//sensor_msgs::JointState

//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

//Patas del robot RHex
CLeg *pata[RHEX_LEG_NUM];

std::vector<int> legs_vector_class;

std::string pata_seleccionada;
// Vector para definir el estado de las patas en la posición de origen 0.0 la primera
// vez debe ser false todos ya que se encuentran en -4.5
std::vector<bool> patas_posicion_origen(6);

// Vector para definir los estados de la MEF
// El primer estado es la posición de espera. El robot con las 6 patas en 0.0
std::vector<bool> estados_mef;

// Vector para la posicion de las patas
std::vector<float> patas_posicion_actual(6);

// Estados de movimiento: avance, giro, giro+avance
enum robot_movement {forward, turn_left, turn_right, forward_left, forward_right, stop};
robot_movement robot_movement_actual;

// Estados del robot
enum robot_modes { ground, stand_by, altern_tripod};
robot_modes robot_modes_actual;
robot_modes robot_modes_last;

// Estados de los tripodes
enum tripods_states { origin, phase_1, phase_2, transition };
tripods_states state_actual = origin;
tripods_states state_last;
tripods_states state_next;


float pos;
bool patas_lectura_flag = false;
bool tripode = false;
bool posicion_origen = false;
const float leg_diameter = 0.210; // en m -> 210 en mm.
float phase_one_velocity = 0.0;
float phase_two_velocity = 0.0;

bool estaticas = false;

float alpha_robot = 60.0; // Angulo total de la fase suelo [degrees]
float robot_speed_init = 0.1; // Velocidad inicial del robot [m/s]
float velocity_robot = 0.0;

// Angulo de aterrizaje/despegue
float angulo = 15 * M_PI / 180;

void calculos(float vRobot, float leg_ground_vel, float leg_fly_vel);

//------------------------------------------------------------------------------------
//  Funciones de configuracion
//------------------------------------------------------------------------------------

//Funcion setup de las patas
void setupPatas (ros::NodeHandle &nh){
  for (int i = 0; i < RHEX_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
  }
  return;
}

//Funcion que libera el espacio en memoria de las patas
void deletePatas (){
  for (int i = 0; i < RHEX_LEG_NUM; i++){
    delete pata[i];
  }
  return;
}


/*********************************************
    CALLBACKS
**********************************************/

/******************************************************
 * Joints
 ******************************************************/
void legs_position_callback(const sensor_msgs::JointStatePtr& msg)
{
  patas_joint_state = *msg;
//  ROS_INFO("Lectura de la posicion de las patas, son %d patas", patas_joint_state.name.size());
//  ROS_INFO("n joints %d", patas_joint_state.name.size());
  for (int i = 0; i < patas_joint_state.position.size(); i++)
  {
//    ROS_INFO("joint %d name %s, position %.2f, effort %.2f", i, patas_joint_state.name.at(i).c_str(), patas_joint_state.position.at(i), patas_joint_state.effort.at(i));
//    ROS_INFO("joint %d position  %.2f", i, patas_joint_state.position.at(i));
    patas_posicion_actual.at(i) = patas_joint_state.position.at(i);
//    patas_posicion_actual.push_back(msg->position.at(i));
//    ROS_INFO("joint %d position  %.2f", i, patas_posicion_actual[i]);
//    std::cout << patas_posicion_actual[i];
    patas_lectura_flag = true;
  }
}

/******************************************************
 * Twist
 ******************************************************/
bool avance = false;
float velocity = 0.1; // m/s
float vel_command;
float linear_old = 0.0;
float angular_old = 0.0;
float angular_actual = 0.0;
float linear_actual = 0.0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel)
{
  //ROS_INFO("Lectura de Joystick");
  //Since vel is a const pointer you cannot edit the values inside but have to use the copy new_vel.
  new_vel = *vel;
  linear_actual = vel->linear.x;
  angular_actual = vel->angular.z;

  if(linear_actual > 0)
    avance = true;
  else
    avance = false;

  if((linear_old != linear_actual) || (angular_old != angular_actual))
  {
//    ROS_INFO("velocidad lineal: %.2f -- velocidad angular: %.2f", linear_actual, angular_actual);
    linear_old = linear_actual;
    angular_old = angular_actual;
  }

}


/******************************************************
 * Joystick.
 ******************************************************/
bool eStop = true;

void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float vel_increase = 0.01;
  const float speed_max = 0.75;
  const float speed_min = 0.0;

  // Si se pulsa triangulo
  if(joy->buttons[3] == 1)
  {
    if(velocity <= speed_max){
      velocity = velocity + vel_increase;
//      ROS_INFO("velocidad en m/s: %.2f", velocity);
    }
  }

  // Si se pulsa cuadrado
  if(joy->buttons[0] == 1)
  {
    if(velocity >= speed_min){
      velocity = velocity - vel_increase;
//      ROS_INFO("velocidad en m/s: %.2f", velocity);
    }
  }

  velocity_robot = velocity;
  if(joy->buttons[6] == 1)
    eStop = false;
  else
    eStop = true;

}



/*robot_movement movimiento(){
  if(angular_actual == 0)
  {
    if(linear_actual > 0)
      return forward;
    else
      return stop;
  }
  else if(angular_actual > 0)
  {
    if(linear_actual > 0)
      return forward_right;
    else
      return turn_right;
  }
  else if(angular_actual < 0)
  {
    if(linear_actual > 0)
      return forward_left;
    else
      return turn_left;
  }
  else
    return stop;
}*/

//####################################################################################
//  Programa principal
//####################################################################################

int main(int argc, char **argv)
{

    //---------------------------------------------------------------------------------
    //  Setup
    //---------------------------------------------------------------------------------

    //Inicio del nodo en ROS y el node handle
    ros::init(argc, argv, "gait_template");
    ros::NodeHandle nh;

    //Suscripción al tópico del JointStates y cmd_vel
    //JointStates entrega el estado de cada articulacion
//    ros::Subscriber sub = nh.subscribe("/hexapodo/joint_states", 1, legs_position_callback);
    //cmd_vel da los valores de velocidad lineal y angular
    ros::Subscriber velocity = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_callback);

    ros::Rate rate(100.0);

    ros::AsyncSpinner spinner(0); // Use 4 threads
    spinner.start();


    // Vector que define que las 6 patas no se encuentran en el origen 0.0
    // Esto es para la primera vez que se inicialice el robot, que pase del suelo
    // a la posición de espera.
//    legs_joint_state.position.resize(6);
//    legs_in_origin.resize(6);
//    for(int i = 0; i < 6; i++)
//    {
//    legs_in_origin.at(i) = false;
//    }



    //Se inicializan las patas
//    setupPatas(nh);

    //---------------------------------------------------------------------------------
    //  Rutinas de una sola ejecucion
    //---------------------------------------------------------------------------------
altern_tripod_fem hexapodo(true, alpha_robot, robot_speed_init);

hexapodo.vel_calcs(velocity_robot);
    //---------------------------------------------------------------------------------
    //  Bucle del programa principal
    //---------------------------------------------------------------------------------

    bool avance_old;
    bool flag_fem;

    while(ros::ok()){

      hexapodo.vel_calcs(velocity_robot);


      /*if(!eStop)
        robot_movement_actual = movimiento();

      switch (robot_modes_actual) {

      case stop:

        break;

      case forward:

        break;

      }*/


    switch (robot_modes_actual) {
      case ground:
        flag_fem = hexapodo.stand_up();
        if(flag_fem)
        {
          robot_modes_actual = stand_by;
          robot_modes_last = ground;
        }
        break;

      case stand_by:
        if(robot_modes_actual != robot_modes_last)
        {
          ROS_INFO("Estoy en Stand By");
          robot_modes_last = stand_by;
        }

        if (avance)
        {
          robot_modes_actual = altern_tripod;
          robot_modes_last = stand_by;
        }
          break;

      case altern_tripod:

        if(robot_modes_actual != robot_modes_last)
        {
          ROS_INFO("Estoy en altern Tripod");
          robot_modes_last = altern_tripod;
        }
          if(avance)
        {
            ROS_INFO("Voy a la maquina de estados con FALSE");
            hexapodo.fem(false);
//            altern_tripod_walking(false);
        }
          else
          {
//            for(int i = 0; i < 1; i++)
            {
              ROS_INFO("Voy a la maquina de estados con TRUE");
              hexapodo.fem(true);
//              altern_tripod_walking(true);
            }
          }

        break;

      default:
        ROS_INFO("Default");
//        if(avance)
//            robot_modes_actual = stand_by;
        break;

    }

    rate.sleep();

//    hexapodo.stand_up();
  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}
