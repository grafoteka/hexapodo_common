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

//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

#define RHEX_LEG_NUM  6             //Numero de patas del robot RHEX
sensor_msgs::JointState patas_joint_state;  //Vector que lee los datos del JointState
geometry_msgs::Twist new_vel;   //Variable para guardar las velocidades del joystick

//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

//Patas del robot RHex
CLeg *pata[RHEX_LEG_NUM];



//------------------------------------------------------------------------------------
//  Funciones de configuracion
//------------------------------------------------------------------------------------

//Funcion setup de las patas
// Esta funcion se ejecuta nada mas lanzar el nodo.
// Configura el numero de patas del robot
void setupPatas (ros::NodeHandle &nh){
  for (int i = 0; i < RHEX_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
  }
  return;
}

//Funcion que libera el espacio en memoria de las patas
// Esta funcion se ejecuta al terminar el nodo
// Es necesaria por si se lanza de nuevo un nodo, que las patas se puedan volver a denominar de la misma manera
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
 *
 * Callback para leer las posicion de los motores.
 * La posicion de las patas se almacena en el vector patas_posicion_actual.
 * El flag patas_lectura_flag es una bandera para indicar que la posicion
 * de las patas se ha leido, al menos, una vez.
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
 *
 * Callback que lee el Twist del Joystick.
 * Se lee el eje X y el eje Y del joystick segun los valores se decide si
 * se avanza, se gira o ambos.
 ******************************************************/
float velocity = 0.1; // [m/s] Velocidad inicial del robot por defecto
bool avance = false; bool retroceso = false;
bool right = false; bool left = false;
float vel_command;
float linear_old = 0.0;
float angular_old = 0.0;
float angular_actual = 0.0;
float linear_actual = 0.0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel)
{
  // Velocidad maxima del robot segun el numero de Froude.
  // Para la pata de 200mm es 1.4  [m/s]
  // Para la pata de 150mm es 1.21 [m/s]
  const float velocity_max = 1.21;
  const float velocity_min = - velocity_max;
  //ROS_INFO("Lectura de Joystick");
  //Since vel is a const pointer you cannot edit the values inside but have to use the copy new_vel.
  new_vel = *vel;
  linear_actual = vel->linear.x;
  angular_actual = vel->angular.z;

  // Avance o retroceso
  if(linear_actual == 0)
  {
    avance = false;
    retroceso = false;
  }
  else if(linear_actual > 0){
    avance = true;
    retroceso = false;
  }
  else {
    avance = false;
    retroceso = true;
  }

  // Izquierda o derecha
  if(angular_actual == 0)
  {
    right = false;
    left = false;
  }
  else if(angular_actual > 0){
    right = true;
    left = false;
  }
  else {
    right = false;
    left = true;
  }

  if((linear_old != linear_actual) || (angular_old != angular_actual))
  {
//    ROS_INFO("velocidad lineal: %.2f -- velocidad angular: %.2f", linear_actual, angular_actual);
    linear_old = linear_actual;
    angular_old = angular_actual;
  }

}

/******************************************************
 * Joystick.
 *
 * Callback que lee si se pulsa algun boton del joystick
 * El boton triangulo aumenta la velocidad del robot
 * El boton cuadrado disminuye la velocidad del robot
 * El boton R1 es el hombre muerto
 ******************************************************/
bool eStop = true;  //Probar a meterla como static

void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // static bool eStop = true; // <-- Probar esto

  // Velocidad maxima del robot segun el numero de Froude.
  // Para la pata de 200mm es 1.4  [m/s]
  // Para la pata de 150mm es 1.21 [m/s]
  const float speed_max = 1.21;
  const float speed_min = - speed_max;
  float vel_increase = 0.01;
  //const float speed_max = 0.75;
  //const float speed_min = 0.0;

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

  // Si se pulsa R1
  if(joy->buttons[6] == 1)
    eStop = false;
  else
    eStop = true;

  calculos((1/velocity), phase_one_velocity, phase_two_velocity);
}

//------------------------------------------------------------------------------------
//  Funciones auxiliares
//------------------------------------------------------------------------------------


/******************************************************
 * Calculos de las velocidades
 * Funcion para el calculo de los pasos por segundo segun una velocidad
 * de avance del cuerpo del robot
 *
 * Como parametros son necesarios la velocidad del cuerpo del robot y
 * el angulo de total en la fase del suelo.
 *
 * vrobot [m/s] = TotSteps * Lstep(alpha)
 ******************************************************/
float ground_vel_rads_old;
float fly_vel_rads_old;

// Esta funcion la debe de coger de hexapod_common_methods
/*void calculos(float vRobot, float leg_ground_vel, float leg_fly_vel)
{
  const float alpha = 45;
  float lStep = (2 * PI * alpha * leg_diameter) / 360;
  float totSteps = vRobot / lStep;
  float steps_per_second = 1 / totSteps;
  float alpha_rads = alpha * PI / 180;
  phase_one_velocity = alpha_rads * (steps_per_second * 100);

  // Calculo de la velocidad de vuelo segun el porcentaje de seguridad
  const float fly_vel_security = 0.9;
  phase_two_velocity = (2 * PI - alpha_rads) * (steps_per_second * fly_vel_security * 100);
//  phase_one_velocity = ground_vel_rads;
//  phase_two_velocity = fly_vel_rads;
  if ((phase_one_velocity != ground_vel_rads_old) || (phase_two_velocity != fly_vel_rads_old))
  {
    ground_vel_rads_old = phase_one_velocity;
    fly_vel_rads_old = phase_two_velocity;
    ROS_INFO("phase 1 velocity = %.2f  --- phase 2 velocity = %.2f", phase_one_velocity, phase_two_velocity);
    ROS_INFO("CULO");
  }

}*/
