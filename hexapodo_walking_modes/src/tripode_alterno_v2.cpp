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
//sensor_msgs::JointState

//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

//Patas del robot RHex
CLeg *pata[RHEX_LEG_NUM];

std::string pata_seleccionada;
// Vector para definir el estado de las patas en la posición de origen 0.0 la primera
// vez debe ser false todos ya que se encuentran en -4.5
std::vector<bool> patas_posicion_origen(6);

// Vector para definir los estados de la MEF
// El primer estado es la posición de espera. El robot con las 6 patas en 0.0
std::vector<bool> estados_mef;

// Vector para la posicion de las patas
std::vector<float> patas_posicion_actual(6);

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

  float alpha_robot = 60.0;

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

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel)
{
  //ROS_INFO("Lectura de Joystick");
  //Since vel is a const pointer you cannot edit the values inside but have to use the copy new_vel.
  new_vel = *vel;
  float linear_actual = vel->linear.x;
  float angular_actual = vel->angular.z;

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

  calculos((1/velocity), phase_one_velocity, phase_two_velocity);
}


//------------------------------------------------------------------------------------
//  Funciones auxiliares
//------------------------------------------------------------------------------------


/******************************************************
 * Calculos de las velocidades
 ******************************************************/

/*
 * Funcion para el calculo de los pasos por segundo segun una velocidad
 * de avance del cuerpo del robot
 *
 * Como parametros son necesarios la velocidad del cuerpo del robot y
 * el angulo de total en la fase del suelo.
 *
 * vrobot (m/s) = TotSteps * Lstep(alpha)
 *
 * */

float ground_vel_rads_old;
float fly_vel_rads_old;

void calculos(float vRobot, float leg_ground_vel, float leg_fly_vel)
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
  }

}


/******************************************************
 * Función para poner en pie el robot -> standUp
 ******************************************************/

robot_modes stand_up(std::vector<double> patas_consignas_vector)
{
  ROS_INFO("Stand UP");
  //vector con la consigna de posicion de las patas.
  // En este caso, las patas las queremos situar en la posición absoluta 0.0
  //std::vector<double> patas_consignas_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


  // Solo se entra en el bucle si se ha leido, al menos una vez el joint_states y el flag de que el robot este
  // en la posicion de origen no se ha activado.

  //  int posicion_origen_int = posicion_origen; //ROS_INFO("posicion origen = %d", posicion_origen_int);
  //  int patas_lectura_flag_int = patas_lectura_flag; ROS_INFO("posicion origen = %d -- patas_lectura_flag = %d", posicion_origen_int, patas_lectura_flag_int);

  if(!posicion_origen && patas_lectura_flag)
  {
    // Aqui vamos a la clase con la funcion stand_up
    // esta funcion devuelve un booleano que indica si se ha levantado el robot o no
    bool flag = stand_up(patas_consignas_vector, patas_posicion_actual);

    if(flag)
      return (stand_by);
    else
      return(origin); //este estado es el original del programa, por lo que volvería a entrar en esta funcion desde el main;
   }
}


/*********************************************
    MOVIMIENTO DE LAS PATAS
**********************************************/

// Variables para contar las fases y sin son pares o impares
int phase = 0;
float phase_mod = 0;

bool move_legs(bool start_movement)
{
  // Llamada a la funcion de la clases que mueve las patas
  bool flag = stand_up(start_movement*);

  return flag;

}


/*********************************************
    MAQUINA DE ESTADOS DEL TRIPODE ALTERNO
**********************************************/

bool altern_tripod_walking(bool exit)
{
    bool flag;

    switch(state_actual)
    {
        case origin:
            if(!exit)
            {
              ROS_INFO("Origin");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_1;
                state_last = origin;
            }

        break;

        case phase_1:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_1");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_2;
                state_last = phase_1;
                ROS_INFO("Move to TRANSITION");
            }
            if(exit)
            {
              ROS_INFO("phase_1 exit");
              for(int i = 0; i < tripods_quantity; i++)
              {
                  for(int j = 0; j < tripods_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_legs_configuration[0][j]]->setPosition(take_off_angle);
                    }
                    else
                      pata[robot_legs_configuration[1][j]]->setPosition(take_land_angle);
                  }
              }
            }

        break;

        case phase_2:
            if(!exit && state_next == transition)
            {
              ROS_INFO("phase_2");
                flag = move_legs(true);
                state_actual = transition;
                state_next = phase_1;
                state_last = phase_2;
            }
            if(exit)
            {
              for(int i = 0; i < tripods_quantity; i++)
              {
                  for(int j = 0; j < tripods_length; j++)
                  {
                    if(i == 0)
                    {
                      pata[robot_legs_configuration[0][j]]->setPosition(take_land_angle);
                    }
                    else
                      pata[robot_legs_configuration[1][j]]->setPosition(take_off_angle);
                  }
              }
            }

        break;

        case transition:
            if(state_last == origin && state_next == phase_1){
                ROS_INFO("transition from origin");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from origin");
                    state_actual = phase_1;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_1 && state_next == phase_2){
              ROS_INFO("transition from phase 1");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_2;
                    state_next = transition;
                    state_last = transition;
                }
            }

            if(state_last == phase_2 && state_next == phase_1){
              ROS_INFO("transition from phase 2");
                flag = move_legs(false);
                if(flag)
                {
                  ROS_INFO("FLAG TRUE transition from phase 1");
                    state_actual = phase_1;
                    state_next = transition;
                    state_last = transition;
                }
            }


        break;

    }
}


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
    ros::Subscriber sub = nh.subscribe("/hexapodo/joint_states", 1, legs_position_callback);
    //cmd_vel da los valores de velocidad lineal y angular
    ros::Subscriber velocity = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber joystick = nh.subscribe("/joy", 1, joystick_callback);

    ros::Rate rate(100.0);

    // Vector que define que las 6 patas no se encuentran en el origen 0.0
    // Esto es para la primera vez que se inicialice el robot, que pase del suelo
    // a la posición de espera.
//    legs_joint_state.position.resize(6);
//    legs_in_origin.resize(6);
//    for(int i = 0; i < 6; i++)
//    {
//    legs_in_origin.at(i) = false;
//    }

    state_next = phase_1;

    //Se inicializan las patas
    setupPatas(nh);

    //---------------------------------------------------------------------------------
    //  Rutinas de una sola ejecucion
    //---------------------------------------------------------------------------------


    //---------------------------------------------------------------------------------
    //  Bucle del programa principal
    //---------------------------------------------------------------------------------

    bool avance_old;

    while(ros::ok()){

    switch (robot_modes_actual) {
      case ground:
        robot_modes_actual = stand_up({-0.260, 0.26, 0.260, -0.260, -0.260, 0.260});
        robot_modes_last = ground;
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
            altern_tripod_walking(false);
        }
          else
          {
            for(int i = 0; i < 1; i++)
            {
              altern_tripod_walking(true);
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
    ros::spinOnce();
  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}
