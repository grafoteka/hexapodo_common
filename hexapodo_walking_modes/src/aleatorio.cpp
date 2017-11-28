//====================================================================================
//  Autor: Raul Cebolla Arroyo
//  Matricula: 13069
//  Archivo: aleatorio.cpp
//  Descripcion: Modo de marcha de prueba para el robot RHex
//====================================================================================


//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <c_leg/c_leg.h>
#include <sensor_msgs/JointState.h>
#include <string>

//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

#define RHEX_LEG_NUM  6             //Numero de patas del robot RHEX
sensor_msgs::JointState patas_joint_state;  //Vector que lee los datos del JointState
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
enum robot_modes { ground, standBy, stop, alternTripod};

// Estados de los tripodes
enum tripods_states { origin, land, flight, transition };
tripods_states tripodOneStateActual = origin;
tripods_states tripodOneStateLast;
tripods_states tripodOneStateNext;


float pos;
bool patas_lectura_flag = false;
bool tripode = false;
bool posicion_origen;
const float leg_diameter = 0.210; // en m -> 210 en mm.
float velocity_ground = 0;
float velocity_flight = 0;

// Angulo de aterrizaje/despegue
float angulo = 15 * M_PI / 180;

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

//------------------------------------------------------------------------------------
//  Funciones auxiliares
//------------------------------------------------------------------------------------

//  Definidas por el usuario

float conversionRadianes(float grados)
{
  float radianes =  grados * PI / 180;
  return radianes;
}

/******************************************************
 * Función que se llama periódicamente para obtener la posición de los joints.
 ******************************************************/
void lectura_posicion_patas(const sensor_msgs::JointStatePtr& msg)
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
 * Función que se llama periódicamente para obtener la posición de los joints.
 ******************************************************/
/*
void callback_timer(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
  for (int i = 0; i < patas_joint_state.position.size(); i++)
  {
      ROS_INFO("peto aqui");
    //ROS_INFO("joint %d name %s", i, patas_joint_state.name.at(i).c_str());
    ROS_INFO("joint %d position  %.2f", i, patas_joint_state.position.at(i));
    patas_pos.push_back(msg->position.at(i));

  }
}
*/


/******************************************************
 * Calculos
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

void calculos(float vRobot, float alpha, float leg_ground_vel, float leg_fly_vel)
{
  float lStep = (2 * PI * alpha * leg_diameter) / 360;
  float totSteps = vRobot / lStep;
  float steps_per_second = 1 / totSteps;
  float alpha_rads = alpha * PI / 180;
  float ground_vel_rads = alpha_rads * (steps_per_second * 100);
  ROS_INFO("velocity ground = %.2f", ground_vel_rads);
  // Calculo de la velocidad de vuelo segun el porcentaje de seguridad
  const float fly_vel_security = 0.9;
  float fly_vel_rads = (2 * PI - alpha_rads) * (steps_per_second * fly_vel_security * 100);
  ROS_INFO("velocity fly = %.2f", fly_vel_rads);
  velocity_ground = ground_vel_rads;
  velocity_flight = fly_vel_rads;

}


/******************************************************
 * Función para poner en pie el robot -> standUp
 ******************************************************/

robot_modes standUp()
{
  //vector con la consigna de posicion de las patas.
  // En este caso, las patas las queremos situar en la posición absoluta 0.0
  std::vector<double> patas_consignas_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//  int posicion_origen_int = posicion_origen; ROS_INFO("posicion origen = %d", posicion_origen_int);
//  int patas_lectura_flag_int = patas_lectura_flag; ROS_INFO("posicion origen = %d", patas_lectura_flag_int);

  // Solo se entra en el bucle si se ha leido, al menos una vez el joint_states y el flag de que el robot este
  // en la posicion de origen no se ha activado.
  if(!posicion_origen && patas_lectura_flag)
  {
//    ROS_INFO("Dentro del while");
    for(int i = 0; i < 6; i++)
    {
      // Si la pata no se encuentra en la posicion 0.0
      // se calcula el error y con ello la velocidad en un controlador P.
      // La velocidad es negativa para que las patas giren en sentido antihorario. Pero es el giro correcto.
      if(patas_posicion_origen.at(i) == false){
        float error = (patas_posicion_actual.at(i) - patas_consignas_vector.at(i));
        float velocity = -2.5 * error;
//        ROS_INFO("joint dentro del while %d position  %.2f", i, patas_posicion_actual[i]);
        pata[i]->setVelocity(velocity);

      if(abs(velocity) < 0.05)
      {
//        ROS_INFO("Estoy en el 0");
//        pata[i]->setPosition(-0.02);
        patas_posicion_origen[i] = true;
        pata[i]->setVelocity(0.0);
      }
}

      if(std::all_of(patas_posicion_origen.begin(), patas_posicion_origen.end(), [](bool posicion_origen) {return posicion_origen;})){
        ROS_INFO("Posicion origen = TRUE");
//        posicion_origen = true;float velocity_ground, velocity_flight;
        return (standBy);
      }
    }
  }
}


/******************************************************
 * Función para mover el primer tripode
 ******************************************************/
std::vector<bool> moveLegsConsignaPatasAlcanzada = {false, false, false};
std::vector<bool> moveLegsTripodOneConsignaAlcanzada = {false, false, false};
std::vector<bool> moveLegsTripodTwoConsignaAlcanzada = {false, false, false};

// Funcion que mueve los tripodes de las patas
bool moveLegs(std::vector<int> tripod_one, std::vector<int> tripod_two, float tripod_one_angulo, float tripod_two_angulo, bool moveLegsConsigna, float tripod_one_velocity, float tripod_two_velocity)
{
  ROS_INFO("MoveLegs");
  if(moveLegsConsigna == false)
  {
    ROS_INFO("Bucle_legs");
    for(int i = 0; i < tripod_one.size(); i++)
    {
      if(moveLegsTripodOneConsignaAlcanzada.at(i) == false)
      {
        pata[tripod_one.at(i)]->setVelocity(tripod_one_velocity);
      }
    }
    for(int i = 0; i < tripod_two.size(); i++)
    {
      if(moveLegsTripodTwoConsignaAlcanzada.at(i) == false)
      {
        pata[tripod_two.at(i)]->setVelocity(tripod_two_velocity);
      }
    }
   }
  else
  {
    for(int i = 0; i < tripod_one.size(); i++)
    {
//      ROS_INFO("%.2f", alpha_rads);
      float distancia;
      float pos_pata_actual = fmod(patas_posicion_actual.at(tripod_one.at(i)), (2*PI));

      distancia = fabs(tripod_one_angulo - pos_pata_actual );
      ROS_INFO("Pata %d -- Con: %.2f  Lectura -> %.2f - distancia %.2f", tripod_one.at(i), tripod_one_angulo, pos_pata_actual, distancia);
      if(distancia <= 0.05)
      {
        pata[tripod_one.at(i)]->setVelocity(0.0);
//        pata[tripod.at(i)]->setPosition(alpha_rads);
        moveLegsTripodOneConsignaAlcanzada.at(i) = true;
        ROS_INFO("Pata %d consigna alcanzada, velocidad = %.2f", tripod_one.at(i), tripod_one_velocity);
      }
     }
    for(int i = 0; i < tripod_two.size(); i++)
    {
//      ROS_INFO("%.2f", alpha_rads);
      float distancia;
      float pos_pata_actual = fmod(patas_posicion_actual.at(tripod_two.at(i)), (2*PI));

      distancia = fabs(tripod_two_angulo - pos_pata_actual );
      ROS_INFO("Pata %d -- Con: %.2f  Lectura -> %.2f - distancia %.2f", tripod_two.at(i), tripod_two_angulo, pos_pata_actual, distancia);
      if(distancia <= 0.05)
      {
        pata[tripod_two.at(i)]->setVelocity(0.0);
//        pata[tripod.at(i)]->setPosition(alpha_rads);
        moveLegsTripodTwoConsignaAlcanzada.at(i) = true;
        ROS_INFO("Pata %d consigna alcanzada, velocidad = %.2f", tripod_two.at(i), tripod_two_velocity);
      }
     }

    }

  bool tripodOneConsignaAlcanzada = std::all_of(moveLegsTripodOneConsignaAlcanzada.begin(), moveLegsTripodOneConsignaAlcanzada.end(), [](bool posicion_origen) {return posicion_origen;});
  bool tripodTwoConsignaAlcanzada = std::all_of(moveLegsTripodTwoConsignaAlcanzada.begin(), moveLegsTripodTwoConsignaAlcanzada.end(), [](bool posicion_origen) {return posicion_origen;});


  if(tripodOneConsignaAlcanzada && tripodTwoConsignaAlcanzada)
    {
      ROS_INFO("TRANSICION");
      //moveLegsConsigna = true;
      for(int i = 0; i < moveLegsTripodOneConsignaAlcanzada.size(); i++)
      {
        moveLegsTripodOneConsignaAlcanzada.at(i) = false;
      }
      for(int i = 0; i < moveLegsTripodTwoConsignaAlcanzada.size(); i++)
      {
        moveLegsTripodTwoConsignaAlcanzada.at(i) = false;
      }
      return true;
    }

    return false;
  }


// MAQUINA DE ESTADOS PARA LOS TRIPODES
bool consignaAlcanzada = false;
bool tripodOneConsignaAlcanzada  = false;
bool tripodTwoConsignaAlcanzada  = false;
std::vector<bool> tripodOnePatasPosicionAlcanzada = {false, false, false};

robot_modes alternTripodWalking(float leg_ground_velocity, float leg_flight_velocity, float alpha)
{
  float alpha_rads = (alpha / 2) * PI / 180.0;
  std::vector<int> tripod_one = {0, 3, 4};
  std::vector<int> tripod_two = {1, 2, 5};
  bool flag = false;

  static float tripod_one_angulo;
  static float tripod_two_angulo;

  switch(tripodOneStateActual)
  {
 // ORIGIN
  case origin:
    // Origin -> FLIGHT
    // Las patas deben de ponerse en +alpha/2
    if(tripodOneStateNext == flight)
    {
      ROS_INFO("Origin To TRANSITION");
//      tripodOneStateActual = transition;
//      tripodOneStateLast = origin;
//      tripodOneStateNext = flight;
      tripod_one_angulo = alpha_rads;
      tripod_two_angulo = 2 * PI - alpha_rads;
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, false, leg_ground_velocity, leg_flight_velocity);

//      if(flag)
//      {
        tripodOneStateActual = transition;
        tripodOneStateLast = origin;
        tripodOneStateNext = flight;
//      }
    }

    // Origin -> LAND
    // Las patas deben de ponerse en 360-alpha/2
    if(tripodOneStateNext == land)
    {
      ROS_INFO("Origin To TRANSITION");
      tripod_one_angulo = 2 * PI - alpha_rads;
      tripod_two_angulo = alpha_rads;
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, false, leg_flight_velocity, leg_ground_velocity);
      tripodOneStateActual = transition;
      tripodOneStateLast = origin;
      tripodOneStateNext = land;

//      moveLegs(tripod_one, angulo, false);
    }
    break;

  // TRANSITION
  case transition:
    // Origin -> LAND
//    flag = moveLegs(tripod_one, tripod_two, angulo, true, leg_flight_velocity, leg_ground_velocity);
    if((tripodOneStateNext == land) && (tripodOneStateLast == origin))
    {
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, true, leg_flight_velocity, leg_ground_velocity);
      if(flag)
      {
        ROS_INFO("Origin To LAND");
        tripodOneStateActual = land;
//      tripodOneStateLast = transition;
//      tripodOneStateNext = land;
      }
    }

    // Origin -> FLIGHT
    if((tripodOneStateNext == flight) && (tripodOneStateLast == origin))
    {
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, true, leg_ground_velocity, leg_flight_velocity);
      if(flag)
      {
        ROS_INFO("Origin To FLIGHT");
        tripodOneStateActual = flight;
        tripodOneStateLast = origin;
        tripodOneStateNext = flight;
      }
    }

    // Flight -> LAND
    if((tripodOneStateNext == land) && (tripodOneStateLast == flight))
    {
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, true, leg_flight_velocity, leg_ground_velocity);
      if(flag)
      {
        ROS_INFO("Tansition To LAND");
        tripodOneStateActual = land;
  //      tripodOneStateLast = flight;
        tripodOneStateNext = flight;
      }
    }

    // Land -> FLIGHT
    if((tripodOneStateNext == flight) && (tripodOneStateLast == land))
    {
      flag = moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, true, leg_ground_velocity, leg_flight_velocity);
      if(flag)
      {
        ROS_INFO("Transition To FLIGHT");
        tripodOneStateActual = flight;
        tripodOneStateLast = land;
  //      tripodOneStateNext = flight;
      }
    }

    break;

  case land:
    //LAND -> transition -> flight
    if((tripodOneStateNext == flight))
    {
//      ROS_INFO("Land To TRANSITION");
      ROS_INFO("LAND");
      tripod_one_angulo = alpha_rads;
      tripod_two_angulo = 2 * PI - alpha_rads;
      moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, false, leg_ground_velocity, leg_flight_velocity);

      tripodOneStateActual = transition;
      tripodOneStateLast = land;
      tripodOneStateNext = flight;

    }
    //flight -> LAND -> flight

    break;

  case flight:
    if((tripodOneStateNext == flight))// && (tripodOneStateLast == origin))
    {
//      ROS_INFO("Flight to TRANSITION");
      ROS_INFO("FLIGHT");
      tripod_one_angulo = 2 * PI - alpha_rads;
      tripod_two_angulo = alpha_rads;
      moveLegs(tripod_one, tripod_two, tripod_one_angulo, tripod_two_angulo, false, leg_flight_velocity, leg_ground_velocity);
      tripodOneStateActual = transition;
      tripodOneStateLast = flight;
      tripodOneStateNext = land;

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
  ros::NodeHandle node_sub;   //Nodo que se suscribe al JointState

  //Suscripción al tópico del JointStates
  ros::Subscriber sub = node_sub.subscribe("/rhex/joint_states", 1, lectura_posicion_patas);

//  ros::Timer timer = node_sub.createTimer(ros::Duration(1.0), callback_timer);

  ros::Rate rate(50.0);

  // Vector que define que las 6 patas no se encuentran en el origen 0.0
  // Esto es para la primera vez que se inicialice el robot, que pase del suelo
  // a la posición de espera.

  patas_joint_state.position.resize(6);
  patas_posicion_origen.resize(6);
  for(int i = 0; i < 6; i++)
  {
    patas_posicion_origen.at(i) = false;
  }

  posicion_origen = false;

  robot_modes robot_modes_actual;
  tripodOneStateNext = flight;

  // Vector que define los estados de la MEF.
  // El primero es la posición de espera. Todas las patas en 0.0
  //estados_mef.resize(1);
  //estados_mef.at(0) = false;


  //Se inicializan las patas
  setupPatas(nh);


  //---------------------------------------------------------------------------------
  //  Rutinas de una sola ejecucion
  //---------------------------------------------------------------------------------

//  ROS_INFO("%d", patas_joint_state.position.size());

  for(int i = 0; i < 6; i++)
  {
    pata[i]->setVelocity(0);
    //patas_pos.at(i) = pata[i]->getPos();
    //ROS_INFO("pata %d -> posicion %.2f", i, pata[i]->getPos());
  }

  float vel_robot = 0.05; float alpha_robot = 60;

  calculos((1/vel_robot), alpha_robot, velocity_ground, velocity_flight);
  /*
  return 0;
  ROS_INFO("Asigno velocidades = 2.5");
  for (int i = 0; i < 6; i++)
  {
      pata[i]->setVelocity(2.5);  //Giro en sentido positivo. Incrementa la posición del encoder
      ROS_INFO("pata %d -> velocidad %.2f -> posicion %.2f", i, pata[i]->getVel(), pata[i]->getPos() );
  }
  */

  //---------------------------------------------------------------------------------
  //  Bucle del programa principal
  //---------------------------------------------------------------------------------

  while(ros::ok()){

    switch (robot_modes_actual) {
      case ground:
        robot_modes_actual = standUp();
        break;

      case standBy:
        ROS_INFO("Estoy en StandBy");
        robot_modes_actual = alternTripod;
        break;

      case stop:

        break;

      case alternTripod:
        ROS_INFO("Estoy en alternTripod");
        tripodOneStateNext;
        alternTripodWalking(velocity_ground, velocity_flight, alpha_robot);
        break;

      default:
//        ROS_INFO("Default");
        robot_modes_actual = ground;
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
