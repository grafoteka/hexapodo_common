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

float pos;
bool posicion_origen = false;
bool patas_lectura_flag = false;
bool tripode = false;

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
 * Función para situar al robot en la posición de espera.
 * Todas las patas en la posición 0.0
 ******************************************************/
/*
void posicionEspera()
{

  for (int i = 0; i < 6; i++)
  {
    patas_pos.at(i) = pata[i]->getPos();
    if(pata_origen_flag.at(i) == false)
    {
      pata[i]->setVelocity(2.5);  //Giro en sentido positivo. Incrementa la posición del encoder
      ROS_INFO("pata %d -> velocidad %.2f -> posicion %.2f", i, pata[i]->getVel(), pata[i]->getPos() );
    }
    if ((patas_pos.at(i) > (-0.1)) && (pata_origen_flag.at(i) == false))
    {
      ROS_INFO("Entrando");
      pata[i]->setPosition(0.0);
      pata_origen_flag.at(i) == true;
    }
  }

  for (int i = 0; i < 6; i++)
  {

  }

  // Comprobar si todos los elementos del vector son TRUE

  /*if (std::all_of(std::begin(pata_origen_flag),
      std::end(pata_origen_flag),
             [](bool i)
              {
                  ROS_INFO("%d", i);
                estados_mef.at(0) = i;
                return i; // or return !i ;
              })) {
          std::cout << "All numbers are true\n";
      }*/


//  if((pata_origen_flag.at(0) == true) && (pata_origen_flag.at(1) == true) && (pata_origen_flag.at(2) == true) && (pata_origen_flag.at(3) == true) && (pata_origen_flag.at(4) == true) && (pata_origen_flag.at(5) == true))
//  {
//    ROS_INFO("Esto es para salir");
//    estados_mef.at(0) = true;
//  }
//  /*if (std::find(begin(pata_origen_flag), end(pata_origen_flag), true) == end(pata_origen_flag))
//  {
//    estados_mef.at(0) = true;
//    ROS_INFO("saliendo");
//  }*/

//}


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


/*
void movimiento_pata_1(int num_pata){
    pos = pata[num_pata]->getPos();
    float vel = 2.0 * pos;
    ROS_INFO("He entrado en la funcion de la pata");
    while(pos < -0.1)
    {
        pata[num_pata]->setVelocity(vel);
        vel = 2.0; // * pos;
        pata[num_pata]->setVelocity(vel);
        pos = pata[num_pata]->getPos();
        ROS_INFO("Posicion de la pata 1 es %.2f y la velocidad es %.2f", pos, vel);
//        ROS_INFO("caca");

//        if(pos > -0.1 )
//        {
//            break;
//        }
    }
    ROS_INFO("He salido");
    pata[num_pata]->setPosition(pos);
    pata_origen_flag.at(num_pata) = true;


  return;
}
*/

/******************************************************
 * Función para poner en pie el robot
 ******************************************************/

bool enPie(std::vector<double> patas_consignas_vector)
{
  if(!posicion_origen && patas_lectura_flag)
  {
    ROS_INFO("Dentro del while");
    for(int i = 0; i < 6; i++)
    {
      if(patas_posicion_origen.at(i) == false){
        float velocity = (patas_posicion_actual.at(i));
        float error = (patas_posicion_actual.at(i) - patas_consignas_vector.at(i));
        ROS_INFO("joint dentro del while %d position  %.2f", i, patas_posicion_actual[i]);
        pata[i]->setVelocity(-2.5 * velocity);

      if(abs(velocity) < 0.05)
      {
//        ROS_INFO("Estoy en el 0");
//        pata[i]->setPosition(-0.02);
        patas_posicion_origen[i] = true;
        pata[i]->setVelocity(0.0);
      }
}
      if((patas_posicion_origen.at(0) == true) && (patas_posicion_origen.at(1) == true) && (patas_posicion_origen.at(2) == true) &&
         (patas_posicion_origen.at(3) == true) && (patas_posicion_origen.at(4) == true) && (patas_posicion_origen.at(5) == true))
      {
        ROS_INFO("Posicion origen = TRUE");
        posicion_origen = true;
      }

    }
  }
  ROS_INFO("salgo de enPie");
  return posicion_origen;
}


/******************************************************
 * Función para mover el primer tripode
 ******************************************************/

bool consignaAlcanzada = false;

bool tripodeAlterno(std::vector<double> patas_consignas_vector, std::vector<bool> patas_consigna_alcanzada)
{
  if(!consignaAlcanzada)
  {
    ROS_INFO("Dentro de tripodeAlterno");
    for(int i = 0; i < patas_consigna_alcanzada.size(); i++)
    {
      if(patas_consigna_alcanzada.at(i) == false){
        int consigna_alcanzada = patas_consigna_alcanzada.at(i);
        float error = (patas_posicion_actual.at(i) - patas_consignas_vector.at(i));
        float velocity = -2.5 * (error);
        ROS_INFO("pata %d consigna %d", i, consigna_alcanzada);
        ROS_INFO("joint dentro del tripode alterno %d position  %.2f velocidad %.2f, error %.2f", i, patas_posicion_actual[i], velocity, error);
        pata[i]->setVelocity(velocity);

        if(abs(velocity) < 0.05)
        {
  //        ROS_INFO("Estoy en el 0");
  //        pata[i]->setPosition(-0.02);
          ROS_INFO("Se cumple la condicion");
          patas_consigna_alcanzada[i] = true;
          pata[i]->setVelocity(0.0);
        }
      }
    }
    if((patas_consigna_alcanzada.at(0) == true) && (patas_consigna_alcanzada.at(1) == true) && (patas_consigna_alcanzada.at(2) == true) &&
       (patas_consigna_alcanzada.at(3) == true) && (patas_consigna_alcanzada.at(4) == true) && (patas_consigna_alcanzada.at(5) == true))
    {
      ROS_INFO("Consigna alcanzada = TRUE");
      consignaAlcanzada = true;
      tripode = true;
    }
  }

  return consignaAlcanzada;
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

  ros::Rate rate(10.0);

  // Vector que define que las 6 patas no se encuentran en el origen 0.0
  // Esto es para la primera vez que se inicialice el robot, que pase del suelo
  // a la posición de espera.

  patas_joint_state.position.resize(6);
  patas_posicion_origen.resize(6);
  for(int i = 0; i < 6; i++)
  {
    patas_posicion_origen.at(i) = false;
  }

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
    pata[i]->setVelocity(1.3);
    //patas_pos.at(i) = pata[i]->getPos();
    //ROS_INFO("pata %d -> posicion %.2f", i, pata[i]->getPos());
  }


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

    //
    // Rutina que levanta al robot del suelo
    //
//     if(posicion_origen == false)
//     {
//       enPie({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//       ROS_INFO("Estoy en pie");
//     }

//     if((posicion_origen == true) && (tripode == false) && (consignaAlcanzada == false))
//     {
// //      ROS_INFO("dentro del if");
//       float consigna = conversionRadianes(-90.0);
//       for(int i = 0; i < 6; i++)
//       {
//         patas_posicion_origen.at(i) == true;
//       }
//       std::vector<double> patas_consigna = {0.0, 0.0, 0.0, consigna, 0.0, 0.0};
//       std::vector<bool> patas_consigna_estado = {false, false, false, false, false, false};
//       ROS_INFO("Consignas %.2f %.2f %.2f %.2f %.2f %.2f", patas_posicion_actual.at(0), patas_posicion_actual.at(1), patas_posicion_actual.at(2), consigna, patas_posicion_actual.at(4), patas_posicion_actual.at(5));
//       tripodeAlterno(patas_consigna, patas_consigna_estado);
//     }

// //      ROS_INFO("dentro del while");
// //      ROS_INFO("%.2f", patas_joint_state.position.at(0));


// //    ROS_INFO("%.2f", patas_posicion_actual.at(0));

// //    if(!posicion_origen)
// //    {
// //      enPie();
// //    }


//     ROS_INFO("Main");
//      if((-0.0) <= patas_posicion_actual.at(i))
//      {
//        ROS_INFO("Estoy en el top");
//        pata[i]->setVelocity(-1 * patas_posicion_actual.at(i));
//      }



//    if((pata_origen_flag.at(0) == false) && (pata_origen_flag.at(1) == false) && (pata_origen_flag.at(2) == false) && (pata_origen_flag.at(3) == false) && (pata_origen_flag.at(4) == false) && (pata_origen_flag.at(5) == false))
//    {
//      ROS_INFO("Bucle de movimiento");
//      for (int i = 0; i < 6; i++)
//      {
//          patas_pos.at(i) = pata[i]->getPos();
//          ROS_INFO("pata %d -> velocidad %.2f -> posicion %.2f", i, pata[i]->getVel(), pata[i]->getPos() );
//          if (((fmod(pata[i]->getPos(), M_PI) > (-angulo)) && (fmod(pata[i]->getPos(), M_PI) < (angulo))) && (pata_origen_flag.at(i) == false))
//          {
//            // ROS_INFO("Cambiando el estado de la pata");
//            pata[i]->setVelocity(0.0);
//            pata[i]->setPosition(pata[i]->getPos());
//            pata_origen_flag.at(i) = true;

//            int x = pata_origen_flag.at(i);
//            ROS_INFO("pata %d -> estado %d", i, x);
//          }

//      }
//    }

//    for(int i = 0; i < 6; i++){
//      int x = pata_origen_flag.at(i);
//      ROS_INFO("pata %d -> posicion %.2f -> estado %d", i, pata[i]->getPos(), x);
//    }
    //ROS_INFO("Estoy en el main");

//    if(estados_mef.at(0) == false){
//      for (int i = 0; i < 6; i++)
//      {

//        patas_pos.at(i) = pata[i]->getPos();
//        ROS_INFO("pata %d -> velocidad %.2f -> posicion %.2f", i, pata[i]->getVel(), pata[i]->getPos() );

//      }
//      posicionEspera();
//    }




/*
//      ROS_INFO("hola");
    //pos = pata[1]->getPos();
    // Comprobamos que la pata 1 se encuentra en el origen
    //if((pata[1]->getPos() < 0.1) && (pata[1]->getPos() > -0.1)) // fmod(pata[1]->getPos(), 2*M_PI)))

//    if(false)
//    {
////      pata[1]->setVelocity(0);
////      pata[1]->setPosition(0.00);
//      ROS_INFO("Estoy en posicion");
//    }
*/
/*
    for(int i = 0; i < 6; i++){
    //Si la pata no se encuentra en posición
      if((pata[i]->getPos() < -0.1) && (!pata_origen_flag.at(i))) // fmod(pata[1]->getPos(), 2*M_PI)))
      {
          //Llamando a la pata 1
          ROS_INFO("Llamando a la pata %d", i);
          movimiento_pata_1(i);
      }
    }

    /*
//    if((pata[2]->getPos() < -0.1) && (!pata_2_origen) && (pata_1_origen))
//    {
//        ROS_INFO("Llamando a la pata 2");
//        movimiento_pata_1(2);
//    }
*/

    //ROS_INFO("Estoy en el main");


/*
//    if((pata[1]->getPos() != 0.0) || (pata[1]->getPos() != fmod(pata[1]->getPos(),(2*M_PI))))
//    {
//      movimiento_pata_1();
//    }
//    if(pata[1]->getPos() < (-4.5))
//    {
//      movimiento_pata_1();
//    }
//    else {
//      pata[1]->setPosition(patas_joint_state.position.at(0));
//      ROS_INFO("La posición de la pata es: %.2f", patas_joint_state.position.at(0));
//    }
    //ROS_INFO("Probando");
    //lectura_posicion_patas();
    */
    rate.sleep();
    ros::spinOnce();
  }
  //---------------------------------------------------------------------------------
  //  Rutinas de salida
  //---------------------------------------------------------------------------------

  //deletePatas();


  return 0;

}
