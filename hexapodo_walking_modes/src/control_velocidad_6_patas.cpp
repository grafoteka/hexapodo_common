
//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <c_leg/c_leg.h>

#define _USE_MATH_DEFINES // for C++
#include <cmath>
//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

#define RHEX_LEG_NUM  6             //Numero de patas del robot RHEX
sensor_msgs::JointState patas_joint_state;  //Vector que lee los datos del JointState
//sensor_msgs::JointState
std::vector<float> patas_posicion_actual(6);
bool patas_lectura_flag = false;

void joint_states_callback(const sensor_msgs::JointStatePtr& msg)
{
  patas_joint_state = *msg;
  for (int i = 0; i < patas_joint_state.position.size(); i++)
  {
    patas_posicion_actual.at(i) = patas_joint_state.position.at(i);
    patas_lectura_flag = true;
    //ROS_INFO("Pata %d posicion %.2f", i, patas_posicion_actual.at(i));
  }
}
//------------------------------------------------------------------------------------
//  Variables globales
//------------------------------------------------------------------------------------

//Patas del robot RHex
CLeg *pata[RHEX_LEG_NUM];

float phase_one_vel = 0.0;
float phase_two_vel = 0.0;

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

/******************************************************
 * Calculo de velocidades
 ******************************************************/
void velocity_calculate()
{
  const float velocity_robot_ = 0.075;
  const float leg_diameter = 0.200;
  const float total_angle_degrees_ = 45.0;
  float alpha = total_angle_degrees_;
  float lStep = (2 * PI * alpha * leg_diameter) / 360;
  float totSteps = (1/velocity_robot_) / lStep;
  float steps_per_second = 1 / totSteps;
  float alpha_rads = alpha * PI / 180;
  //static float phase_one_vel = 0.0;
  static float phase_one_vel_old = 0.0;
  //static float phase_two_vel = 0.0;
  static float phase_two_vel_old = 0.0;

  phase_one_vel = alpha_rads * (steps_per_second * 100);
//    ROS_INFO("phase one velocity = %.2f, robot speed = %.2f", phase_one_vel, robot_speed);

    // Calculo de la velocidad de vuelo segun el porcentaje de seguridad
  const float phase_two_vel_security = 0.9;
  phase_two_vel = (2 * PI - alpha_rads) * (steps_per_second * phase_two_vel_security * 100);
  //  phase_one_vel = ground_vel_rads;
  //  phase_two_vel = fly_vel_rads;
  if ((phase_one_vel != phase_one_vel_old) || (phase_two_vel != phase_two_vel_old))
  {
    phase_one_vel_old = phase_one_vel;
    phase_two_vel_old = phase_two_vel;
    ROS_INFO("\n velocidad = %.2f \n phase 1 velocity = %.2f  \n phase 2 velocity = %.2f", velocity_robot_, phase_one_vel, phase_two_vel);
  }
}

/******************************************************
 * Movimiento de las patas
 ******************************************************/

bool move_legs(bool start_movement)
{

  const int tripods_quantity = 2;
  const int tripods_length = 3;

  std::vector<int> tripod_one = {0, 3, 4};
  std::vector<int> tripod_two = {1, 2, 5};
  int robot_configuration[tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)},
                                                               {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

  static std::vector<bool> leg_in_position = {false, false, false, false, false, false};

  //int robot_configuration[groups_quantity][groups_length] = {{tripod_one.at(0)}, {tripod_one.at(1)}};
  // Comprobacion de lo que llega
  static bool start_movement_old = false;
  int start_movement_int = start_movement;
  int start_movement_old_int = start_movement_old;

  const float take_land_angle = 5.89;
  const float take_off_angle = 0.39;
  //ROS_INFO("Start movement = %d -- Start movement old = %d", start_movement_int, start_movement_old_int);

  static int phase = 0;
  static float phase_module = fmod(phase, 2);

  if(start_movement != start_movement_old)
    {
      //ROS_INFO("START_MOVEMENT");
        /* SET VELOCITIES */
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                if (leg_in_position.at(robot_configuration[i][j]) == false)
                //if(leg_in_position.at(0) == false)
                {
                    if(phase_module == 0)
                    {
                        //Si estamos en el primer ciclo, el tripode 1 va por el suelo y el tripode 2 por el aire
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[0][j], phase_one_vel);
//                      ROS_INFO("pata %d - velocidad %.2f", robot_configuration[1][j], phase_two_vel);
                      if(phase == 0)
                      {
                        ROS_INFO("Fase inicial");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
                      }

                      else{
                        ROS_INFO("Fase 1 -- Asignacion velocidades");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel);
                      }
                    }
                    else
                    {
                        ROS_INFO("Fase 2 -- Asignacion velocidades");
                        pata[robot_configuration[0][j]]->setVelocity(phase_two_vel);
                        pata[robot_configuration[1][j]]->setVelocity(phase_one_vel);
                    }
                }
            }
        }
            start_movement_old = start_movement;
  }

//        ROS_INFO("Fase = %.2f -- Velocidad fase 1 = %.2f -- Velocidad fase 2 = %.2f", phase_module, phase_one_vel, phase_two_vel);


    /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
    else
    {
      //ROS_INFO("ELSE");
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                //float tripod_one_angle, tripod_two_angle;
                float leg_actual_pos = fmod(patas_posicion_actual.at(robot_configuration[i][j]), 2 * M_PI);
                float angle;
//                ROS_INFO("phase module = %d", phase_module);
                if(phase_module == 0)
                {
                    if(i == 0)
                        angle = take_off_angle; //Angulo para tripode_1 y phase_0

                    else //(i == 1)
                        angle = take_land_angle;    //Angulo para el tripode_1 y phase_1
//                    ROS_INFO("angulo %.2f", angle);
                }
                else
                {
                    if(i == 0)
                        angle = take_land_angle;
                    else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};
                        angle = take_off_angle;
                }
                float distance_old = 0.0;
                float distance = fabs(angle - leg_actual_pos);
                //if(i == 0 && j == 0 && (distance != distance_old)){
                //ROS_INFO("Distancia de la pata %d = %.2f", i, distance);
                   //distance_old = distance;
                //}
                //ROS_INFO("Fase %.2f -- Pata 0 -- Posicion %.2f -- Error %.2f -- Velocidad %.2f", phase_module, leg_actual_pos, distance, pata[0]->getVel());
                if(distance < 0.1 && !leg_in_position.at(3*i+j))
                {
                  ROS_INFO("Distancia correcta para la pata %d", robot_configuration[i][j]);
                  //leg_in_position.at(robot_configuration[i][j]) = true;
                  pata[robot_configuration[i][j]]->setPosition(leg_actual_pos);

                    // Escribimos en el vector de posiciones que esa pata tambien esta en posicion
                    if(i == 0)
                        leg_in_position.at(j) = true;
                    else
                        leg_in_position.at(j+3) = true;
                }
            }
        }
  }

//        ROS_INFO("FIN de un un ciclo");


    /* COMPROBACION DE QUE LOS TRIPODES ESTAN EN LA POSICION DESEADA */
    // Si estan todas las patas en posicion, entonces se resetea el estado de sus valores
    // Y tambien se devuelve el estado de la variable FLAG de la maquina de estados
    bool position_reached = std::all_of(leg_in_position.begin(), leg_in_position.end(), [](bool reached) {return reached;});

    if(position_reached)
    //if(leg_in_position[0] && leg_in_position[1])
//    if(leg_in_position[1][0] && leg_in_position[1][1] && leg_in_position[1][2])
    {
      ROS_INFO("Se ha alcanzado la consigna de todas las patas");
       for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                //leg_in_position[0] = false;
                //leg_in_position[1] = false;
                //pata[0]->setVelocity(0.001);
                //pata[1]->setVelocity(0.001);
                if(i == 0)
                    leg_in_position.at(j) = false;
                else
                    leg_in_position.at(j+3) = false;
        }
      }
        phase++;
        phase_module = phase % 2;
        //ROS_INFO("phase = %d -- module = %.2f", phase, phase_module);
        start_movement_old = false;
        return true;
    }
    else
        return false;

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
  ros::init(argc, argv, "hexapodo_1_pata_velocidad");
  ros::NodeHandle nh;

  ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1, joint_states_callback);
  
  ros::Rate rate(100.0);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  //Se inicializan las patas
  setupPatas(nh);
  
  enum leg_state {origin, phase_one, phase_two, transition};
  leg_state state_actual = origin;
  leg_state state_last;
  leg_state state_next;

  velocity_calculate();

  while(ros::ok()){
    /******************************************************
     * Maquina de estados
     ******************************************************/

    static bool flag;

    switch(state_actual)
    {
      case origin:
      {
        ROS_INFO("Origin");
        flag = move_legs(true);
        state_actual = transition;
        state_next = phase_one;
        state_last = origin;
      }
      break;

      case phase_one:
        if(state_next == transition)
        {
          ROS_INFO("phase_1");
          flag = move_legs(true);
          state_actual = transition;
          state_next = phase_two;
          state_last = phase_one;
          ROS_INFO("Move to TRANSITION");
        }
      break;

      case phase_two:
        if(state_next == transition)
        {
          ROS_INFO("phase_2");
          flag = move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = phase_two;
          ROS_INFO("Move to TRANSITION");
        }
      break;

      case transition:
        if(state_last == origin && state_next == phase_one){
          static int i = 0;
          if(i == 0){
            ROS_INFO("transition from origin");
            i = 1;
          }
          flag = move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from origin");
            state_actual = phase_one;
            state_next = transition;
            state_last = transition;
            i = 0;
          }
        }

        if(state_last == phase_one && state_next == phase_two){
          static int i = 0;
          if(i == 0){
            ROS_INFO("TRANSITION from phase 1");
            i = 1;
          }
          flag = move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 1");
            state_actual = phase_two;
            state_next = transition;
            state_last = transition;
            i = 0;
          }
        }

        if(state_last == phase_two && state_next == phase_one){
          static int i = 0;
          if(i == 0){
            ROS_INFO("transition from phase 2");
            i = 1;
          }
          flag = move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 1");
            state_actual = phase_one;
            state_next = transition;
            state_last = transition;
            i = 0;
          }
        }
        break;
        }
    }


      ros::spinOnce();
  }
