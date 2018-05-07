
//------------------------------------------------------------------------------------
//  Includes
//------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <c_leg/c_leg.h>
#include <geometry_msgs/Twist.h>
#include <stand_by/stand_by.h>
#include <common_methods/hexapod_common_methods.h>

#define _USE_MATH_DEFINES // for C++
#include <cmath>
//------------------------------------------------------------------------------------
//  Defines
//------------------------------------------------------------------------------------

#define RHEX_LEG_NUM  6             //Numero de patas del robot RHEX
sensor_msgs::JointState patas_joint_state;  //Vector que lee los datos del JointState
geometry_msgs::Twist robot_vel;
//sensor_msgs::JointState
std::vector<float> patas_posicion_actual(6);
bool patas_lectura_flag = false;

std::vector<float> patas_valor_actual(6);

float robot_vel_linear;
float robot_vel_angular;

int phase;
float phase_module;

enum leg_state {origin, phase_one, phase_two, transition, transition_turn, phase_one_turn, transition_turn_right, transition_turn_left};
leg_state state_actual = origin;
leg_state state_last;
leg_state state_next;

//------------------------------------------------------------------------------------
//  Callbacks
//------------------------------------------------------------------------------------

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

void cmd_vel_callback(geometry_msgs::Twist vel) {
  //Since vel is a const pointer you cannot edit the values inside but have to use the copy new_vel.
  robot_vel = vel;
  robot_vel_linear = vel.linear.x;
  robot_vel_angular = vel.angular.z;
  //ROS_INFO("Velocidad linear = %.2f -- Velocidad angular = %.2f", robot_vel_linear, robot_vel_angular);
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
    ROS_INFO("\n velocidad = %.3f \n phase 1 velocity = %.2f  \n phase 2 velocity = %.2f", velocity_robot_, phase_one_vel, phase_two_vel);
  }
}

/******************************************************
 * Giro del robot
 ******************************************************/


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

  //velocity_calculate();

  static float tripod_one_vel_multiplier, tripod_two_vel_multiplier;

  if(robot_vel_angular < 0){
    tripod_one_vel_multiplier = 0;
    tripod_two_vel_multiplier = 1;
  }
  else if(robot_vel_angular > 0){
    tripod_one_vel_multiplier = 1;
    tripod_two_vel_multiplier = 0;
  }

  else{
    tripod_one_vel_multiplier = 1;
    tripod_two_vel_multiplier = 1;
  }

  //static int phase = 0;
  phase_module = fmod(phase, 2);

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
                        ROS_INFO("Fase inicial -- Asignacion velocidades");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel * tripod_one_vel_multiplier);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel * tripod_two_vel_multiplier);
                      }

                      else{
                        ROS_INFO("Fase 1 -- Asignacion velocidades");
                        pata[robot_configuration[0][j]]->setVelocity(phase_one_vel * tripod_one_vel_multiplier);
                        pata[robot_configuration[1][j]]->setVelocity(phase_two_vel * tripod_two_vel_multiplier);
                      }
                    }
                    else
                    {
                        ROS_INFO("Fase 2 -- Asignacion velocidades");
                        pata[robot_configuration[0][j]]->setVelocity(phase_two_vel * tripod_one_vel_multiplier);
                        pata[robot_configuration[1][j]]->setVelocity(phase_one_vel * tripod_two_vel_multiplier);
                    }
                }
            }
        }
            start_movement_old = start_movement;
            ROS_INFO("Fase = %.2f -- Velocidad fase 1 = %.2f -- Velocidad fase 2 = %.2f", phase_module, phase_one_vel, phase_two_vel);

  }



    /**** INICIO DE LA COMPROBACION DE QUE LAS PATAS YA SE ENCUENTRAN EN POSICION ****/
    else
    {
      //ROS_INFO("ELSE");
        for(int i = 0; i < tripods_quantity; i++)
        {
            for(int j = 0; j < tripods_length; j++)
            {
                //float tripod_one_angle, tripod_two_angle;
                float angle;
//                ROS_INFO("phase module = %.1f", phase_module);
                /* --- GIRAR A LA DERECHA --- */
//                if(robot_vel_angular == 1){
//                  ROS_INFO("Giro a la derecha");
//                  leg_in_position.at(1) = true;
//                  leg_in_position.at(2) = true;
//                  leg_in_position.at(5) = true; //leg_in_position.at(3) = true; //leg_in_position.at(4) = true;
//                  if(phase_module == 0)
//                  {

//                      if(i == 0)
//                          angle = take_off_angle; //Angulo para tripode_1 y phase_0

//                      else //(i == 1)
//                          angle = take_off_angle;    //Angulo para el tripode_1 y phase_1
//  //                    ROS_INFO("angulo %.2f", angle);
//                      pata[1]->setPosition(take_off_angle);
//                      pata[2]->setPosition(take_off_angle);
//                      pata[5]->setPosition(take_off_angle);

//                  }
//                  else
//                  {
//                    //leg_in_position.at(3*i + j) = true;
//                      if(i == 0)
//                          angle = take_land_angle;
//                      else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};
//                          angle = take_land_angle;

//                      //pata[1]->setPosition(take_land_angle);
//                      //pata[2]->setPosition(take_land_angle);
//                      //pata[5]->setPosition(take_land_angle);
//                  }
//                }

//                /* --- GIRAR A LA IZQUIERDA --- */
//                else if(robot_vel_angular == -1){
//                  if(phase_module == 0)
//                  {
//                      if(i == 0)
//                          angle = take_land_angle; //Angulo para tripode_1 y phase_0

//                      else //(i == 1)
//                          angle = take_land_angle;    //Angulo para el tripode_1 y phase_1
//  //                    ROS_INFO("angulo %.2f", angle);
//                  }
//                  else
//                  {
//                      if(i == 0)
//                          angle = take_off_angle;
//                      else //(i = 1)bool leg_in_position[groups_length][groups_quantity] = {{false, false, false},{false, false, false}};
//                          angle = take_off_angle;
//                  }
//                }

                /* --- AVANCE RECTO --- */
//                else { //robot vel_angular == 0
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
//                }

                float leg_actual_pos = fmod(patas_posicion_actual.at(robot_configuration[i][j]), 2 * M_PI);
                float distance_old = 0.0;
                float distance = fabs(angle - leg_actual_pos);
                //if(i == 0 && j == 0 && (distance != distance_old)){
                //ROS_INFO("Distancia de la pata %d = %.2f -- i = %d -- j = %d", robot_configuration[i][j]+1, distance, i, j);
                   //distance_old = distance;
                //}
                //ROS_INFO("Fase %.2f -- Pata 0 -- Posicion %.2f -- Error %.2f -- Velocidad %.2f", phase_module, leg_actual_pos, distance, pata[0]->getVel());
                if(distance < 0.1)// && !leg_in_position[robot_configuration[i][j]]) //&& !leg_in_position.at(3*i+j))
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

// Funcion para dejar al robot quieto en tripode alterno
bool stop_robot()
{
  const int tripods_quantity  = 2;
  const int tripods_length = 3;
  std::vector<int>tripod_one = {0, 3, 4};
  std::vector<int>tripod_two = {1, 2, 5};

  static std::vector<bool>tripod_one_position = {false, false, false};
  static std::vector<bool>tripod_two_position = {false, false, false};

  static std::vector<bool>tripod_position = {false, false, false, false, false, false};

  int robot_configuration[tripods_quantity][tripods_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)},
                                                               {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};

  const float take_off_angle = 0.39;
  const float take_land_angle = 5.89;

  float tripod_one_desired_angle;
  float tripod_two_desired_angle;



  for(int i = 0; i < tripods_quantity; i++)
  {
    for(int j = 0; j < tripods_length; j++)
    {
      // Modulo de fase == 0 --> tripod_one == take_off_angle // tripod_two == take_land_angle
      // Modulo de fase == 1 --> tripod_one == take_land_angle // tripod_two == take_off_angle
      if(phase_module == 0)
      {
        tripod_one_desired_angle = take_off_angle;
        tripod_two_desired_angle = take_land_angle;
      }

      else  //phase_module == 1
      {
        tripod_one_desired_angle = take_land_angle;
        tripod_two_desired_angle = take_off_angle;
      }

      float leg_actual_pos = fmod(patas_posicion_actual.at(robot_configuration[i][j]), 2 * M_PI);

      if(i == 0)
      {
        float distance = fabs(tripod_one_desired_angle - leg_actual_pos);
        if(distance < 0.07)
        {
          tripod_one_position.at(j) = true;
          tripod_position.at(robot_configuration[i][j]) = true;
          pata[robot_configuration[i][j]]->setPosition(tripod_one_desired_angle);
        }
      }

      else  //i = 1
      {
        float distance = fabs(tripod_two_desired_angle - leg_actual_pos);
        if(distance < 0.07)
        {
          tripod_two_position.at(j) = true;
          tripod_position.at(robot_configuration[i][j]) = true;
          pata[robot_configuration[i][j]]->setPosition(tripod_two_desired_angle);
        }
      }
    }

  }

  for(int i = 0; i < tripods_quantity; i++)
  {
    for(int j = 0; j < tripods_length; j++)
    {
      if(tripod_position.at(robot_configuration[i][j]) == false)
      {
        pata[robot_configuration[i][j]]->setVelocity(1.0);
      }
    }
  }

  bool position_reached = std::all_of(tripod_position.begin(), tripod_position.end(), [](bool reached) {return reached;});

  if(position_reached)
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
                  tripod_position.at(j) = false;
              else
                  tripod_position.at(j+3) = false;
      }
    }
      phase++;
      phase_module = phase % 2;
      //ROS_INFO("phase = %d -- module = %.2f", phase, phase_module);
      return true;
  }
  else
      return false;
}

bool move_robot(){
  static bool flag;
  bool move_robot = ((robot_vel_linear != 0) || (robot_vel_angular != 0));
    switch(state_actual)
    {
      case origin:

        static int i = 0;
        if(!move_robot){
          if(i = 0){
            ROS_INFO("Origin --> Exit");
            i = 1;
          }
          state_actual = origin;
          return true;
          break;
        }

        if(move_robot && (robot_vel_linear > 0.5) && (robot_vel_angular == 0))//&& (-1 < robot_vel_angular < 1)){
        {
          ROS_INFO("Origin");
          flag = move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = origin;

          i = 0;
          break;
        }

        if(move_robot && (robot_vel_linear == 0) && (robot_vel_angular != 0))//&& (-1 < robot_vel_angular < 1)){
        {
          ROS_INFO("Dentro");
          if(robot_vel_angular > 0.5)
          {
            ROS_INFO("Origin --> Turn RIGH");
            state_actual = transition_turn_right;
            //flag = move_legs(true);
            }
            if(robot_vel_angular < -0.5)
            {
            ROS_INFO("Origin --> Turn LEFT");
            state_actual = transition_turn_left;
            //flag = move_legs(true);
          }

          state_actual = transition_turn;
          state_next = phase_one_turn;
          state_last = origin;

          i = 0;
          break;
        }

      case phase_one:
        if(!move_robot){
          if(i == 0){
            ROS_INFO("Phase 1 --> Exit");
            i = 1;
          }
          state_actual = phase_one;
          state_next = transition;

          break;
        }
        if(state_next == transition && move_robot && (robot_vel_linear > 0.5) && (robot_vel_angular == 0))// && (-1 < robot_vel_angular < 1))
        {
          ROS_INFO("phase_1");
          flag = move_legs(true);
          state_actual = transition;
          state_next = phase_two;
          state_last = phase_one;
          ROS_INFO("Move to TRANSITION");
          i = 0;

          break;
        }


      case phase_two:
        if(!move_robot){
          if(i == 0){
            ROS_INFO("Phase 2 --> Exit");
            i = 1;
          }
          state_actual = phase_two;
          state_next = transition;

          break;
        }
        if(state_next == transition && move_robot && (robot_vel_linear > 0.5) && (robot_vel_angular == 0))// && (-1 < robot_vel_angular < 1))
        {
          ROS_INFO("phase_2");
          flag = move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = phase_two;
          ROS_INFO("Move to TRANSITION");
          i = 0;
          break;
        }


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

/*bool stand_up()
{
  static std::vector<bool> legs_in_position = {false, false, false, false, false, false};

  std::vector<int>tripod_one = {0, 3, 4};
  std::vector<int>tripod_two = {1, 2, 5};

  const int tripod_quantity = 2;
  const int tripod_length = 3;
  int robot_configuration[tripod_quantity][tripod_length] = {{tripod_one.at(0), tripod_one.at(1), tripod_one.at(2)},
                                                             {tripod_two.at(0), tripod_two.at(1), tripod_two.at(2)}};


  double tripod_one_desired_position = 2 * M_PI; //5.89;
  double tripod_two_desired_position = 2 * M_PI; //0.39;
  // Contador de seguridad por si la pata no puede alcanzar la posicion objetivo
  static std::vector<int> safe_counter = {0, 0, 0, 0, 0, 0};
  static int safe_counter_qty = 10000000;

  for(int i = 0; i < tripod_quantity; i++)
        {
          for(int j = 0; j < tripod_length; j++)
          {
  //          ROS_INFO("El robot esta en el suelo, empieza el movimiento");
            // First tripod
            if(i == 0)
            {
              if(!legs_in_position.at(3 * i + j))
              {
                //float error = (fmod(pata[robot_configuration[i][j]]->getPos(), tripod_one_desired_position));
                float error = (fmod(patas_posicion_actual.at(robot_configuration[i][j]), tripod_one_desired_position));

                ROS_INFO("Error de la pata %d --> %.2f", robot_configuration[i][j], error);
                float velocity = abs(2.0 * error);
                if(safe_counter.at(j) > safe_counter_qty)
                {
                  velocity = velocity * 1.5;
                }
                //ROS_INFO("Velocity tripod 1 = %.2f", velocity);
                pata[robot_configuration[i][j]]->setVelocity(velocity);
                if (fabs(error) < 0.05)
                {
                  pata[robot_configuration[i][j]]->setPosition(tripod_one_desired_position);
                  ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                  legs_in_position.at(j) = true;
                  safe_counter.at(j) = 0;
                }

              }
              // Second tripod
              else
              {
                if(!legs_in_position.at(j + tripod_length))
                {
                  float error = abs(fmod(pata[robot_configuration[i][j]]->getPos(), tripod_two_desired_position));
                  float velocity = (1.0 * error);
                  if(safe_counter.at(j + tripod_length) > safe_counter_qty)
                  {
                    velocity = velocity * 1.5;
                  }
                  pata[robot_configuration[i][j]]->setVelocity(velocity);
                  if (fabs(error) < 0.05)
                  {
                    pata[robot_configuration[i][j]]->setPosition(tripod_two_desired_position);
                    ROS_INFO("Posicion de la pata %d alcanzada", robot_configuration[i][j]);
                    legs_in_position.at(j + tripod_length) = true;
                    safe_counter.at(j + tripod_length) = 0;
                  }
                }
              }
            }
            safe_counter.at(j + tripod_length) = safe_counter.at(j + tripod_length) + 1;
          }

        }

        // Ahora toca chequear las condiciones de salida.
        if(std::all_of(legs_in_position.begin(), legs_in_position.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
        {
          for(int i = 0; i < 6; i++)
          {
            legs_in_position.at(i) = false;
          }
          ROS_INFO ("All legs in position");
          if(grounded_)
          {
            ROS_INFO("Grounded = false");
            grounded_ = !grounded_;
          }
          else
          {
            ROS_INFO("Grounded = true");
            grounded_ = true;
          }
          return true;
        }
        else
          return false;

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
  ros::init(argc, argv, "hexapodo_1_pata_velocidad");
  ros::NodeHandle nh;

  ros::Subscriber joint_states_sub = nh.subscribe("/hexapodo/joint_states", 1, joint_states_callback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);
  
  ros::Rate rate(100.0);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  //Se inicializan las patas
  //setupPatas(nh);
  
  stand_by hexapod_stand_by;
  common_methods hexapod_common_methods;
  hexapod_stand_by.stand_up();

  enum robot_state {stop, advance, turn_left, turn_right};
  robot_state robot_state_actual = stop;
  robot_state robot_state_next;
  robot_state robot_state_last;

  velocity_calculate();

  int phase = 0;
  std::vector<float>tripode_1_posiciones = {2 * M_PI - 0.52, 0.52}; //{5.76, 0.52};
  std::vector<float>tripode_2_posiciones = {0.52, 2 * M_PI - 0.52};

  /*pata[0]->setPosition(0.39);
  pata[1]->setPosition(5.89);
  pata[2]->setPosition(5.89);
  pata[3]->setPosition(0.39);
  pata[4]->setPosition(0.39);
  pata[5]->setPosition(5.89);*/

  while(ros::ok()){

    //ROS_INFO("Posicion de la pata %d --> %.2f", 1, fmod(hexapod_common_methods.pata[0]->getPos(), (2 * M_PI)));
    if(fmod(hexapod_common_methods.pata[0]->getPos(), (2 * M_PI)) < 0)
    {
      ROS_INFO("Posicion de la pata %d --> %.2f", 1, (fmod(hexapod_common_methods.pata[0]->getPos(), (2 * M_PI)) + (2 * M_PI)));

    }

    if((phase % 2) == 0)
    {
      std::vector<int>tripode_1 = {0, 4};
      std::vector<int>tripode_2 = {1, 5};
      float tripod_one_actual_position = tripode_1_posiciones.at(0);
      float tripod_one_desired_position = tripode_1_posiciones.at(1);
      float tripod_two_actual_position = tripode_2_posiciones.at(1);
      float tripod_two_desired_position = tripode_2_posiciones.at(0);

      float pata_1_actual_position = fmod(hexapod_common_methods.pata[0]->getPos(), (2 * M_PI));
      float pata_5_actual_position = fmod(hexapod_common_methods.pata[4]->getPos(), (2 * M_PI));
      float pata_2_actual_position = fmod(hexapod_common_methods.pata[1]->getPos(), (2 * M_PI));
      float pata_6_actual_position = fmod(hexapod_common_methods.pata[5]->getPos(), (2 * M_PI));
      float pata_1_actual_position_abs = fabs(pata_1_actual_position);
      std::vector<float> error(6);

      error.at(1) = tripod_one_desired_position - pata_2_actual_position;
      error.at(5) = tripod_one_desired_position - pata_6_actual_position;

      if(M_PI < pata_1_actual_position <= (2 * M_PI))
      {
        error.at(0) = (tripod_one_desired_position ) - pata_1_actual_position;
        error.at(4) = tripod_one_desired_position - pata_5_actual_position;
      }

      else
      {
        error.at(0) = tripod_one_desired_position - pata_1_actual_position;
        error.at(4) = tripod_one_desired_position - pata_5_actual_position;
      }

      ROS_INFO("Error de la pata 1 -- %.2f Posicion deseada -- %.2f", error.at(0), tripod_one_desired_position);
      ROS_INFO("Error de la pata 2 -- %.2f Posicion deseada -- %.2f", error.at(1), tripod_two_desired_position);
      ROS_INFO("Error de la pata 5 -- %.2f Posicion deseada -- %.2f", error.at(4), tripod_one_desired_position);
      ROS_INFO("Error de la pata 6 -- %.2f Posicion deseada -- %.2f", error.at(5), tripod_two_desired_position);

      hexapod_common_methods.pata[2]->setPosition(2 * M_PI - 0.52);
      hexapod_common_methods.pata[3]->setPosition(0.52);
      hexapod_common_methods.pata[0]->setVelocity(-error.at(0));
      hexapod_common_methods.pata[4]->setVelocity(-error.at(4));
      hexapod_common_methods.pata[5]->setPosition(2 * M_PI - 0.52);
      hexapod_common_methods.pata[4]->setPosition(0.52);
      //hexapod_common_methods.pata[1]->setVelocity(error.at(1));
      //hexapod_common_methods.pata[5]->setVelocity(error.at(5));
    }




    //ROS_INFO("Entro al while");
    /*pata[1]->setVelocity(10.0);
    pata[5]->setVelocity(10.0);
    pata[0]->setVelocity(-10.0);
    pata[4]->setVelocity(-10.0);*/
    /*if((pata[1]->getPos() == 5.89) && (pata[5]->getPos() == 5.89)){
      ROS_INFO("Entro en el bucle");
      pata[1]->setVelocity(1.0);
      pata[5]->setVelocity(1.0);
    }
    else
    {
      pata[1]->setPosition(5.89);
      pata[5]->setPosition(5.89);
      pata[1]->setVelocity(1.0);
      pata[5]->setVelocity(1.0);
    }*/

    /******************************************************
     * Maquina de estados
     ******************************************************/

    /*
    if(robot_vel_linear > 0.5 && robot_vel_angular == 0)
    {
      robot_state_last = robot_state_actual;
      robot_state_actual = advance;
    }
    
    else if(robot_vel_linear == 0 && robot_vel_angular != 0)
    {
      if(robot_vel_angular == -1)
      {
        robot_state_last = robot_state_actual;
        robot_state_actual = turn_left;
      }
      else if(robot_vel_angular == 1)
      {
        robot_state_last = robot_state_actual;
        robot_state_actual = turn_right;
      }
      else
      {
        robot_state_last = robot_state_actual;
        robot_state_actual = stop;
      }
    }
    else
    {
      robot_state_last = robot_state_actual;
      robot_state_actual = stop;
    }
    
    switch(robot_state_actual)
    {
    case stop:
    {
      //static bool flag;
      ROS_INFO("Stop Robot");
      //if(robot_state_last != stop)
//      {
//        bool flag = stop_robot();
//      }
//      if(flag)
//      {
//        ROS_INFO("End Stop Robot");
//        //robot_state_last = robot_state_actual;
//        robot_state_actual = stop;

//      }
    }
      break;
      
    case advance:

      ROS_INFO("Advance");
      static bool flag = move_robot();
      if(flag)
      {
        break;
      }
      
    case turn_right:
      ROS_INFO("Turn Right");
      
      break;
      
    case turn_left:
      ROS_INFO("Turn Left");

      break;
    } */
  }



      //ros::spinOnce();
}
