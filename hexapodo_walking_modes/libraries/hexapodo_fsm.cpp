#include "hexapodo_fsm.h"

/******************************************************
 * Callbacks
 ******************************************************/
void hexapodo_fsm::init_subscribers()
{
  velocity_subs = nh.subscribe("/cmd_vel", 1, &hexapodo_fsm::cmd_vel_callback, this);
}

void hexapodo_fsm::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &vel)
{
  robot_vel_linear_ = vel->linear.x;
}

/******************************************************
 * Movimiento de las patas
 ******************************************************/


/******************************************************
 * Maquina de estados
 ******************************************************/
bool hexapodo_fsm::fsm(float vel_linear)
{
  enum leg_state {origin, phase_one, phase_two, transition};  // Fases de la maquina de estados
  /*
   *
   * Origin: Es la posicion inicial del robot, debe estar en tripode alterno por defecto. Con el tripode 1 para despegar y el tripode 2 en aterrizaje
   * Phase_one: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de despegue y el tripode 2 desde el angulo de aterrizaje
   * Phase_two: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de aterrizaje y el tripode 2 desde el angulo de despegue
   * Transition: son los estados en los que las patas se encuentran circulando
   *
   * */

  // Se cargan los estados por defecto de la fsm
  static leg_state state_actual = phase_one;
  static leg_state state_last;
  static leg_state state_next = transition;

  while(vel_linear > 0)
  {
    switch(state_actual)
    {
      case phase_one:
      {
        vel_linear = robot_vel_linear_;
        ROS_INFO("vel_linear = %.2f", vel_linear);
//        eStop = hexapod_common_methods_.eStop_;
//        advance = hexapod_common_methods_.advance();
//        turn_left = hexapod_common_methods_.turn_left();
//        turn_right = hexapod_common_methods_.turn_right();
        static bool flag;

        // Se suelta R1 y por tanto se sale
        //if(!turn_left && !turn_right && !advance)
        if(vel_linear <= 0)
        {
          ROS_INFO("Phase 1 --> Exit");
          state_actual = phase_one;
          state_next = transition;
          return true;

          break;
        }

        else if((vel_linear > 0) && (state_next == transition))
        {
          ROS_INFO("Phase 1 --> Start movement");
          //flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_two;
          state_last = phase_one;
          ROS_INFO("Phase 1 --> Transition");

          break;
        }

      } // Phase one

      case phase_two:
      {
        ROS_INFO("Entro en fase 2");
        vel_linear = robot_vel_linear_;
        /*eStop = hexapod_common_methods_.eStop_;
        advance = hexapod_common_methods_.advance();
        turn_left = hexapod_common_methods_.turn_left();
        turn_right = hexapod_common_methods_.turn_right();*/
        static bool flag;
        if(vel_linear <= 0)
        {
          ROS_INFO("Phase 2 --> Exit");
          state_actual = phase_two;
          state_next = transition;

          return true;
        }

        if((vel_linear > 0) && (state_next == transition))
        {
          ROS_INFO("Phase 2 --> Start movement");
          //flag = this->move_legs(true);
          state_actual = transition;
          state_next = phase_one;
          state_last = phase_two;
          ROS_INFO("Phase 2 --> Transition");
        }

        break;
      } // Phase two

      case transition:
      {
        static bool flag;
        // Phase 1 --> transition --> Phase 2
        if(state_last == phase_one && state_next == phase_two)
        {
          flag = true;
          //ROS_INFO("transition from phase 1");
          //flag = this->move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 1");
            state_actual = phase_two;
            state_next = transition;
            state_last = transition;
          }
        }

        // Phase 2 --> transition --> Phase 1
        else if(state_last == phase_two && state_next == phase_one)
        {
          flag = true;
          //ROS_INFO("transition from phase 2");
          //flag = this->move_legs(false);
          if(flag)
          {
            ROS_INFO("FLAG TRUE transition from phase 2");
            state_actual = phase_one;
            state_next = transition;
            state_last = transition;
          }
        }

        break;
        } // Transition
    }
  } // while(vel_linear > 0)

}

/******************************************************
 * Init
 ******************************************************/


/******************************************************
 * Constructor
 ******************************************************/
hexapodo_fsm::hexapodo_fsm()
{
  init_subscribers();
}


