#include <altern_tripod/altern_tripod.h>

/******************************************************
 * Movimiento de las patas
 ******************************************************/
bool altern_tripod::move_legs(bool start_movement)
{

}

/******************************************************
 * Maquina de estados
 ******************************************************/
bool altern_tripod::fsm(bool exit)
{
  enum leg_state {origin, phase_one, phase_two, transition};  // Fases de la maquina de estados
  /*
   *
   * Origin:
   * Phase_one: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de despegue y el tripode 2 desde el angulo de aterrizaje
   * Phase_two: Es el inicio de la fase cuando el tripode 1 se empieza a mover desde el angulo de aterrizaje y el tripode 2 desde el angulo de despegue
   * Transition: son los estados en los que las patas se encuentran circulando
   *
   * */

  bool eStop = hexapod_common_methods_.eStop_;

  leg_state state_actual = phase_one;
  leg_state state_last;
  leg_state state_next = transition;

  switch(state_actual)
  {
    case phase_one:
    {
      static bool flag;
      if(eStop){
        ROS_INFO("Phase 1 --> Exit");
        state_actual = phase_one;
        state_next = transition;
      }

      if(!eStop && state_next == transition)
      {
        ROS_INFO("Phase 1 --> Start movement");
        flag = this->move_legs(true);
        state_actual = transition;
        state_next = phase_two;
        state_last = phase_one;
        ROS_INFO("Phase 1 --> Transition");
      }

      break;

    }


  }
}

/******************************************************
 * Init
 ******************************************************/
void altern_tripod::init()
{
  legs_in_position_ = hexapod_stand_by_.getGrounded();

  if(legs_in_position_ && !hexapod_common_methods_.eStop_)
  {
    ROS_INFO("Las patas estan en posicion");
    // Podemos ir a la fsm
    this->fsm(hexapod_common_methods_.exit_button_);
  }

  if(!legs_in_position_ && !hexapod_common_methods_.eStop_)
  {
    ROS_INFO("Las patas no estan en posicion, llamamos a Stand UP");
    hexapod_stand_by_.stand_up();
  }

}


/******************************************************
 * Constructor
 ******************************************************/
altern_tripod::altern_tripod()
{

}

