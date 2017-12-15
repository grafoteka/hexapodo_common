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

  bool eStop = hexapodo_altern_tripod.eStop_;

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
 * Check tripods
 ******************************************************/
bool altern_tripod::check_tripod()
{
  static std::vector<bool> legs_in_position = {false, false, false, false, false, false};
  float take_off_angle_rads = (hexapodo_altern_tripod.total_angle_rads_ / 2);
  float take_land_angle_rads = (2 * M_PI - hexapodo_altern_tripod.total_angle_rads_ / 2);

  // Check tripod one
  for(int i = 0; i < tripods_length_; i++)
  {
    if(!legs_in_position.at(i))
    {
  //    ROS_INFO("Primer for");
      float error = fmod(hexapodo_altern_tripod.pata[tripod_one_.at(i)]->getPos(), take_off_angle_rads);
//      ROS_INFO("Error pata %d -- %.2f", i, error);
      if(error < 0.11)
      {
        legs_in_position.at(i) = true;
        ROS_INFO("Pata %d en posicion", tripod_one_.at(i));
      }
    }
  }

  // Check tripod two
  for(int i = 0; i < tripods_length_; i++)
  {
    if(!legs_in_position.at(i+3))
    {
      float error = fmod(hexapodo_altern_tripod.pata[tripod_two_.at(i)]->getPos(), take_land_angle_rads);
      if(error < 0.11)
      {
        legs_in_position.at(i+3) = true;
        ROS_INFO("Pata %d en posicion", tripod_two_.at(i));
      }
    }
  }


  // Comprobacion de que las patas se encuentran en posicion
  if(std::all_of(legs_in_position.begin(), legs_in_position.end(), [](bool legs_in_position_true) {return legs_in_position_true;}))
  {
    ROS_INFO ("All legs in position");
    for(int i = 0; i < legs_in_position.size(); i++)
    {
      legs_in_position.at(i) = false;
    }

    return true;
  }

  else
    return false;


}

/******************************************************
 * Init
 ******************************************************/
void altern_tripod::init(bool exit)
{
  static bool flag = false; // Bandera que indica que las patas se encuentran en la posicion adecuada

  flag = this->check_tripod();

  // Mientras las patas no se encuentren en la posicion adecuada se vuelve a llamar a la funcion que las coloca
//  while(!flag)
//  {
//    flag = this->check_tripod();
//    if(!flag)
//      hexapodo_stand_by.get_position();
//  }

//  ROS_INFO("Las patas ya estan en posicion -- flag = %d", flag);

  if(flag && eStop_)
  {
    ROS_INFO("Vamos a la maquina de estados");
    this->fsm(!eStop_);
  }
  }


/******************************************************
 * Constructor
 ******************************************************/
altern_tripod::altern_tripod()
{

}

