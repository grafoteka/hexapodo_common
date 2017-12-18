#ifndef STAND_BY_H
#define STAND_BY_H

#include <vector>
#include <math.h>
#include <common_methods/hexapod_common_methods.h>

class stand_by
{
private:

  // Angulo total de la fase suelo
  float total_angle_degrees;
  float total_angle_rads;

  std::vector<int> tripod_one_;
  std::vector<int> tripod_two_;
  
  // Variable para llamar a la funcion de set_position con la posicion adecuada de cada tripode
//  float tripod_one_position_;
//  float tripod_two_position_;

  // Variable que indica si viene desde el suelo o desdefloat tripod_one_desired_position, float tripod_two_desired_position otro estado
  // TRUE = estado anterior = origen (en el suelo)
  // FALSE = estado anterior = otro estado (ya esta en pie)
  bool grounded_;

  bool set_position(float, float);

  void get_down();

  common_methods hexapodo_common_methods;

public:
  stand_by(); // Constructor

  void stand_by_fsm();  // Maquina de estados para levantar o poner en el suelo el robot

  // Esta funcion la hago publica para poder llamarla desde otros modos en caso de que no
  // se encuentre el robot en la posicion adecuada.
  void stand_up();  // Funcion para levantar el robot y dejarlo en tripode alterno

  bool getGrounded() const;
};

#endif // STAND_BY_H
