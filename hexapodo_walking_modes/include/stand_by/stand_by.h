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

  std::vector<int> tripod_one;
  std::vector<int> tripod_two;

  // Variable que indica si viene desde el suelo o desde otro estado
  // TRUE = estado anterior = origen (en el suelo)
  // FALSE = estado anterior = otro estado (ya esta en pie)
  bool grounded;

  common_methods hexapodo_stand_by;

public:
  stand_by(); // Constructor
  void get_position();
};

#endif // STAND_BY_H
