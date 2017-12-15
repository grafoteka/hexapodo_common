#ifndef ALTERN_TRIPOD_H
#define ALTERN_TRIPOD_H

//====================================================================================
//  Autor: Jorge De Leon Rivas
//  mail: jorge.deleon@upm.es
//  date: 12/december/2017
//  version: 1.0
//  Archivo: altern_tripod.h
//  Descripcion: Archivo de cabezera para describir la funcionalidad de un modo de
//               marcha en tripode alterno para un robot hexapodo.
//====================================================================================

#include <vector>
#include <ros/ros.h>
#include <common_methods/hexapod_common_methods.h>
#include <stand_by/stand_by.h>

class altern_tripod : public common_methods
{
private:

  common_methods hexapodo_altern_tripod;
  stand_by hexapodo_stand_by;

  // vectores de las patas
  std::vector<int> tripod_one_ = {0, 3, 4};
  std::vector<int> tripod_two_ = {1, 2, 5};

  int tripods_quantity_ = 2;
  int tripods_length_ = 3;

  bool move_legs(bool); // Funcion que mueve las patas

  bool check_tripod();

  bool fsm(bool); // Funcion que es la maquina virtual de las fases

public:
  altern_tripod();  // Constructor
  void init(bool);

};

#endif // ALTERN_TRIPOD_H
