//====================================================================================
//  Autor: Raul Cebolla Arroyo
//  Matricula: 13069
//  Archivo: modo_tripode_alterno.cpp
//  Descripcion: Codigo correspondiente al nodo que organiza el modo de marcha
//               de tripode alterno para un robot rhex.
//====================================================================================

//---------------------------------------------------------------------------
//  Includes y namespaces
//---------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "c_leg/c_leg.h"
#include <math.h>


using namespace std;

//---------------------------------------------------------------------------
//  Define Macros
//---------------------------------------------------------------------------

#define GRAD2RAD(X) X*2*PI/360.0

//---------------------------------------------------------------------------
//  Defines constantes
//---------------------------------------------------------------------------

#define VEL_AIR 0.5                 //Velocidad que toma la pata en el aire
#define VEL_GND VEL_AIR*0.25        //Velocidad que toma la pata en el suelo
#define ATR_GRD 30                  //Angulo de aterrizaje en grados [º]
#define DESP_GRD 330                //Angulo de despegue en grados [º]
#define ATR_RAD GRAD2RAD(ATR_GRD)   //Angulo de aterrizaje en radianes [rad]
#define DESP_RAD GRAD2RAD(DESP_GRD) //Angulo de despegue en radianes [rad]
#define MARGIN GRAD2RAD(10)          //Margen de aceptacion de la posicion en radianes

//---------------------------------------------------------------------------
//  Definicion de tipos y enumeraciones
//---------------------------------------------------------------------------

//Enumeracion de la variable que servira como estado
enum EstadoMov {REPOSO, POSICIONAMIENTO, MOV_TRIPODE_1, MOV_TRIPODE_2};

//---------------------------------------------------------------------------
//  Variables globales necesarias
//---------------------------------------------------------------------------

//Objetos de cada una de las patas

PataC *pata[6];

//Declaracion de la variable de estado del movimiento
EstadoMov estado = REPOSO;

//Variable correspondiente a las señales de las transiciones de los estados,
//en este caso se liga a cada una de las patas
bool fin_transicion_patas[6] = {false, false, false, false, false, false};

//---------------------------------------------------------------------------
//  Rutina de setup
//---------------------------------------------------------------------------

//Setup
void setup(ros::NodeHandle &nh){

    for (int i = 0; i < 6; i++){
        pata[i] = new PataC(i+1, nh);
    }

}

//---------------------------------------------------------------------------
//  Rutinas de control de los estados del movimiento
//---------------------------------------------------------------------------

//Rutina que comprueba que la pata ha alcanzado la posicion indicada
bool comprobarPosicion (unsigned int num, float pos){

    double pos_pata;

    pos_pata = pata[num-1]->getPos();
    ROS_INFO("Posicion pata %d : %lf", num, pos_pata);

    //Si ha alcanzado el limite que debe alcanzar en el estado debe cambiar
    //el control a posicion y comandarlo a la posicion de destino
    ROS_INFO("Se valora si ha llegado a la posicion: %lf", pos);
    if((fabs((pos_pata - pos)) < MARGIN)||(fabs((pos_pata - pos)) < MARGIN + 2*PI)){
        //ROS_INFO("Se ha valorado que ha llegado a la posicion %lf", pos);
        //ROS_INFO("Se cambia el control de posicion de la pata %d", num);
        pata[num-1]->activarControlPosicion();
        //ROS_INFO("Se pone la posicion de referencia en la pata %d", num);
        pata[num-1]->setPosRef(pos);
        //ROS_INFO("Se pone true a la bandera de transicion de la pata %d", num);
        fin_transicion_patas[num-1]=true;
        return true;
    }

    //ROS_INFO("Aun no ha llegado a la posicion");
    return false;
}

//Funcion que limpia el vector de señales de fin de movimiento para cada
//pata en cada fin de transicion
void clearFinTransicionPatas(){
    for(int i = 0; i < 6; i++){
        fin_transicion_patas[i] = false;
        continue;
    }
    return;
}

//Rutina que valora las variables de señal y cambia de estado
bool realizarTransicion(){

    //Se valora si todas las banderas se han alcanzado para dar la trans
    bool realizar_trans = true;
    for(int i = 0; i < 6; i++){
        realizar_trans = realizar_trans && fin_transicion_patas[i];
    }

    //En caso de que se tenga que dar la transicion se da el cambio
    if(realizar_trans){
        switch(estado){
        case REPOSO:            estado = POSICIONAMIENTO; break;
        case POSICIONAMIENTO:   estado = MOV_TRIPODE_1; break;
        case MOV_TRIPODE_1:     estado = MOV_TRIPODE_2; break;
        case MOV_TRIPODE_2:     estado = MOV_TRIPODE_1; break;
        default:                break;
        }
        clearFinTransicionPatas();
    }else{
        //ROS_INFO("No se realiza la transicion de estados");
    }

    //Devuelve el valor logico si se ha realizado la transicion
    return realizar_trans;
}

//  --- Modo Reposo ---
//          En el modo reposo el robot se mantiene de pie sin moverse

void comandoReposo(){

    //ROS_INFO("Se comanda para el estado REPOSO");
    for(int i = 0; i < 6; i++){
        //Se activa el control de posicion y se establece el cero como posicion
        //a tomar
        pata[i]->activarControlPosicion();

        //Se manda a cero
        pata[i]->setPosRef(0);
    }

    return;

}

//Durante la etapa REPOSO no tiene que comprobar nada
void checkReposo(){
    comandoReposo();
    for(int i = 0; i < 6; i++){
        if (!fin_transicion_patas[i]){
            fin_transicion_patas[i] = comprobarPosicion(i + 1, 0);
        }
    }
    return;
}

//  --- Modo Posicionamiento ---
//          Primer paso para dar el movimiento, el robot se coloca en los
//          angulos de despegue y aterrizaje de cada tripode

void comandoPosicionamiento (){

    //ROS_INFO("Se comanda para POSICIONAMIENTO");
    //Se manda los tripodes a cada posicion

    //Tripode 1 (2, 3, 6) --> angulo de aterrizaje
    pata[2-1]->setPosRef(ATR_RAD);
    pata[3-1]->setPosRef(ATR_RAD);
    pata[6-1]->setPosRef(ATR_RAD);

    //Tripode 2 (1, 4, 5) --> angulo de despegue
    pata[1-1]->setPosRef(DESP_RAD);
    pata[4-1]->setPosRef(DESP_RAD);
    pata[5-1]->setPosRef(DESP_RAD);

    return;

}

//Durante la etapa de posicionamiento debe comprobar que se encuentra en
//las posiciones
void checkPosicionamiento(){

    //ROS_INFO("Se checkea el posicionamiento");

    for(int i = 0; i < 6; i++){
      if(!fin_transicion_patas[i]){
        switch(i){
        case 0: fin_transicion_patas[i] = comprobarPosicion(1, DESP_RAD); break;
        case 1: fin_transicion_patas[i] = comprobarPosicion(2, ATR_RAD); break;
        case 2: fin_transicion_patas[i] = comprobarPosicion(3, ATR_RAD); break;
        case 3: fin_transicion_patas[i] = comprobarPosicion(4, DESP_RAD); break;
        case 4: fin_transicion_patas[i] = comprobarPosicion(5, DESP_RAD); break;
        case 5: fin_transicion_patas[i] = comprobarPosicion(6, ATR_RAD); break;
        }
      }
    }

    return;
}

//  --- Modo movimiento tripode 1 ---

void comandoMovTripode1(){

    //ROS_INFO("Se comanda para MOV_TRIPODE_1");

    //Se activan los controladores de velocidad de las patas
    for (int i = 0; i < 6; i++){
        pata[i]->activarControlVelocidad();
    }

    //Se comandan las velocidades correspondientes

    //El tripode 1, se dispone a velocidad lenta
    pata[2-1]->setVelRef(VEL_GND);
    pata[3-1]->setVelRef(VEL_GND);
    pata[6-1]->setVelRef(VEL_GND);

    //El tripode 2, se dispone a velocidad rapida
    pata[1-1]->setVelRef(VEL_AIR);
    pata[4-1]->setVelRef(VEL_AIR);
    pata[5-1]->setVelRef(VEL_AIR);

    return;
}

void checkMovTripode1(){

    //ROS_INFO("Se checkea el MOV TRIPODE 1");

    //El angulo de comparacion del tripode 1 es el de despegue
    comprobarPosicion(2, DESP_RAD);
    comprobarPosicion(3, DESP_RAD);
    comprobarPosicion(6, DESP_RAD);

    //El angulo de comparacion del tripode 2 es el de aterrizaje
    comprobarPosicion(1, ATR_RAD);
    comprobarPosicion(4, ATR_RAD);
    comprobarPosicion(5, ATR_RAD);
    return;

}

// --- Modo movimiento tripode 2 ---

void comandoMovTripode2(){

    ROS_INFO("Se comanda para MOV_TRIPODE_2");

    //Se activan los controladores de velocidad de las patas
    for (int i = 0; i < 6; i++){
        pata[i]->activarControlVelocidad();
    }

    //Se comandan las velocidades correspondientes

    //El tripode 2, se dispone a velocidad lenta
    pata[1-1]->setVelRef(VEL_GND);
    pata[4-1]->setVelRef(VEL_GND);
    pata[5-1]->setVelRef(VEL_GND);

    //El tripode 1, se dispone a velocidad rapida
    pata[2-1]->setVelRef(VEL_AIR);
    pata[3-1]->setVelRef(VEL_AIR);
    pata[6-1]->setVelRef(VEL_AIR);

    return;
}

void checkMovTripode2(){

    //ROS_INFO("Se checkea el MOV TRIPODE 2");

    //El angulo de comparacion del tripode 2 es el de despegue
    comprobarPosicion(1, DESP_RAD);
    comprobarPosicion(4, DESP_RAD);
    comprobarPosicion(5, DESP_RAD);

    //El angulo de comparacion del tripode 1 es el de aterrizaje
    comprobarPosicion(2, ATR_RAD);
    comprobarPosicion(3, ATR_RAD);
    comprobarPosicion(6, ATR_RAD);
    return;
}

//---------------------------------------------------------------------------
//  Programa principal
//---------------------------------------------------------------------------

int main(int argc, char **argv){

    //Rutinas de inicializacion de ROS y del nodo

    // Set up ROS.
    ros::init(argc, argv, "modo_tripode_alterno");

    ros::NodeHandle nh;

    // Ejecutar la configuracion del nodo
    setup(nh);

    ros::Rate loop_rate(10);

    //Bucle principal del nodo
    while (ros::ok()){

        //Comprueba si se debe realizar una transicion en los estados
        //del movimiento
        if(realizarTransicion()){
            switch(estado){
            case REPOSO:
                ROS_INFO("Estado: REPOSO");
                comandoReposo();
                break;
            case POSICIONAMIENTO:
                ROS_INFO("Estado: POSICIONAMIENTO");
                comandoPosicionamiento();
                break;
            case MOV_TRIPODE_1:
                ROS_INFO("Estado: MOV_TRIPODE_1");
                comandoMovTripode1();
                break;
            case MOV_TRIPODE_2:
                ROS_INFO("Estado: MOV_TRIPODE_2");
                comandoMovTripode2();
                break;
            default:
                break;
            }

        }

        //Comprueba y actualiza las banderas de las señales de transicion
        //de estados
        switch(estado){
        case REPOSO:
            checkReposo();
            break;
        case POSICIONAMIENTO:
            checkPosicionamiento();
            break;
        case MOV_TRIPODE_1:
            checkMovTripode1();
            break;
        case MOV_TRIPODE_2:
            checkMovTripode2();
            break;
        default:
            break;
        }

        loop_rate.sleep();

    }

    for(int i=0; i<6; i++){
      delete pata[i];
    }

    return 0;

}
