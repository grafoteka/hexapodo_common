#ifndef CLEG_H
#define CLEG_H

//====================================================================================
//  Autor: Raul Cebolla Arroyo
//  Matricula: 13069
//  Archivo: CLeg.h
//  Descripcion: Archivo de cabezera para describir la funcionalidad de una pata c de
//               un robot rhex y asi establecer su funcionalidad
//====================================================================================


//----------------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "string"

//----------------------------------------------------------------------------
//  Definicion de macros
//----------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265359
#endif

//----------------------------------------------------------------------------
//  Clase pata C
//----------------------------------------------------------------------------

class CLeg{

    //------------------------------------------------------------------------
    //  Variables miembro de la clase
    //------------------------------------------------------------------------

    //Campo con el numero de la pata que representa
    unsigned int num_pata;

    //Campo con el nombre de la pata
    std::string nom;

    //Variables de estado de posicion, velocidad y par
    float pos;  //Posicion      [rad]
    float vel;  //Velocidad     [rad/s]
    float eff;  //Par           [Nm]

    //Variables de referencia de posicion y velocidad
    float pos_ref;  //Posicion  [rad]
    float vel_ref;  //Velocidad [rad/s]

    //Estado del controlador
    //  True -> Posicion
    //  False -> Velocidad
    bool control_posicion;

    //--    ROS     --//

    //Node handle
    ros::NodeHandle *nh;

    //--    Publishers  --//

    //None

    //--    Subscribers --//

    //Joint state subscriber
    ros::Subscriber joint_state_subs;

    //--    Service Clients --//

    //Controller interface for each c-leg client
    ros::ServiceClient controller_interface_command_client;

    //------------------------------------------------------------------------
    //  Metodos privados de la clase
    //------------------------------------------------------------------------

    //Metodo que inicializan los clientes
    void inicControllerInterfaceClient();

    //Metodos que inicializan los subscriber
    void inicJointStateSub();

    //Metodo que actualiza los campos correspondientes a las variables de estado de la junta
    void parseJointState(const sensor_msgs::JointState &message);

    //Metodo que envia una request de comando hacia el controller interface
    bool requestCommand (float value, bool position_command);

public:

    //------------------------------------------------------------------------
    //  Constructor de la clase
    //------------------------------------------------------------------------

    //Constructor por defecto -- no hay
    //CLeg();

    //Constructor de la clase
    CLeg(unsigned int numero_pata, ros::NodeHandle &n);

    //------------------------------------------------------------------------
    //  Metodos publicos
    //------------------------------------------------------------------------

    //Metodo que actualiza el estado de las patas
    void    actualizarEstado();

    //Metodos GET
    float   getPos();
    float   getVel();
    float   getEff();
    bool    getEstControl();

    //Metodos que mandan los comandos de velocidad y posicion
    void    setPosition(float value);
    void    setVelocity(float value);

};

#endif // CLEG_H
