//----------------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------------

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

//----------------------------------------------------------------------------
//  Clase robot
//----------------------------------------------------------------------------

class hexapodo
{

	//------------------------------------------------------------------------
    //  Variables miembro de la clase
    //------------------------------------------------------------------------

	const int tripods_quantity;
	const int tripods_length;

	std::vector<int> tripod_one;
	std::vector<int> tripod_two; 

	robot_configuration [tripods_quantity][tripods_length];

	std::vector<bool> patas_posicion_origen = {false, false, false, false, false, false};

	// Angulos
	float total_angle = 45;
	float total_angle_rads = total_angle * PI / 180;
	float take_off_angle = total_angle_rads / 2;
	float take_land_angle = 2 * PI - (total_angle_rads / 2);

	//--    ROS     --//

    //Node handle
    ros::NodeHandle *nh;

    //--    Publishers  --//

    //None

    //--    Subscribers --//

    //Joint state subscriber
    ros::Subscriber joint_state_subs;

    void jointState_callback(const sensor_msgs::JointState &msg);


public:

    //------------------------------------------------------------------------
    //  Constructor de la clase
    //------------------------------------------------------------------------


    //Constructor por defecto -- no hay
    //CLeg();

    //Constructor de la clase
    hexapodo();

    //------------------------------------------------------------------------
    //  Metodos publicos
    //------------------------------------------------------------------------

    //Metodo que levanta el robot
    void stand_up(std::vector<double> patas_consignas_vector, std::vector<double> patas_posicion_actual);

    //Metodo que mueve las patas del robot
	bool movement(bool start_movement, int phase, int phase_mod);


}