
#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


#define ROBOT_LEG_NUM  6  //Numero de patas del robot RHEX

class altern_tripod_fem
{
	private:

        // stand up
        bool stand_up_position; // Flag que devuelve si el robot esta erguido -- posicion despues de iniciarse tumbado en el suelo
        std::vector<bool> legs_origin_position; // Vector que indica que las patas están en la posicion de erguido -- 0 grados == TRUE

		bool flag;	//flag que devuelve que se ha terminado una transicion
		int groups_quantity; // cantidad de grupos de patas
		int groups_length;	// longitud de los grupos de patas
        std::vector<int> tripod_one;	// Vector con las patas del primer tripode
        std::vector<int> tripod_two;	// Vector con las patas del segundo tripode
        std::vector<bool> leg_in_position_vector; // Vector que confirma que todas las patas estan en posicion
        bool leg_in_position[2][3] = {{false, false, false},{false, false, false}};

        // Joint State
        sensor_msgs::JointState legs_joint_state; // Vector que almacena la posicion de las patas con el mensaje del jointState
        std::vector<float> legs_actual_position; // Vector que almacena la posicion actual de cada pata -- su joint
        bool legs_joint_state_flag;  // Bandera que indica que si es TRUE se ha leido al menos una vez el joint_states

		// Fases de los ciclos
		int phase;
		float phase_module;

		// Angulos de posiciones de las patas
        float total_angle_degrees;
		float total_angle_rads;
		float take_off_angle;	// Angulo de despegue [rads]
		float take_land_angle;	// Angulo de aterrizaje [rads]

		// Velocidades
        float phase_one_vel;	// Velocidad de la fase suelo [rads/s]
        float phase_two_vel;	// Velocidad de la fase vuelo [rads/s]
        float phase_one_vel_old;
        float phase_two_vel_old;

        // Variables para la instancia move_legs
        bool start_movement_old;  // Variable que guarda el estado anterior de start_movement;

        // ROS

        ros::NodeHandle nh;  //Node handle
        ros::Subscriber joint_state_subs; //Joint state subscriber

		/*
		/* METODOS
		*/

        // void begin();	// Funcion para inicializar los parametros
		bool move_legs(bool); // Funcion para mover las patas segun la consigna de la fem	
        void init_joint_state_sub(); //Metodos que inicializan los subscriber
        void joint_state_callback(const sensor_msgs::JointStatePtr& msg); //Metodo que actualiza los campos correspondientes a las variables de estado de la junta
//        void setup_legs(ros::NodeHandle &nh); // Funcion setup de las patas


	public:
		bool exit; //Parametro que le dice si salir en el siguiente ciclo o seguir con los movimientos
		float total_angle; //Angulo total de la fase de suelo en grados [grados]
		float robot_speed;	// Velocidad de avance del cuerpo del robot [m/s]

        std::vector<int>legs_vector[2][6];

        //Patas del robot RHex
        CLeg *pata[ROBOT_LEG_NUM];

		/*
		/* FUNCIONES
		*/
        altern_tripod_fem(bool exit, float total_angle, float robot_speed);	//Constructor
        bool fem(bool); // Funcion que es la maquina virtual de las fases
        bool stand_up();  // Funcion para levantar el robot
        void vel_calcs(float);  // Funcion para calcular las velocidades de los ciclos de las patas segun la velocidad de avance del robot
        void setup_legs(ros::NodeHandle &nh);
};

