#include <c_leg/c_leg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class robot_configuration
{
private:

  #define ROBOT_LEG_NUM 6 // Numero de patas del robot
  // Patas del robot
  CLeg *pata[ROBOT_LEG_NUM];

  // libreria CLegs
  void setup_legs(ros::NodeHandle &nh);  // Inicializacion de las patas
  void delete_legs(); // Eliminacion de las patas

  // Joint State
  sensor_msgs::JointState legs_joint_state; // Vector que almacena la posicion de las patas con el mensaje del jointState
  bool legs_joint_state_flag;  // Bandera que indica que si es TRUE se ha leido al menos una vez el joint_states

  // ROS
  ros::NodeHandle nh;  //Node handle
  ros::Subscriber joint_state_subs; //Joint state subscriber

  void init_joint_state_sub(); //Metodos que inicializan los subscriber
  void joint_state_callback(const sensor_msgs::JointStatePtr& msg); //Metodo que actualiza los campos correspondientes a las variables de estado de la junta

  // Distintos modos de marcha a implementar
  enum walking_modes {
    stand_by,
    altern_tripod,
    all,
    wave,
    quadruped,
    pronking
  };
  int npatas_;

  walking_modes walking_mode_actual;

public:


  robot_configuration(); // Constructor
//  ~robot_configuration();

  std::vector<float> legs_actual_position; // Vector que almacena la posicion actual de cada pata -- su joint

  void stand_up();
  void walking_mode_fsm(int modo);
  int npatas() const;
  void setNpatas(int npatas);
};
