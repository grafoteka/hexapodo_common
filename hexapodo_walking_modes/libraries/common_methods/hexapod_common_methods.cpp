#include <common_methods/hexapod_common_methods.h>

/******************************************************
 * Getters
 ******************************************************/
float common_methods::phase_two_vel() const
{
  return phase_two_vel_;
}

float common_methods::phase_one_vel() const
{
  return phase_one_vel_;
}

bool common_methods::advance() const
{
  return advance_;
}

bool common_methods::reverse() const
{
  return reverse_;
}

bool common_methods::turn_left() const
{
  return turn_left_;
}

bool common_methods::turn_right() const
{
  return turn_right_;
}
/******************************************************
 * Setup patas
 ******************************************************/
//Funcion setup de las patas
void common_methods::setup_legs(ros::NodeHandle &nh){
//  ROS_INFO("Entro en setup_patas");
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    pata[i] = new CLeg(i + 1, nh);
    ROS_INFO("Pata numero %d ha sido inicializada", i);
  }

  return;
}

//Funcion que libera el espacio en memoria de las patas
void common_methods::delete_legs (){
  for (int i = 0; i < ROBOT_LEG_NUM; i++){
    delete pata[i];
  }
  return;
}

/******************************************************
 * Calculo de velocidades
 ******************************************************/
void common_methods::velocity_calcule()
{
  const float leg_diameter = 0.200;
  float alpha = total_angle_degrees_;
  float lStep = (2 * PI * alpha * leg_diameter) / 360;
  float totSteps = (1/velocity_robot_) / lStep;
  float steps_per_second = 1 / totSteps;
  float alpha_rads = alpha * PI / 180;
  static float phase_one_vel = 0.0;
  static float phase_one_vel_old = 0.0;
  static float phase_two_vel = 0.0;
  static float phase_two_vel_old = 0.0;

  phase_one_vel = alpha_rads * (steps_per_second * 100);
//    ROS_INFO("phase one velocity = %.2f, robot speed = %.2f", phase_one_vel, robot_speed);

    // Calculo de la velocidad de vuelo segun el porcentaje de seguridad
  const float phase_two_vel_security = 0.9;
  phase_two_vel = (2 * PI - alpha_rads) * (steps_per_second * phase_two_vel_security * 100);
  //  phase_one_vel = ground_vel_rads;
  //  phase_two_vel = fly_vel_rads;
  if ((phase_one_vel != phase_one_vel_old) || (phase_two_vel != phase_two_vel_old))
  {
    phase_one_vel_old = phase_one_vel;
    phase_two_vel_old = phase_two_vel;
    ROS_INFO("\n velocidad = %.2f \n phase 1 velocity = %.2f  \n phase 2 velocity = %.2f", velocity_robot_, phase_one_vel, phase_two_vel);
  }
}

/******************************************************
 * Confirmacion de la lectura de las patas
 ******************************************************/
// Esta funcion devuelve:
// TRUE -> si las patas ya han sido leidas en la libreria CLeg
// FALSE -> si no han sido leido todavia. En cuyo caso las vuelve a leer
bool common_methods::legs_readed(){
  if((pata[0]->joint_state_read_flag) &&
     (pata[1]->joint_state_read_flag) &&
     (pata[2]->joint_state_read_flag) &&
     (pata[3]->joint_state_read_flag) &&
     (pata[4]->joint_state_read_flag) &&
     (pata[5]->joint_state_read_flag))
  {
//    ROS_INFO("Patas inicializadas correctamente");
    for(int i = 0; i < ROBOT_LEG_NUM; i++)
    {
//      ROS_INFO("Posicion de la pata %d -- %.2f", i, pata[i]->getPos());
    }

    return true;
  }

  else{
    for(int i = 0; i < ROBOT_LEG_NUM; i++){
      pata[i]->getPos();
    }
    return false;
  }

}


/******************************************************
 * Callbacks
 ******************************************************/

// La suscripcion a cmd_vel nos indica la velocidad lineal y angular segun el joystick del mando
// La suscripcion a joy nos permite leer los botones y ejes
void common_methods::init_subscribers()
{
  velocity_subs = nh.subscribe("/cmd_vel", 1, &common_methods::cmd_vel_callback, this);
  joystick_subs = nh.subscribe("/joy", 1, &common_methods::joystick_callback, this);
}


void common_methods::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &vel)
{
//  static bool avance = false;
  static float linear_old = 0.0;
  static float linear_actual = 0.0;
  static float angular_old = 0.0;
  static float angular_actual = 0.0;

  geometry_msgs::Twist new_vel;   //Variable para guardar las velocidades del joystick
  linear_actual = vel->linear.x;
  angular_actual = vel->angular.z;

  if(linear_actual == 0){
    reverse_ = false;
    advance_ = false;
  }
  else if (linear_actual < 0){
    advance_ = false;
    reverse_ = true;
  }
  else if (linear_actual > 0){
    advance_ = true;
    reverse_ = false;
  }

  if(angular_actual == 0){
    turn_left_ = false;
    turn_right_ = false;
  }
  else if(angular_actual < 0){
    turn_left_ = true;
    turn_right_ = false;
  }
  else if(angular_actual > 0){
    turn_left_ = false;
    turn_right_ = true;
  }

  if((linear_old != linear_actual) || (angular_old != angular_actual))
  {
    //ROS_INFO("velocidad lineal: %.2f -- velocidad angular: %.2f", linear_actual, angular_actual);
    linear_old = linear_actual;
    angular_old = angular_actual;
  }
}

void common_methods::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  static bool eStop = true;
  const int eStop_button = 5; // Boton del Joystick que es R1
  const int exit_button = 2; // Boton del Joystick que es el circulo
  static float velocity = 0.2;

  // Velocidad segun el numero de Froude
  // Para una pata con un diametro de 200[mm] = 1.4
  // Para una pata con un diametro de 150[mm] = 1.21
  const float speed_max = 0.75; //0.75
  const float speed_min = 0.0;
  float vel_increase = 0.01;


  // Si se pulsa triangulo
  if((joy->buttons[3] == 1) && (joy->buttons[eStop_button] == 1))
  {
    if(velocity < speed_max){
      velocity = velocity + vel_increase;
      ROS_INFO("velocidad de avance [m/s]: %.2f", velocity);
    }
  }

  // Si se pulsa cuadrado
  if((joy->buttons[0] == 1) && (joy->buttons[eStop_button] == 1))
  {
    if(velocity >= speed_min){
      velocity = velocity - vel_increase;
      ROS_INFO("velocidad de avance [m/s]: %.2f", velocity);
    }
  }

  velocity_robot_ = velocity;

  // Boton R1
  if(joy->buttons[eStop_button] == 1)
  {
    eStop = false;
    eStop_ = false;
  }
  else
  {
    eStop = true;
    eStop_ = true;
  }

  // Boton circulo -> cambio modo de caminar
  if(joy->buttons[exit_button] == 1)
  {
    exit_button_ = false;
  }
  else
  {
    exit_button_ = true;
  }

  velocity_calcule();
}


/******************************************************
 * Constructor
 ******************************************************/
common_methods::common_methods()
{
  ROS_INFO("Entro en el constructor");
  init_subscribers();

  setup_legs(nh);

  legs_readed_flag = false;

  legs_readed_flag = legs_readed();


  ROS_INFO("Constructor iniciado");

}

/******************************************************
 * Destructor
 ******************************************************/
//common_methods::~common_methods()
//{
//  delete_legs();

//}
