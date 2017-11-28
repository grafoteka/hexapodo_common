
#include <ros/ros.h>
#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sstream>


/******************************************************
 * Declaraciones
 ******************************************************/
//Declaracion del vector que lee los datos del jointState
sensor_msgs::JointState pos_joints;

// Declaracion del publisher
//ros::Publisher velocity_controller_command_pub;

// Declaracion del vector para el publisher de cada pata
std::vector<ros::Publisher> leg_command_pub;



/*
void callback(const sensor_msgs::JointStatePtr& msg)
{
  //ROS_INFO("n joints %d", msg->position.size());
  pos_joints = *msg;
  //pos_patas.clear();
  //ROS_INFO("I heard: [%f]", pata_1);
}
*/

/******************************************************
 * Función que envía las consignas a las patas.
 ******************************************************/
void consignas_patas()
{
  std_msgs::Float64 leg_1_vel;
  leg_1_vel.data= 100.0;
  ROS_INFO("He entrado");
  if(pos_joints.position.at(1) < 5)
  {
    ROS_INFO("Esto ha entrado tb");
    leg_command_pub.at(1).publish(leg_1_vel);
  }
}


/******************************************************
 * Función que se llama periódicamente para obtener la posición de los joints.
 ******************************************************/
void callback_timer(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
  for (int i = 0; i < pos_joints.position.size(); i++)
  {
    ROS_INFO("joint %d name %s", i, pos_joints.name.at(i).c_str());
    ROS_INFO("joint %d position  %.2f", i, pos_joints.position.at(i));
    //pos_patas.push_back(msg->position.at(i));

    if(i == 5)
    {
      consignas_patas();
    }
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_commands");
  ros::NodeHandle node_sub;
  ros::NodeHandle node_pub;

  ros::Rate rate(1.0);

//  tf::TransformListener listener;
//  tf::StampedTransform transform;
  //tf::TransformListener tf;
  //tf::TransformBroadcaster broadcaster;

  //sensor_msgs::JointState pata_c_6;
  //ssensor_msgs::JointState base_link_1;

  ros::Subscriber sub = node_sub.subscribe("/joint_states", 1, callback);
  leg_command_pub.resize(6);
  for(int i = 0; i < 6; i++ )
  {
    std::stringstream topic_name;

    topic_name << "/pata" << i+1 << "_pata_c_link/command";
    leg_command_pub.at(i) = node_pub.advertise<std_msgs::Float64>(topic_name.str(), 1);
  }


  //Node's publisher about the velocity controller command

  ros::Timer timer2 = node.createTimer(ros::Duration(1.0), callback_timer);
  //velocity_controller_command_pub = n.advertise<std_msgs::Float64>("/pata" + pos_joints + "_pata_c_link/command", 1000);
  //pos_patas.resize(6);
  ros::spin();


//  while(node.ok()){
//    try
//    {

//        ros::Time now = ros::Time::now();
//        listener.waitForTransform("/pata_6_pata_c_link", "/base_link", now, ros::Duration(1.0));
//        listener.lookupTransform("/pata_6_pata_c_link", "/base_link", now, transform);
//        //listener.transformPose("/pata_6_c_link", ros::Time(0), base_link_1, "/base_link", pata_c_6);

//    }

//    catch (tf::TransformException ex)
//    {
//        ROS_ERROR("%s",ex.what());
//        ros::Duration(1.0).sleep();
//        continue;
//    }
//    ROS_INFO("transform: X %.2f -  Y %.2f", transform.getOrigin().getX(), transform.getOrigin().getY());
//    ros::spinOnce();
//
//    rate.sleep();
//  }
  return 0;
}
