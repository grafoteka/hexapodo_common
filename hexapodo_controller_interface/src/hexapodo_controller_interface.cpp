#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/LoadController.h"
#include <string>
#include <hexapodo_controller_interface/CLegCommand.h>

using namespace std;

//------------------------------------------------------------------------------------
//Global variables
//------------------------------------------------------------------------------------

//Node's services
ros::ServiceServer c_leg_command_server;

//Node's client to the swapping service
ros::ServiceClient swap_controller_client;
ros::ServiceClient load_controller_client;

//The joint's name must be passed by an argument to the node otherwise the node throws an error, thus ends
//execution
string jointName;

//Node's publisher about the position controller command
ros::Publisher position_controller_command_pub;

//Node's publisher about the velocity controller command
ros::Publisher velocity_controller_command_pub;

//Fields containing the current command being sent to each controller
float position_command_value = 0;
float velocity_command_value = 0;
bool position_control =  false;

//------------------------------------------------------------------------------------
//Callback functions
//------------------------------------------------------------------------------------

//Callback for Command over C-Leg service
bool newCommandServiceCallback (hexapodo_controller_interface::CLegCommand::Request &req,
                                hexapodo_controller_interface::CLegCommand::Response &res){

    //Swaps the controller by the service of the controller_manager
    //Sets the service msg to be requested
    controller_manager_msgs::SwitchController switch_controller_srv;

    if(req.position_control){
      //The command belongs to the position control type
      position_command_value = req.value;
    }else{
      velocity_command_value = req.value;
    }

    //If the velocity controller is switched for the position controller
    if (req.position_control && (!position_control)){

        //Constructs the switch controller service call
        switch_controller_srv.request.start_controllers   =   {jointName + "_position_controller"};
        switch_controller_srv.request.stop_controllers    =   {jointName + "_velocity_controller"};
        switch_controller_srv.request.strictness = 2;

        //Calling to the service
        swap_controller_client.call(switch_controller_srv);

        //Informs whether the swap has been carried out successfully
        if (switch_controller_srv.response.ok){
            ROS_INFO("Controllers swapped successfully");
            res.ok = true;
            position_control = true;
        }else{
            ROS_ERROR("Couldn't swap controllers");
            res.ok = false;
        }

        return true;
    }

    //If the position controller is switched for the velocity controller
    if ((!req.position_control) && (position_control)){

        //Constructs the switch controller service call
        switch_controller_srv.request.start_controllers   =   {jointName + "_velocity_controller"};
        switch_controller_srv.request.stop_controllers    =   {jointName + "_position_controller"};
        switch_controller_srv.request.strictness = 2;

        //Calling to the service
        swap_controller_client.call(switch_controller_srv);

        //Informs whether the swap has been carried out successfully
        if (switch_controller_srv.response.ok){
            ROS_INFO("Controllers swapped successfully");
            res.ok = true;
            position_control = false;
        }else{
            ROS_ERROR("Couldn't swap controllers");
            res.ok = false;
        }

        return true;
    }

    //If there's no need in changing the controllers, the answer is true.
    res.ok = true;

    //End of callback
    return true;
}

//-------------------------------------------------------------------------------------
//  Other functions
//-------------------------------------------------------------------------------------

//Advertise the position and velocity topics towards each controller
void advertisePublishers(ros::NodeHandle &nh){
    //In the following lines the publisher are advertise

    //The publisers are:
    //  - Position command publisher
    //  - Velocity command publisher
    position_controller_command_pub = nh.advertise<std_msgs::Float64>("/hexapodo/" + jointName + "_position_controller/command", 1000);
    velocity_controller_command_pub = nh.advertise<std_msgs::Float64>("/hexapodo/" + jointName + "_velocity_controller/command", 1000);
}

//Publish the messages needed
void publishCommands (){

  //Message sent as command
  std_msgs::Float64 command_msg;

  //Depending on the control type, the position or the velocity command are published
  if (position_control){
    command_msg.data = position_command_value;
    position_controller_command_pub.publish(command_msg);
  }else{
    command_msg.data = velocity_command_value;
    velocity_controller_command_pub.publish(command_msg);
  }

  return;

}

//Advertise the node's services
void advertiseServices(ros::NodeHandle &nh){
  //In the following lines, the nodes's services are going to be published

  //The sole server is the c_leg_command_srv
  c_leg_command_server = nh.advertiseService("/hexapodo/" + jointName + "_controller_interface/c_leg_command", newCommandServiceCallback);

  return;

}

//Advertise the node's clients
void advertiseClients(ros::NodeHandle &nh){
  swap_controller_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/hexapodo/controller_manager/switch_controller");
  load_controller_client = nh.serviceClient<controller_manager_msgs::LoadController>("/hexapodo/controller_manager/load_controller");

  return;
}

//-------------------------------------------------------------------------------------
//main function
//-------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

    //----------------------------------------------------------------
    //ROS and node's configuration
    //----------------------------------------------------------------

    // Set up ROS.
    ros::init(argc, argv, "hexapodo_controller_interface");

    //Instantiation of the node handle
    ros::NodeHandle nh;

    //----------------------------------------------------------------
    //Rate controller
    //----------------------------------------------------------------

    //Each loop should be executed each 0.01 sec
    ros::Rate rate_loop(100);

    //----------------------------------------------------------------
    //  Messages and services
    //----------------------------------------------------------------

    //Load controller service
    controller_manager_msgs::LoadController load_controller_srv;

    //----------------------------------------------------------------
    //Node's main process
    //----------------------------------------------------------------

    //Check whether it is a suitable name for the joint, if so, sets the joint's name, subscribes to the command topic,
    //sets the node's clients and advertises the service command.
    if (!argc){
        ROS_ERROR("No leg name given, node will not start");
        return 0;
    }else{
        jointName = argv[1];
        advertiseClients(nh);
        advertisePublishers(nh);
        advertiseServices(nh);
    }

    //service message for loading the controllers is built during this code segment
    //string name
    //---
    //bool ok

    if(position_control){
        load_controller_srv.request.name = jointName + "_velocity_controller";

        //Calling the service for loading the velocity controller for the joint
        //  Informs whether the controller was successfully loaded or not
        load_controller_client.call(load_controller_srv);
        if (load_controller_srv.response.ok){
            string info = load_controller_srv.request.name + " loaded successfully";
            ROS_INFO(info.c_str());
        }else{
            string error = "Couldn't load " + load_controller_srv.request.name;
            ROS_ERROR(error.c_str());
        }
    }else{
        load_controller_srv.request.name = jointName + "_position_controller";

        //Calling the service for loading the position controller for the joint
        //  Informs whether the controller was successfully loaded or not
        load_controller_client.call(load_controller_srv);
        if (load_controller_srv.response.ok){
            string info = load_controller_srv.request.name + " loaded successfully";
            ROS_INFO(info.c_str());
        }else{
            string error = "Couldn't load " + load_controller_srv.request.name;
            ROS_ERROR(error.c_str());
        }
    }

    //---------------------------------------------------------------
    //  Main node loop
    //---------------------------------------------------------------

    while (ros::ok()){
        //Depending on the controller set, the command for each controller is sent
        publishCommands();

        //Spins for messages and services
        ros::spinOnce();

        //Sleep until the next loop
        rate_loop.sleep();
    }

    return 0;
}
