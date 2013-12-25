/******************************************************************************
 * 
 * Author:
 * Adam Jardim
 *
 *	This function creates and loads the controller manager for the youbot to 
 *	allow the control of its joints and motors.  It uses the pr2_controller_manager
 *	to interface with MoveIt! to allow for arm trajectory planning.
 *	
 ******************************************************************************/
#include <pr2_controller_manager/controller_manager.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "youbot_controller_manager");
	ros::NodeHandle node;

	//Hardware interface for the youbot
	//It will describe the actuators and joints of the youbot
	//These are described more directly in the youbot_description URDFs
	pr2_hardware_interface::HardwareInterface hw;
	
	//Gripper Actuators
	hw.actuators_["gripper_finger_l_motor"] = new pr2_hardware_interface::Actuator("gripper_finger_l_motor");
	hw.actuators_["gripper_finger_r_motor"] = new pr2_hardware_interface::Actuator("gripper_finger_r_motor");

	//Arm Actuators
	hw.actuators_["arm_motor_1"] = new pr2_hardware_interface::Actuator("arm_motor_1");
	hw.actuators_["arm_motor_2"] = new pr2_hardware_interface::Actuator("arm_motor_2");
	hw.actuators_["arm_motor_3"] = new pr2_hardware_interface::Actuator("arm_motor_3");
	hw.actuators_["arm_motor_4"] = new pr2_hardware_interface::Actuator("arm_motor_4");
	hw.actuators_["arm_motor_5"] = new pr2_hardware_interface::Actuator("arm_motor_5");

	//Base Actuators
	hw.actuators_["wheel_motor_fl"] = new pr2_hardware_interface::Actuator("wheel_motor_fl");
	hw.actuators_["wheel_motor_fr"] = new pr2_hardware_interface::Actuator("wheel_motor_fr");
	hw.actuators_["wheel_motor_bl"] = new pr2_hardware_interface::Actuator("wheel_motor_bl");
	hw.actuators_["wheel_motor_br"] = new pr2_hardware_interface::Actuator("wheel_motor_br");
	hw.actuators_["caster_motor_fl"] = new pr2_hardware_interface::Actuator("caster_motor_fl");
	hw.actuators_["caster_motor_fr"] = new pr2_hardware_interface::Actuator("caster_motor_fr");
	hw.actuators_["caster_motor_bl"] = new pr2_hardware_interface::Actuator("caster_motor_bl");
	hw.actuators_["caster_motor_br"] = new pr2_hardware_interface::Actuator("caster_motor_br");

	ROS_INFO("YouBot actuators set!");

	pr2_controller_manager::ControllerManager cm(&hw, node);

	ROS_INFO("YouBot_controller_manager initialized!");

	// read robot description from parameter server
	std::string robot_description_string;
	TiXmlDocument robot_description_xml;
	if (node.getParam("robot_description", robot_description_string))
		robot_description_xml.Parse(robot_description_string.c_str());
	else
	{
		ROS_ERROR("Could not load the robot description from the parameter server");
		return -1;
	}
		TiXmlElement *robot_description_root = robot_description_xml.FirstChildElement("robot");
	if (!robot_description_root)
	{
		ROS_ERROR("Could not parse the robot description");
		return -1;
	}

	// Initialize controller manager from robot description
	if (!cm.initXml(robot_description_root)){
		ROS_ERROR("Could not initialize controller manager");
		return -1;
	}

	int timer = 0;
  bool unloaded = true;

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate rate(1000.0);
	
	ROS_INFO("YouBot controller manager starting!");

  pr2_controller_interface::Controller* check1 = NULL;
  
	while (ros::ok()){
		//ROS_INFO("BEEP");
  
    if(unloaded)
    {
      check1 = cm.getControllerByName("arm_1/arm_controller");
    }    

    if(timer < 2000)
    {
      if((check1 != NULL))
      {
        unloaded = false;
      } 
      cm.update();
    }

    if((unloaded == false)&&(ros::ok()))
    {
      timer++;
    }

    if(timer == 2000)
    {
      ROS_INFO("Arm Controller loaded.");
    }

    if(!ros::ok())
      break;

		rate.sleep();
	}

  spinner.stop();

	return 0;
}


