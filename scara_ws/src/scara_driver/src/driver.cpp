#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16.h"
#include "math.h"
#include <string.h>    
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <cstdlib>
#include <boost/lexical_cast.hpp>

//This node handles communication with the arm and gripper over the serial port

using namespace std;

//Template used to give std::to_string function (see references)
template <typename T>
std::string to_string(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

string use_port = "/dev/ttyUSB0";
int use_baud = 57600;
bool update = false;
float gripper_yaw_offset; //variable used to offset gripper rotation
serial::Serial ser;	  //declare serial port object

float gripper_scale = 18820; 			//gripper scale, offset used to convert from metres to gripper potentiomer readings
float gripper_offset = -51.791;
float target[4] = {0, 0, 0, 0}; 		//target joint values for manipulator
float current_joint_state[4] = {0, 0, 0, 0};	//current joint values, as reported by GRBL
float total_error = 0; 
float  low_lim[5] = {-270, -270, 0,  -180, 0  };//x,y,z,a upper and lower limits
float high_lim[5] = {   0,    0, 220,   180, 770};  

//sends strings recieved on the write topic directly
void write_callback(const std_msgs::String::ConstPtr& msg){

	cout << msg->data << endl;

	if (msg->data == "run"){
		update = true;
		cout << "Updating Jointstates from GRBL.." << endl;
	}

	if (msg->data == "stop"){
		update = false;
		cout << "Not updating Jointstates" << endl;
	}

	string to_write = msg->data  + "\r" + "\n";
	ser.write(to_write);
}


//callback triggered when target joint states change, passes target joint states to GRBL as G-CODE
void arm_cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
	
	vector<string> names = cmd_arm.name; //strings of joint names with new targets
	
	string gcode = "#G1 F2200 "; //'#' prefix specifies command for GRBL, G1 F2200 specifies controlled motion and velocity 
	bool send_gcode = false;     //won't send G-CODE if a valid command cannot be interpreted
	
	string gripper_cmd = "&C";   //'&' prefix specifies command for gripper
	bool send_gripper_position = false; //won't send by default

	for (int i=0; i<names.size(); i++){ //for every joint with new target
		
		//if J2, XJ
		if (names[i] == "XJ"){ 
			     float joint_val = 57.2958*cmd_arm.position[i]; 			//convert to degrees
			     joint_val = roundf(joint_val * 1000) / 1000;			//allow 3 decimal places

			     if((joint_val <= high_lim[0])&&(joint_val >= low_lim[0])){		//if within limits
				     gcode += " X" + to_string(joint_val);			//construct G-CODE string
				     target[0] = joint_val;					//store as new joint target
				     send_gcode = true;						//send G-CODE later
				}
			      else{
				cout << "Limits exceeded:" <<  names[i] << joint_val << endl;	//don't send any thing if limits exceeded, report error
				send_gcode = false;
				break;
				}
		}
		
		//repeat same process for all other joints of manipulator (YJ, ZJ, AJ)

		//if J3, YJ
		else if (names[i] == "YJ"){
			     float joint_val = 57.2958*cmd_arm.position[i];
			     joint_val = roundf(joint_val * 1000) / 1000;

			     if((joint_val <= high_lim[1])&&(joint_val >= low_lim[1])){
				     gcode += " Y" + to_string(joint_val);
				     target[1] = joint_val;
				     send_gcode = true;
				}
			      else{
				cout << "Limits exceeded:" <<  names[i] << joint_val << endl;
				send_gcode = false;
				break;
				}
		}

		//if J1, ZJ
		else if (names[i] == "ZJ"){
			     float joint_val = 1000*cmd_arm.position[i];
			     joint_val = roundf(joint_val * 10) / 10;

			     if((joint_val <= high_lim[2])&&(joint_val >= low_lim[2])){
				     gcode += " Z" + to_string(joint_val);
				     target[2] = joint_val;
				     send_gcode = true;
				}
			      else{
				cout << "Limits exceeded:" <<  names[i] << joint_val << endl;
				send_gcode = false;
				break;
				}
		}

		//if J4, AJ
		else if (names[i] == "AJ"){
			     float joint_val = 57.2958*(cmd_arm.position[i] - gripper_yaw_offset);
			     joint_val = roundf(joint_val * 10) / 10;		//round to 1dp (0.1mm)

			     if((joint_val <= high_lim[3])&&(joint_val >= low_lim[3])){
				     gcode += " A" + to_string(joint_val);
				     target[3] = joint_val;
				     send_gcode = true;
				}
			      else{
				cout << "Limits exceeded:" <<  names[i] << joint_val << endl;
				send_gcode = false;
				break;
				}
		}

		//if gripper jaw, G0
		else if (names[i] == "G0"){
			    float joint_val = gripper_scale*cmd_arm.position[i] + gripper_offset; //if gripper target specified, apply scaling
			    joint_val = roundf(joint_val);					  //round to 1dp

			    if((joint_val <= high_lim[4])&&(joint_val >= low_lim[4])){
				    gripper_cmd += to_string(joint_val);
				    cout << "gripper_cmd:" <<  gripper_cmd << endl;
				    send_gripper_position = true;
				}
			      else{
				cout << "Limits exceeded:" <<  names[i] << joint_val << endl;
				send_gcode = false;
				break;
				}
		}

		//G1 not used as the gripper jaws are mirrored
		else if (names[i] == "G1"){
		}

		//catch case- Don't send any commands
		else {
		      cout << "Unknown joint name recieved:" <<  names[i] << endl;
		      send_gcode = false;
		      send_gripper_position = false;
		}
	}

    //if a valid GRBL command was constructed, write it to the serial port
    if(send_gcode == true){
	cout << gcode << endl;
	ser.write(gcode);
    ser.write("\r");
	}

    //if a valid gripper command was constructed, write it to the serial port
    else if(send_gripper_position == true){
	cout << gripper_cmd << endl;
	ser.write(gripper_cmd);
	ser.write("\r");
        ser.write("\n");
	}
}

//main initializes node, serial port, then recieves feedback from GRBL and gripper
//sending commands are handled by the callbacks
int main (int argc, char** argv){
    gripper_yaw_offset = 0;
    std_msgs::Char gripper_status;
    gripper_status.data = 'U'; //gripper status initialized as unknown
    ros::init(argc, argv, "scara_driver");
    ros::NodeHandle nh;

    ros::Subscriber joint_sub     = nh.subscribe("target_joint_states",1000, arm_cmd_cb);
    ros::Subscriber write_sub     = nh.subscribe("scara_write", 1000, write_callback);
    ros::Publisher  feedback_pub  = nh.advertise<sensor_msgs::JointState>("/update/joint_states", 1);
    ros::Publisher  grip_state_pub     = nh.advertise<std_msgs::Char>("/grip_status", 1);
    ros::Publisher  error_pub     = nh.advertise<std_msgs::Float32>("/total_joint_error", 1);

	//try to open serial port, uses 57600 baud
    try
    {
        ser.setPort(use_port);
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

		
    ros::Rate loop_rate(10); //loop at 10hz
    loop_rate.sleep();

	cout << "Printing responses from GRBL and the gripper controller" << endl;
	cout << endl;

	while(ros::ok()){//enter main loop
        ros::spinOnce();

			if (update == true){ 
				ser.write("#?"); //'?' prompts GRBL to report motor positions - sent at 10hz
				ser.write("\r");
				}


	if(ser.available()){ //if anything waiting in serial buffer

		std_msgs::String result;	//put in string 'recieved'
		result.data = ser.read(ser.available());
		string recieved = result.data;
		
		if(recieved[0] == '&'){ //prefix means message came from the gripper

			
			if     (recieved[1] == 'C'){ //gripper status C: closure sucessful
				gripper_status.data = 'C';
				grip_state_pub.publish(gripper_status);
			}
			else if(recieved[1] == 'F'){ //gripper status F: closure or PID failed
				gripper_status.data = 'F';
				grip_state_pub.publish(gripper_status);
			}
			else if(recieved[1] == 'T'){ //gripper status T: PID target reached
				gripper_status.data = 'T';
				grip_state_pub.publish(gripper_status);
				
			}
			else if(recieved[1] == 'S'){ //gripper status S: returned sensor value from gripper
				int gripper_sensor_val =  atoi((recieved.substr(2,4)).c_str()); //convert sensor value to integer

				sensor_msgs::JointState joint_state; //construct a joint state message to update the gripper jaw positions
				joint_state.name.resize(2);
				joint_state.position.resize(2);
				joint_state.header.stamp = ros::Time::now();
				joint_state.name[0] = "G0";
				joint_state.name[1] = "G1";
				joint_state.position[0] = (float(gripper_sensor_val) - gripper_offset)/gripper_scale; //convert sensor value to metres
				joint_state.position[1] = joint_state.position[0]; //mirrored jaws are always equal
				feedback_pub.publish(joint_state); //publish to update_joint_states

			}
		
		}
		
		
		//no & prefix means message came from GRBL
		bool display = true;
		std::size_t found = recieved.find("MPos:"); //phrase MPos is followed by motor positions reported by GRBL
		if (found!=std::string::npos){	//if phrase found
			string feedback = recieved.substr ((int(found)+5), 100); //substring containing the motor (joint) positions
			vector<string> str_joint_angles;
			boost::split(str_joint_angles, feedback, boost::is_any_of(",|")); //split into individual string for each joint
		
			if(str_joint_angles.size() > 4){ //otherwise only partial message recieved, which should not be used
				float joint_angles[4];  //put each joint value into float array
				for(int i = 0; i < 4; i++){
					joint_angles[i] = atof(str_joint_angles[i].c_str());
					}

				nh.param("gripper_yaw_offset", gripper_yaw_offset, float(0)); //get the gripper yaw offset parameter

				total_error = 0;
				for(int i = 0; i < 4; i++){ //compute total manipulator joint error as sum of absolute error
					float joint_error = target[i] - joint_angles[i];
					if (joint_error < 0){
						joint_error = -joint_error;
					}
					total_error += joint_error; 
				}

				std_msgs::Float32 error;
				error.data = total_error;
				error_pub.publish(error); //publish computed error on total_joint_error topic
				
				//construct a jointstate message updating all manipulator joints
				sensor_msgs::JointState joint_state;
				joint_state.name.resize(4);
				joint_state.position.resize(4);
				joint_state.header.stamp = ros::Time::now();
				joint_state.name[0] = "XJ";
				joint_state.name[1] = "YJ";
				joint_state.name[2] = "ZJ";
				joint_state.name[3] = "AJ";
				joint_state.position[0] = joint_angles[0]/57.2958; //convert to radians for XJ, YJ
				joint_state.position[1] = joint_angles[1]/57.2958;
				joint_state.position[2] = joint_angles[2]/1000;   //convert to metres for prismatic ZJ
				joint_state.position[3] = (joint_angles[3]/57.2958) + gripper_yaw_offset; //apply gripper yaw offset to AJ
				feedback_pub.publish(joint_state); //publish to update_joint_states

				if (update == true){ 
					display = false; //don't print recieved message on screen as motor positions are requested from GRBL at 10hz
				}
			}
		}

		if (display == true){ //do display messages from GRBL that are not reporting motor positions, as these may indicate errors
		cout << recieved << endl;
		}

        }
        loop_rate.sleep();
    }
}

/* References-

Implimentation of std::to_string for C++ 11 (lines 19-25):
https://stackoverflow.com/questions/19122574/to-string-isnt-a-member-of-std

https://answers.ros.org/question/43157/trying-to-use-get-joint-state-with-my-urdf/
*/






