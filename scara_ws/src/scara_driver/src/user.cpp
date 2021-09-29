#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "math.h"
#include <string.h>     // std::string, std::to_string
#include <sstream>
#include <iostream>
using namespace std;

template <typename T>
std::string to_string(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}
	
int main (int argc, char** argv){
    ros::init(argc, argv, "user_coms");
    ros::NodeHandle nh;
    ros::Publisher send_pub = nh.advertise<std_msgs::String>("scara_write", 1000);

	cout << "Use this terminal to pass user commands to GRBL or the gripper" << endl;
	cout << "Prefix GRBL commands with '#'" << endl;
	cout << "Prefix gripper commands with '&'" << endl;
	cout << "Use '&E' to enable, '&D' to disable" << endl;
	cout << endl;
	
while(ros::ok()){	

ros::spinOnce();
string user_input;
cin >> user_input;
std_msgs::String to_send;
to_send.data = user_input;
send_pub.publish(to_send);

    }
}




