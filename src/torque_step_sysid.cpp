
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <time.h>
#include <math.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Bool.h"

//constant setup variables change those values here
#define NODE_NAME "torque_step_sysid"
#define ADVERTISE_POWER "motor_power" //publishing channel
#define POWER_BUFFER_SIZE 200
#define LOOP_FREQ 20

geometry_msgs::Twist pwr_msg;
ros::Publisher motor_power_pub;


#define STEP_SIZE 0.5
#define START_TIME 3
#define PULSE_TIME 5

clock_t t_start;
float t_elapsed = 0;

// publish when new steering directions are set
void pubEnginePower()
{
	t_elapsed = 100*((float)clock() - (float)t_start)/CLOCKS_PER_SEC; // in seconds

	if( t_elapsed < 3){
		pwr_msg.linear.x = 0;
		pwr_msg.linear.y = 0;
	}
	else if(t_elapsed < PULSE_TIME + START_TIME){
		pwr_msg.linear.x = STEP_SIZE;
		pwr_msg.linear.y = STEP_SIZE;
	}
	else {
		pwr_msg.linear.x = 0;
		pwr_msg.linear.y = 0;
	}
	
	pwr_msg.linear.z = t_elapsed; //ros::Time::now().toSec();
	motor_power_pub.publish(pwr_msg);
	return;
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_FREQ);
  
  //set up communication channels
  // TODO add the other channels
  motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, POWER_BUFFER_SIZE);
  
  float updatefreq = LOOP_FREQ;

  t_start = clock();

  while(ros::ok()){

	pubEnginePower();
  	ros::spinOnce();
  	loop_rate.sleep();

  }

  return 0;
}
