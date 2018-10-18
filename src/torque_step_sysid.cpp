
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
#define SUBSCRIBE_WHEEL_VEL "wheel_velocity"
#define BUFFER_SIZE 5
#define POWER_BUFFER_SIZE 200
#define LOOP_FREQ 100

//#define STEP_SIZE 100
#define START_TIME 2
//#define PULSE_TIME 10
#define DELAY_OFF 8
float pulse_time;

geometry_msgs::Twist pwr_msg;
ros::Publisher motor_power_pub;
ros::Subscriber wheel_vel_sub;

float current_L_vel = 0;
float current_R_vel = 0;

float step_size;

//clock_t t_start;
float t_elapsed = 0;
int loop_times = 0;
bool run = true;

// publish when new steering directions are set
void pubEnginePower()
{
	//t_elapsed = 100*((float)clock() - (float)t_start)/CLOCKS_PER_SEC; // in seconds

	t_elapsed = (float)loop_times/LOOP_FREQ ;

	if( t_elapsed < START_TIME){
		pwr_msg.linear.x = 0;
		pwr_msg.linear.y = 0;
	}
	else if(t_elapsed < pulse_time + START_TIME){
		pwr_msg.linear.x = step_size;
		pwr_msg.linear.y = step_size;
	}
	else if(t_elapsed < pulse_time + START_TIME + DELAY_OFF ){
		pwr_msg.linear.x = 0;
		pwr_msg.linear.y = 0;
	}
	else
		run = false;
	
	pwr_msg.linear.z = t_elapsed; //ros::Time::now().toSec();
	motor_power_pub.publish(pwr_msg);
	return;
}


void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    current_L_vel = array->data[0];
    current_R_vel = array->data[1];
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle nh(NODE_NAME);
  nh.param<float>("step_size",step_size,50.0);
  nh.param<float>("pulse_time",pulse_time,5.0);

  ros::Rate loop_rate(LOOP_FREQ);
  
  //set up communication channels
  // TODO add the other channels
  motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, POWER_BUFFER_SIZE);
  wheel_vel_sub = n.subscribe<std_msgs::Float32MultiArray>(SUBSCRIBE_WHEEL_VEL, BUFFER_SIZE, encoderCallback);
  
  float updatefreq = LOOP_FREQ;

  ROS_INFO("Data in format: t, L_pow, R_pow, L_vel, R_vel");
 // t_start = clock();
  int i = 1;
  while(ros::ok() && run){
	loop_times++;
	//std::cout << "yo\n";
	//ROS_INFO("yo");
	if( i == 5 ) {
		i = 0;
		pubEnginePower();
	}
	ROS_INFO("%f,%f,%f,%f,%f",(float)loop_times/LOOP_FREQ,pwr_msg.linear.x, pwr_msg.linear.y, current_L_vel, current_R_vel);
  	i++;
	ros::spinOnce();
  	loop_rate.sleep();

  }

  return 0;
}
