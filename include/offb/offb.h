 #ifndef OFFB_H
#define OFFB_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>



geometry_msgs::PoseStamped cur_local;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::State cur_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped targetLocal;

ros::Publisher local_pos_pub;

ros::Subscriber state_sub;
ros::Subscriber set_position;
ros::Subscriber cur_local_sub;

ros::Subscriber target_position;
ros::Subscriber target_yaw;
ros::Subscriber rel_Yaw_sub;
ros::Subscriber Lookat;
ros::Subscriber Lookat_pctrl;
ros::Subscriber bf_position;
ros::Subscriber bf_pos_pctrl;
ros::Subscriber bf_yaw_pctrl;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;


void setYaw(double);
void setPosition(double,double,double);
double yawfromQuaternion(double,double,double,double);
void bodyframe(double,double);


//UAV1
geometry_msgs::PoseStamped cur_local2;
mavros_msgs::SetMode offb_set_mode2;
mavros_msgs::CommandBool arm_cmd2;
mavros_msgs::State cur_state2;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped targetLocal2;

ros::Publisher local_pos_pub2;

ros::Subscriber state_sub2;
ros::Subscriber set_position2;
ros::Subscriber cur_local_sub2;

ros::Subscriber target_position2;
ros::Subscriber target_yaw2;
ros::Subscriber rel_Yaw_sub2;
ros::Subscriber Lookat2;
ros::Subscriber Lookat_pctrl2;
ros::Subscriber bf_position2;
ros::Subscriber bf_pos_pctrl2;
ros::Subscriber bf_yaw_pctrl2;

ros::ServiceClient arming_client2;
ros::ServiceClient set_mode_client2;


void setYaw2(double);
void setPosition2(double,double,double);
double yawfromQuaternion2(double,double,double,double);
void bodyframe2(double,double);


#endif


