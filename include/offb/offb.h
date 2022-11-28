 #ifndef OFFB_H
#define OFFB_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


class offb
{
private:
    ros::NodeHandle nh_;
    
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
    ros::ServiceClient set_home_client_;
	ros::ServiceClient takeoff_client_;
	ros::ServiceClient land_client_;


    void setYaw(double);
    void setPosition(double,double,double);
    double yawfromQuaternion(double,double,double,double);
    void bodyframe(double,double);

    void VehicleInit()

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void arming(const bool &arm_state);

    void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void targetPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void targetYaw_cb(const std_msgs::Float64::ConstPtr& msg);
    void targetYaw_rel_cb(const std_msgs::Float64::ConstPtr& msg);
    void targetYaw_Lookat_cb(const geometry_msgs::Point::ConstPtr& msg);
    void targetYaw_Lookat_pctrl_cb(const geometry_msgs::Point::ConstPtr& msg);
    void bodyframe_Position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void bf_pos_pctrl_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void bf_yaw_pctrl_cb(const std_msgs::Float64::ConstPtr& msg);
    

    geometry_msgs::PoseStamped pos;
public:
    offb();
    offb()





    ~offb();
};










#endif


