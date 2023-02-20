#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>

#include "offb/cmdsrv.h"

geometry_msgs::Point leftlower,rightupper;
geometry_msgs::PoseStamped uav0,uav1;
geometry_msgs::PointStamped cur_local;

void ctrl_func(offb::cmdsrv::Request &req,offb::cmdsrv::Response &res){
    leftlower.x=req->x1; //a1
    leftlower.y=req->y1; //a2
    rightupper.x=req->x2; //b1
    rightupper.y=req->y2; //b2


}

void cur_positionCB(geometry_msgs::PoseStamped::Constptr &msg){
    cur_local=*msg;
}



int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Publisher targetpub0 = nh.advertise<geometry_msgs::PoseStamped>
        ("uav0/target_position",1);
    
    ros::Publisher targetpub1 = nh.advertise<geometry_msgs::PoseStamped>
        ("uav1/target_position",1);
    // ros::Subscriber cmd = nh.subscribe
    ros::Subscriber cur_position = nh.subscribe<geometry_msgs::PoseStamped> 
        ("mavros/local_position/pose", 10, cur_positionCB);

    ros::ServiceServer cmd = nh.advertiseService<offb::cmdsrv>
        ("cmdsrv",ctrl_func); 

    
    
}