#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>



geometry_msgs::PoseStamped cur_local2;
mavros_msgs::SetMode offb_set_mode2;
mavros_msgs::CommandBool arm_cmd2;
mavros_msgs::State cur_state2;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped targetLocal2;
geometry_msgs::TwistStamped targetvel2;

ros::Publisher local_pos_pub2;
ros::Publisher local_vel_pub2;


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
ros::Subscriber target_vel2;

ros::ServiceClient arming_client2;
ros::ServiceClient set_mode_client2;


void setYaw2(double);
void setPosition2(double,double,double);
double yawfromQuaternion2(double,double,double,double);
void bodyframe2(double,double);
void setVel2(double,double,double);


float Kp2=0.5;
double cur_yaw2;

// state indicate
void state_cb2(const mavros_msgs::State::ConstPtr& msg){

	cur_state2= *msg;
}



// local position (PoseStamped) subscribe. 
void localPositionCB2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	cur_local2 = *msg;
    cur_yaw2=yawfromQuaternion2(cur_local2.pose.orientation.x,cur_local2.pose.orientation.y,cur_local2.pose.orientation.z,cur_local2.pose.orientation.w);
}


// move to x,y,z
void targetPosition_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // targetLocal.pose.position.x=(*msg).pose.position.x;
    // targetLocal.pose.position.y=(*msg).pose.position.y;
    // targetLocal.pose.position.z=(*msg).pose.position.z;
    setPosition2((*msg).pose.position.x,(*msg).pose.position.y,(*msg).pose.position.z);

    targetLocal2.pose.orientation.x=cur_local2.pose.orientation.x;
    targetLocal2.pose.orientation.y=cur_local2.pose.orientation.y;
    targetLocal2.pose.orientation.z=cur_local2.pose.orientation.z;
    targetLocal2.pose.orientation.w=cur_local2.pose.orientation.w;

}


// yawing absolute radian, subscribe a degree value(std_msgs::Float64)
void targetYaw_cb2(const std_msgs::Float64::ConstPtr& msg){
    // targetLocal.pose.position.x=cur_local.pose.position.x;
    // targetLocal.pose.position.y=cur_local.pose.position.y;
    // targetLocal.pose.position.z=cur_local.pose.position.z;
    setPosition2(cur_local2.pose.position.x,cur_local2.pose.position.y,cur_local2.pose.position.z);
    double deg=msg->data;
    double rad =(deg)*M_PI/180;

    setYaw2(rad);
}

// yawing relative radian, subscribe a degree value(std_msgs::Float64)
void targetYaw_rel_cb2(const std_msgs::Float64::ConstPtr& msg){
    setPosition2(cur_local2.pose.position.x,cur_local2.pose.position.y,cur_local2.pose.position.z);
    double deg=msg->data;
    
    double rad =(deg)*M_PI/180;

    // tf::Quaternion q(
    //     cur_local.pose.orientation.x,
    //     cur_local.pose.orientation.y,
    //     cur_local.pose.orientation.z,
    //     cur_local.pose.orientation.w
    // );
    // tf::Matrix3x3 m(q);
    // double cur_roll,cur_pitch,cur_yaw;
    // m.getRPY(cur_roll,cur_pitch,cur_yaw);
    setYaw2(rad + cur_yaw2);
}
// look at the (x,y) point. 
void targetYaw_Lookat_cb2(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    float lookat_yaw=atan2((lookat_point.y-cur_local2.pose.position.y),(lookat_point.x-cur_local2.pose.position.x));

    setYaw2(lookat_yaw);
    // make setYaw function

}

//p 제어를 위한 코드
void targetYaw_Lookat_pctrl_cb2(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    double lookat_yaw=atan2((lookat_point.y-cur_local2.pose.position.y),(lookat_point.x-cur_local2.pose.position.x));


    double error_yaw=lookat_yaw-cur_yaw2;
    
    setYaw2(cur_yaw2+Kp2*error_yaw);
}

//bodyframe 기준으로 위치 제어
void bodyframe_Position_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg){

    bodyframe2((*msg).pose.position.x,(*msg).pose.position.y);
    targetLocal2.pose.position.z=cur_local2.pose.position.z;

    targetLocal2.pose.orientation.x=cur_local2.pose.orientation.x;
    targetLocal2.pose.orientation.y=cur_local2.pose.orientation.y;
    targetLocal2.pose.orientation.z=cur_local2.pose.orientation.z;
    targetLocal2.pose.orientation.w=cur_local2.pose.orientation.w;

}


void bf_pos_pctrl_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg){

    bodyframe2(-((*msg).pose.position.x)*Kp2, -((*msg).pose.position.y)*Kp2);
    targetLocal2.pose.position.z=cur_local2.pose.position.z;

    targetLocal2.pose.orientation.x=cur_local2.pose.orientation.x;
    targetLocal2.pose.orientation.y=cur_local2.pose.orientation.y;
    targetLocal2.pose.orientation.z=cur_local2.pose.orientation.z;
    targetLocal2.pose.orientation.w=cur_local2.pose.orientation.w;

}

void bf_yaw_pctrl_cb2(const std_msgs::Float64::ConstPtr& msg){
    setPosition2(cur_local2.pose.position.x,cur_local2.pose.position.y,cur_local2.pose.position.z);

    // tf::Quaternion q(
    //     cur_local.pose.orientation.x,
    //     cur_local.pose.orientation.y,
    //     cur_local.pose.orientation.z,
    //     cur_local.pose.orientation.w
    // );
    // tf::Matrix3x3 m(q);
    // double cur_roll,cur_pitch,cur_yaw;
    // m.getRPY(cur_roll,cur_pitch,cur_yaw);
    setYaw2((msg->data)*Kp2 + cur_yaw2);
}
void vel_cb2(const geometry_msgs::TwistStamped::ConstPtr& msg){
    setVel2(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}


geometry_msgs::PoseStamped pos;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "offb_node2");
    ros::NodeHandle nh2;
    

    state_sub2 = nh2.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb2);
    local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>
        ("/uav1/mavros/setpoint_position/local", 10);

    cur_local_sub2 = nh2.subscribe<geometry_msgs::PoseStamped> 
        ("mavros/local_position/pose", 10, localPositionCB2);
    target_position2 = nh2.subscribe<geometry_msgs::PoseStamped>
        ("target_position",10,targetPosition_cb2);
    target_yaw2 = nh2.subscribe<std_msgs::Float64>
        ("target_yaw",10,targetYaw_cb2);
    rel_Yaw_sub2 = nh2.subscribe<std_msgs::Float64>
        ("rel_yaw",10,targetYaw_rel_cb2);
    Lookat2 = nh2.subscribe<geometry_msgs::Point>
        ("look_at",10,targetYaw_Lookat_cb2);
    Lookat_pctrl2 = nh2.subscribe<geometry_msgs::Point>
        ("look_at_pctrl",10,targetYaw_Lookat_pctrl_cb2);    
    bf_position2 = nh2.subscribe<geometry_msgs::PoseStamped>
        ("bf_position",10,bodyframe_Position_cb2);
    bf_pos_pctrl2 = nh2.subscribe<geometry_msgs::PoseStamped>
        ("bf_pos_pctrl",10,bf_pos_pctrl_cb2);    
    bf_yaw_pctrl2 = nh2.subscribe<std_msgs::Float64>
        ("bf_yaw_pctrl",10,bf_yaw_pctrl_cb2);
    target_vel2 = nh2.subscribe<geometry_msgs::TwistStamped>
        ("target_vel",10,vel_cb2);

    arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>
        ("/uav1/mavros/cmd/arming");
    set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>
        ("/uav1/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    


    // wait for FCU connection
    while(ros::ok() && !cur_state2.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x=0;pos.pose.position.y=2;pos.pose.position.z=2;
    for(int i = 100; ros::ok() && i > 0; --i){
        
        local_pos_pub2.publish(pos);
        ros::spinOnce();
        rate.sleep();
    }
        //send a few setpoints before starting

    
    mavros_msgs::SetMode offb_set_mode2;
    offb_set_mode2.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd2;
    arm_cmd2.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( cur_state2.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client2.call(offb_set_mode2) &&
                offb_set_mode2.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !cur_state2.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client2.call(arm_cmd2) &&
                    arm_cmd2.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        local_pos_pub2.publish(targetLocal2);

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



// //position 설정
// void setPosition(double xx,double yy,double zz){
//     targetLocal.pose.position.x=xx;
//     targetLocal.pose.position.y=yy;
//     targetLocal.pose.position.z=zz;
// }
// //yaw -> quaternion 설정
// void setYaw(double rad){
//     geometry_msgs::Quaternion Q=tf::createQuaternionMsgFromYaw(rad);

//     targetLocal.pose.orientation.x=Q.x;
//     targetLocal.pose.orientation.y=Q.y;
//     targetLocal.pose.orientation.z=Q.z;
//     targetLocal.pose.orientation.w=Q.w;
// }
// // quaternion에서 yaw를 추출함
// double yawfromQuaternion(double x, double y ,double z ,double w){
//         tf::Quaternion q(
//         x,y,z,w
//         );
//     tf::Matrix3x3 m(q);
//     double roll,pitch,yaw;
//     m.getRPY(roll,pitch,yaw);
//     return yaw;
// }

// //bodyframe 기준 앞으로 x 옆으로 y 움직임
// void bodyframe(double x, double y){
    
//     double xx = cur_local.pose.position.x + x * cos(cur_yaw) - y * sin(cur_yaw);
//     double yy = cur_local.pose.position.y + x * sin(cur_yaw) + y * cos(cur_yaw);
//     setPosition(xx,yy,cur_local.pose.position.z);


// }

//position 설정
void setPosition2(double xx,double yy,double zz){
    targetLocal2.pose.position.x=xx;
    targetLocal2.pose.position.y=yy;
    targetLocal2.pose.position.z=zz;
}
//yaw -> quaternion 설정
void setYaw2(double rad){
    geometry_msgs::Quaternion Q=tf::createQuaternionMsgFromYaw(rad);

    targetLocal2.pose.orientation.x=Q.x;
    targetLocal2.pose.orientation.y=Q.y;
    targetLocal2.pose.orientation.z=Q.z;
    targetLocal2.pose.orientation.w=Q.w;
}
// quaternion에서 yaw를 추출함
double yawfromQuaternion2(double x, double y ,double z ,double w){
        tf::Quaternion q(
        x,y,z,w
        );
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    return yaw;
}

//bodyframe 기준 앞으로 x 옆으로 y 움직임
void bodyframe2(double x, double y){
    
    double xx = cur_local2.pose.position.x + x * cos(cur_yaw2) - y * sin(cur_yaw2);
    double yy = cur_local2.pose.position.y + x * sin(cur_yaw2) + y * cos(cur_yaw2);
    setPosition2(xx,yy,cur_local2.pose.position.z);


}
void setVel2(double vx, double vy, double vz){
    targetvel2.twist.linear.x=vx;
    targetvel2.twist.linear.y=vy;
    targetvel2.twist.linear.z=vz;
}
