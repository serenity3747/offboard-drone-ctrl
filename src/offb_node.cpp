#include <ros/ros.h>
#include <offb/offb.h>
#include <cmath>


float Kp=0.4;

// state indicate
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    	if (cur_state.connected != msg->connected)
	{
		if (msg->connected == true)
			ROS_INFO_STREAM("connected");
		else
			ROS_WARN_STREAM("not connected");
	}
	if (cur_state.armed != msg->armed)
	{
		if (msg->armed == true)
			ROS_INFO_STREAM("armed");
		else
			ROS_INFO_STREAM("disarmed");
	}
	if (cur_state.mode != msg->mode)
	{
		ROS_INFO_STREAM(msg->mode << "mode");
	}
	cur_state= *msg;
}


// local position (PoseStamped) subscribe. 
void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	cur_local = *msg;
}


// move to x,y,z
void targetPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // targetLocal.pose.position.x=(*msg).pose.position.x;
    // targetLocal.pose.position.y=(*msg).pose.position.y;
    // targetLocal.pose.position.z=(*msg).pose.position.z;
    setPosition((*msg).pose.position.x,(*msg).pose.position.y,(*msg).pose.position.z);

    targetLocal.pose.orientation.x=cur_local.pose.orientation.x;
    targetLocal.pose.orientation.y=cur_local.pose.orientation.y;
    targetLocal.pose.orientation.z=cur_local.pose.orientation.z;
    targetLocal.pose.orientation.w=cur_local.pose.orientation.w;

}


// yawing absolute radian, subscribe a degree value(std_msgs::Float64)
void targetYaw_cb(const std_msgs::Float64::ConstPtr& msg){
    // targetLocal.pose.position.x=cur_local.pose.position.x;
    // targetLocal.pose.position.y=cur_local.pose.position.y;
    // targetLocal.pose.position.z=cur_local.pose.position.z;
    setPosition(cur_local.pose.position.x,cur_local.pose.position.y,cur_local.pose.position.z);
    double deg=msg->data;
    double rad =(deg)*M_PI/180;

    setYaw(rad);
}

// yawing relative radian, subscribe a degree value(std_msgs::Float64)
void targetYaw_rel_cb(const std_msgs::Float64::ConstPtr& msg){
    // targetLocal.pose.position.x=cur_local.pose.position.x;
    // targetLocal.pose.position.y=cur_local.pose.position.y;
    // targetLocal.pose.position.z=cur_local.pose.position.z;
    setPosition(cur_local.pose.position.x,cur_local.pose.position.y,cur_local.pose.position.z);
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
    double cur_yaw=yawfromQuaternion(cur_local.pose.orientation.x,cur_local.pose.orientation.y,cur_local.pose.orientation.z,cur_local.pose.orientation.w);
    setYaw(rad + cur_yaw);
}
// look at the (x,y) point. 
void targetYaw_Lookat_cb(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    float lookat_yaw=atan2((lookat_point.y-cur_local.pose.position.y),(lookat_point.x-cur_local.pose.position.x));

    setYaw(lookat_yaw);
    // make setYaw function

}

//p 제어를 위한 코드
void targetYaw_Lookat_pctrl_cb(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    double cur_yaw=yawfromQuaternion(cur_local.pose.orientation.x,cur_local.pose.orientation.y,cur_local.pose.orientation.z,cur_local.pose.orientation.w);
    double lookat_yaw=atan2((lookat_point.y-cur_local.pose.position.y),(lookat_point.x-cur_local.pose.position.x));
    
    double error_yaw;

    error_yaw=lookat_yaw-cur_yaw;
    
    setYaw(cur_yaw+Kp*error_yaw);
}

void setPosition(double xx,double yy,double zz){
    targetLocal.pose.position.x=xx;
    targetLocal.pose.position.y=yy;
    targetLocal.pose.position.z=zz;
}

void setYaw(double rad){
    geometry_msgs::Quaternion Q=tf::createQuaternionMsgFromYaw(rad);

    targetLocal.pose.orientation.x=Q.x;
    targetLocal.pose.orientation.y=Q.y;
    targetLocal.pose.orientation.z=Q.z;
    targetLocal.pose.orientation.w=Q.w;
}

double yawfromQuaternion(double x, double y ,double z ,double w){
        tf::Quaternion q(
        x,y,z,w
        );
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    return yaw;
}

geometry_msgs::PoseStamped pos;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    

    state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    cur_local_sub = nh.subscribe<geometry_msgs::PoseStamped> 
        ("mavros/local_position/pose", 10, localPositionCB);
    target_position = nh.subscribe<geometry_msgs::PoseStamped>
        ("target_position",10,targetPosition_cb);
    target_yaw = nh.subscribe<std_msgs::Float64>
        ("target_yaw",10,targetYaw_cb);
    rel_Yaw_sub = nh.subscribe<std_msgs::Float64>
        ("rel_yaw",10,targetYaw_rel_cb);
    Lookat = nh.subscribe<geometry_msgs::Point>
        ("look_at",10,targetYaw_Lookat_cb);
    Lookat_pctrl = nh.subscribe<geometry_msgs::Point>
        ("look_at_pctrl",10,targetYaw_Lookat_pctrl_cb);    

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    


    // wait for FCU connection
    while(ros::ok() && !cur_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x=0;pos.pose.position.y=0;pos.pose.position.z=2;
    for(int i = 100; ros::ok() && i > 0; --i){
        
        local_pos_pub.publish(pos);
        ros::spinOnce();
        rate.sleep();
    }
        //send a few setpoints before starting

    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !cur_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        local_pos_pub.publish(targetLocal);

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void arming(){
        offb_set_mode.request.custom_mode = "OFFBOARD";   
        arm_cmd.request.value = true;
}



