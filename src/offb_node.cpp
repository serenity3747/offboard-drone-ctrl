#include <ros/ros.h>
#include <offb/offb.h>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>




offb::offb(const VehicleInfo &vehicle_info)
    :drone_info(vehicle_info),
     nh_(ros::NodeHandle(drone_info.vehicle_name_))


{
    VehicleInit();
}

offb::~offb()
{
    release();
}


// state indicate
void offb::state_cb(const mavros_msgs::State::ConstPtr& msg){
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

bool offb::arming(const bool armed){
    
    mavros_msgs::CommandBool msg;
	msg.request.value = armed;

	if (arming_client.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(drone_info.vehicle_name_<< " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}

bool offb::setMode(const std::string &current_mode)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = current_mode;
	if (((current_mode == "offboard") || (current_mode == "OFFBOARD")))
	{
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else
	{
		if (set_mode_client.call(mode) && mode.response.mode_sent)
			;
		else
			ROS_ERROR_STREAM(drone_info.vehicle_name_ << " failed to call set_mode service. " << mode.response.mode_sent);
		return mode.response.mode_sent;
	}
}




// local position (PoseStamped) subscribe. 
void offb::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	cur_local = *msg;
    cur_yaw=yawfromQuaternion(cur_local.pose.orientation.x,cur_local.pose.orientation.y,cur_local.pose.orientation.z,cur_local.pose.orientation.w);
}


// move to x,y,z
void offb::targetPosition_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
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
void offb::targetYaw_cb(const std_msgs::Float64::ConstPtr& msg){
    // targetLocal.pose.position.x=cur_local.pose.position.x;
    // targetLocal.pose.position.y=cur_local.pose.position.y;
    // targetLocal.pose.position.z=cur_local.pose.position.z;
    setPosition(cur_local.pose.position.x,cur_local.pose.position.y,cur_local.pose.position.z);
    double deg=msg->data;
    double rad =(deg)*M_PI/180;

    setYaw(rad);
}

// yawing relative radian, subscribe a degree value(std_msgs::Float64)
void offb::targetYaw_rel_cb(const std_msgs::Float64::ConstPtr& msg){
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
    setYaw(rad + cur_yaw);
}
// look at the (x,y) point. 
void offb::targetYaw_Lookat_cb(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    float lookat_yaw=atan2((lookat_point.y-cur_local.pose.position.y),(lookat_point.x-cur_local.pose.position.x));

    setYaw(lookat_yaw);
    // make setYaw function

}

//p 제어를 위한 코드
void offb::targetYaw_Lookat_pctrl_cb(const geometry_msgs::Point::ConstPtr& msg){
    geometry_msgs::Point lookat_point;
    lookat_point.x=msg->x;
    lookat_point.y=msg->y;

    double lookat_yaw=atan2((lookat_point.y-cur_local.pose.position.y),(lookat_point.x-cur_local.pose.position.x));


    double error_yaw=lookat_yaw-cur_yaw;
    
    setYaw(cur_yaw+Kp*error_yaw);
}

//bodyframe 기준으로 위치 제어
void offb::bodyframe_Position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    bodyframe((*msg).pose.position.x,(*msg).pose.position.y);
    targetLocal.pose.position.z=cur_local.pose.position.z;

    targetLocal.pose.orientation.x=cur_local.pose.orientation.x;
    targetLocal.pose.orientation.y=cur_local.pose.orientation.y;
    targetLocal.pose.orientation.z=cur_local.pose.orientation.z;
    targetLocal.pose.orientation.w=cur_local.pose.orientation.w;

}


void offb::bf_pos_pctrl_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    bodyframe(-((*msg).pose.position.x)*Kp, -((*msg).pose.position.y)*Kp);
    targetLocal.pose.position.z=cur_local.pose.position.z;

    targetLocal.pose.orientation.x=cur_local.pose.orientation.x;
    targetLocal.pose.orientation.y=cur_local.pose.orientation.y;
    targetLocal.pose.orientation.z=cur_local.pose.orientation.z;
    targetLocal.pose.orientation.w=cur_local.pose.orientation.w;

}

void offb::bf_yaw_pctrl_cb(const std_msgs::Float64::ConstPtr& msg){
    setPosition(cur_local.pose.position.x,cur_local.pose.position.y,cur_local.pose.position.z);

    // tf::Quaternion q(
    //     cur_local.pose.orientation.x,
    //     cur_local.pose.orientation.y,
    //     cur_local.pose.orientation.z,
    //     cur_local.pose.orientation.w
    // );
    // tf::Matrix3x3 m(q);
    // double cur_roll,cur_pitch,cur_yaw;
    // m.getRPY(cur_roll,cur_pitch,cur_yaw);
    setYaw((msg->data)*Kp + cur_yaw);
}




void offb::VehicleInit(){
    state_sub = nh_.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &offb::state_cb,this);
    
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    cur_local_sub = nh_.subscribe<geometry_msgs::PoseStamped> 
        ("mavros/local_position/pose", 10, &offb::localPositionCB,this);
    target_position = nh_.subscribe<geometry_msgs::PoseStamped>
        ("target_position",10,&offb::targetPosition_cb,this);
    target_yaw = nh_.subscribe<std_msgs::Float64>
        ("target_yaw",10,&offb::targetYaw_cb,this);
    rel_Yaw_sub = nh_.subscribe<std_msgs::Float64>
        ("rel_yaw",10,&offb::targetYaw_rel_cb,this);
    Lookat = nh_.subscribe<geometry_msgs::Point>
        ("look_at",10,&offb::targetYaw_Lookat_cb,this);
    Lookat_pctrl = nh_.subscribe<geometry_msgs::Point>
        ("look_at_pctrl",10,&offb::targetYaw_Lookat_pctrl_cb,this);    
    bf_position = nh_.subscribe<geometry_msgs::PoseStamped>
        ("bf_position",10,&offb::bodyframe_Position_cb,this);
    bf_pos_pctrl = nh_.subscribe<geometry_msgs::PoseStamped>
        ("bf_pos_pctrl",10,&offb::bf_pos_pctrl_cb,this);    
    bf_yaw_pctrl = nh_.subscribe<std_msgs::Float64>
        ("bf_yaw_pctrl",10,&offb::bf_yaw_pctrl_cb,this);

    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>
        ("mavros/cmd/set_home");
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/land");
}


void offb::release(){
    state_sub.shutdown();
    
    local_pos_pub.shutdown();

    cur_local_sub.shutdown();
    target_position.shutdown();
    target_yaw.shutdown();
    rel_Yaw_sub.shutdown();
    Lookat.shutdown();
    Lookat_pctrl.shutdown();
    bf_position.shutdown();
    bf_pos_pctrl.shutdown();
    bf_yaw_pctrl.shutdown();

    arming_client.shutdown();
    set_mode_client.shutdown();
    set_home_client_ .shutdown();
	takeoff_client_.shutdown();
	land_client_.shutdown();
}



//position 설정
void offb::setPosition(double xx,double yy,double zz){
    targetLocal.pose.position.x=xx;
    targetLocal.pose.position.y=yy;
    targetLocal.pose.position.z=zz;
}
//yaw -> quaternion 설정
void offb::setYaw(double rad){
    geometry_msgs::Quaternion Q=tf::createQuaternionMsgFromYaw(rad);

    targetLocal.pose.orientation.x=Q.x;
    targetLocal.pose.orientation.y=Q.y;
    targetLocal.pose.orientation.z=Q.z;
    targetLocal.pose.orientation.w=Q.w;
}
// quaternion에서 yaw를 추출함
double offb::yawfromQuaternion(double x, double y ,double z ,double w){
        tf::Quaternion q(
        x,y,z,w
        );
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    return yaw;
}

//bodyframe 기준 앞으로 x 옆으로 y 움직임
void offb::bodyframe(double x, double y){
    
    double xx = cur_local.pose.position.x + x * cos(cur_yaw) - y * sin(cur_yaw);
    double yy = cur_local.pose.position.y + x * sin(cur_yaw) + y * cos(cur_yaw);
    setPosition(xx,yy,cur_local.pose.position.z);


}


void offb::running(){
    


    ros::Time last_request = ros::Time::now();



        if( cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
                offb::setMode(cur_state.mode);
                last_request = ros::Time::now();
        } 
        else {
            if( !cur_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                offb::arming(cur_state.armed);
                last_request = ros::Time::now();
            }
        }
        
        local_pos_pub.publish(targetLocal);

}