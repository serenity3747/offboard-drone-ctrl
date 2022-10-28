#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>




geometry_msgs::PoseStamped cur_local_;
double roll_ ,pitch_ ,yaw_ ;

void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	cur_local_ = *msg;
}


int main(int argc,char **argv){
    ros::init(argc,argv,"pose_node");
    ros::NodeHandle nh;

    ros::Publisher send_pose=nh.advertise<geometry_msgs::PoseStamped>
        ("set_position",10);
    ros::Subscriber cur_local = nh.subscribe<geometry_msgs::PoseStamped> 
        ("mavros/local_position/pose", 10, localPositionCB);

    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pos;
    






while(ros::ok()){

    double x,y,z,yawing;
    ROS_INFO("INPUT POSITION X Y Z W");
    scanf("%lf %lf %lf %lf",&x,&y,&z,&yawing);
	tf::Quaternion cur_q(
		cur_local_.pose.orientation.x,
		cur_local_.pose.orientation.y,
		cur_local_.pose.orientation.z,
		cur_local_.pose.orientation.w
	);
    tf::Matrix3x3 cur_m(cur_q);
	cur_m.getRPY( roll_, pitch_, yaw_ );


    
    // tf::Matrix3x3 tar_m;
    // tar_m.setEulerYPR(yaw_ + radYawing,pitch_,roll_);
    // tf::Quaternion tar_q;
    // tar_m.getRotation(tar_q);
	   
    // pos.pose.orientation.w = tar_q.getW();
	// pos.pose.orientation.x = tar_q.getX();
	// pos.pose.orientation.y = tar_q.getY();
	// pos.pose.orientation.z = tar_q.getZ();
    double radYawing= yaw_+(yawing)*M_PI/180;

    pos.pose.position.x=x;
    pos.pose.position.y=y;
    pos.pose.position.z=z;
    pos.pose.orientation=tf::createQuaternionMsgFromYaw(radYawing);
    send_pose.publish(pos);

    if(z<0){
        break;
    }

    }
    return 0;
}