#include <ros/ros.h>
#include <offb/offb.h>

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "drone");
    ros::NodeHandle nh("~");
    
    double num_drone;
	nh.getParam("num_drone", num_drone);
    offb::VehicleInit();


    return 0;
}