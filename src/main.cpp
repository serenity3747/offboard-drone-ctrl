#include <ros/ros.h>
#include <offb/offb.h>






int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "drone");
    ros::NodeHandle nh("~");
    

    ros::Rate rate(20);
    vehicle_info elysium_info;
    nh.getParam("num_drone", elysium_info.vehicle_name_);
    offb elysium(elysium_info);
    
    // double num_drone;
	// nh.getParam("num_drone", num_drone);

   // std::unique_ptr<offb> elysium(new offb({num_drone,"UAV"}));
    
        // wait for FCU connection
    while(ros::ok() && !elysium.cur_state.connected){
        ros::spinOnce();
        rate.sleep();
    }



    for(int i = 50; ros::ok() && i > 0; --i){
        elysium.local_pos_pub.publish(elysium.cur_local);
        ros::spinOnce();
        rate.sleep();
    }
        //send a few setpoints before starting
    
    
    elysium.arm_cmd.request.value = true;
    elysium.offb_set_mode.request.custom_mode = "OFFBOARD";


	while (ros::ok())
	{
		elysium.running();

		ros::spinOnce();
		rate.sleep();
	}

	elysium.release();

    return 0;
}