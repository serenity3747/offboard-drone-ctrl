#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>

// #include "offb/cmdsrv.h"

#define height 4
#define xmargin 1.2/4*height
#define ymargin 1.6/4*height
#define positionerror 0.05
#define position_diff_x 0
#define position_diff_y -2


geometry_msgs::PoseStamped cur_local0,cur_local1;
geometry_msgs::PoseStamped uav0_target;
geometry_msgs::PoseStamped uav1_target;
geometry_msgs::TwistStamped uav0_veltarget;
geometry_msgs::TwistStamped uav1_veltarget;

// void ctrl_func(offb::cmdsrv::Request &req,offb::cmdsrv::Response &res){
//     rightlower.x=req->x1; //a1
//     rightlower.y=req->y1; //a2
//     leftupper.x=req->x2; //b1
//     leftupper.y=req->y2; //b2


// }


class pubclass{
    public:
        // geometry_msgs::Point rightlower,leftupper;
        // geometry_msgs::PoseStamped cur_local0,cur_local1;
        // geometry_msgs::PoseStamped uav0_target;
        // geometry_msgs::PoseStamped uav1_target;

        geometry_msgs::Point rightlower,leftupper;    

        pubclass(){
            targetpub0 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav0/target_position",10);
        
            targetpub1 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav1/target_position",10);

            targetpub0 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav0/target_vel",10);
        
            targetpub1 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav1/target_vel",10);



                nh_.getParam("/control_node/rightlower_x",rightlower.x);
                nh_.getParam("/control_node/rightlower_y",rightlower.y);
                nh_.getParam("/control_node/leftupper_x",leftupper.x);
                nh_.getParam("/control_node/leftupper_y",leftupper.y);
    
            // while(cur_local0.pose.position.x-uav0_target.pose.position.x<positionerror&&cur_local0.pose.position.y-uav0_target.pose.position.y<positionerror
            //     &&cur_local1.pose.position.x-uav1_target.pose.position.x<positionerror &&cur_local1.pose.position.y-uav1_target.pose.position.y<positionerror){
            //     targetpub0.publish(uav0_target);
            //     targetpub1.publish(uav1_target);
            //     }    

        }

        void uav0(float x, float y){
            uav0_target.pose.position.x = x;
            uav0_target.pose.position.y = y;
            uav0_target.pose.position.z = height;
            uav0_target.pose.orientation.w=1;
            uav0_target.pose.orientation.x,uav0_target.pose.orientation.y,uav0_target.pose.orientation.z=0;
        }
      

        void uav1(float x, float y){
            uav1_target.pose.position.x = x-position_diff_x;
            uav1_target.pose.position.y = y-position_diff_y;
            uav1_target.pose.position.z = height;
            uav1_target.pose.orientation.w=1;
            uav1_target.pose.orientation.x,uav1_target.pose.orientation.y,uav1_target.pose.orientation.z=0;
        }


        void ready2start(){
                uav0(rightlower.x-xmargin,(leftupper.y+rightlower.y)/2+ymargin);
                uav1(rightlower.x+xmargin,(leftupper.y+rightlower.y)/2+ymargin);
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
        }
        void runOp(){
            // if(cur_local0.pose.position.x<(leftupper.x+rightlower.x)/2&&cur_local1.pose.position.x<(leftupper.x+rightlower.x)/2){
            if(cur_local0.pose.position.x<leftupper.x&&cur_local1.pose.position.x<leftupper.x){
                uav0(cur_local0.pose.position.x,leftupper.y-ymargin);
                uav0(cur_local1.pose.position.x,leftupper.y-ymargin);
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
                
                if(cur_local0.pose.position.y<leftupper.y&&rightlower.y>cur_local1.pose.position.y){
                    uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
                    uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);
                    targetpub0.publish(uav0_target);
                    targetpub1.publish(uav1_target);
                }
            }
            else if(cur_local0.pose.position.x>(leftupper.x+rightlower.x)/2&&cur_local1.pose.position.x>(leftupper.x+rightlower.x)/2){
                uav0(cur_local0.pose.position.x,rightlower.y+ymargin);
                uav1(cur_local1.pose.position.x,rightlower.y+ymargin);
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
                
                if(cur_local0.pose.position.x>rightlower.x&&leftupper.x<cur_local1.pose.position.x){
                    uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
                    uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);
                    targetpub0.publish(uav0_target);
                    targetpub1.publish(uav1_target);                    
                }
            }
        }

        void test(){
            uav0(rightlower.x,rightlower.y);
            uav1(leftupper.x,leftupper.y);
            // uav0(0,0);
            // uav1(2,-2);
            targetpub0.publish(uav0_target);
            targetpub1.publish(uav1_target);
            // velPub0.publish(uav0_veltarget);
            // velPub1.publish(uav1_veltarget);
        }
            // void test(){
            //     geometry_msgs::PoseStamped test;
            //         uav0_target.pose.position.x=0;
            //         uav0_target.pose.position.y=0;
            //         uav0_target.pose.position.z=4;

            //         uav1_target.pose.position.x=0;
            //         uav1_target.pose.position.y=0;
            //         uav1_target.pose.position.z=4;
            //     targetpub0.publish(uav0_target);
            //     targetpub1.publish(uav1_target);
            // }
       

    private:
        ros::NodeHandle nh_;
        ros::Publisher targetpub0;
        ros::Publisher targetpub1;
        // ros::Publisher velPub0;
        // ros::Publisher velPub1;

};







void cur_positionCB0(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cur_local0=*msg;
}

void cur_positionCB1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cur_local1=*msg;
}     


// void uav0(float x, float y){
//     uav0_target.pose.position.x = x;
//     uav0_target.pose.position.y = y;
//     uav0_target.pose.position.z = height;
// }

// void uav1(float x, float y){
//     uav1_target.pose.position.x = x;
//     uav1_target.pose.position.y = y-2;
//     uav1_target.pose.position.z = height;
// }

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");




    ros::Subscriber cur_position0= nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav0/mavros/local_position/pose", 10, cur_positionCB0);

    ros::Subscriber cur_position1= nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav1/mavros/local_position/pose", 10, cur_positionCB1);    
    
    
    pubclass pub;


    // ros::Publisher targetpub0 = nh.advertise<geometry_msgs::PoseStamped>
    //     ("uav0/target_position",1);
    
    // ros::Publisher targetpub1 = nh.advertise<geometry_msgs::PoseStamped>
    //     ("uav1/target_position",1);
    // ros::Subscriber cmd = nh.subscribe


    // ros::ServiceServer cmd = nh.advertiseService<offb::cmdsrv>
    //     ("cmdsrv",ctrl_func); 

    //시작점으로가기
    // pubclass.uav0((rightlower.x+leftupper.x)/2-xmargin,rightlower.y+ymargin);
    // pubclass.uav1((rightlower.x+leftupper.x)/2+xmargin,rightlower.y+ymargin);
    // pubclass.ready2start();

    // while(cur_local0.pose.position.x-uav0_target.pose.position.x<positionerror&&cur_local0.pose.position.y-uav0_target.pose.position.y<positionerror&&cur_local1.pose.position.x-uav1_target.pose.position.x<positionerror&&cur_local1.pose.position.y-uav1_target.pose.position.y<positionerror){
    //     pubclass.targetpub0.publish(uav0_target);
    //     pubclass.targetpub1.publish(uav1_target);
    // }
    
    //사이클 시작
    ros::Rate rate(20.0);

    while(ros::ok()){
        // pub.runOp();


        pub.test();

        ros::spinOnce();
        rate.sleep();
        // if(cur_local0.pose.position.x<rightlower.x&&cur_local1.pose.position.x>leftupper.x){
        //     pubclass.runOp();
        //     break;
        // }

    }


    

}









// void runOp(){
//     if(cur_local0.pose.position.x-rightlower.y<leftupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-rightlower.y<leftupper.y-cur_local1.pose.position.y){
//         uav0(cur_local0.pose.position.x,leftupper.y-ymargin);
//         uav1(cur_local1.pose.position.y,leftupper.y-ymargin);
//         targetpub0.publish(uav0_target);
//         targetpub1.publish(uav1_target);
        
//         if(cur_local0.pose.position.x>rightlower.x&&leftupper.x<cur_local1.pose.position.x){
//             uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
//             uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

//         }
//     }
//     else if(cur_local0.pose.position.y-rightlower.y>leftupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-rightlower.y>leftupper.y-cur_local1.pose.position.y){
//         uav0(cur_local0.pose.position.x,rightlower.y+ymargin);
//         uav1(cur_local1.pose.position.x,rightlower.y+ymargin);
//         targetpub0.publish(uav0_target);
//         targetpub1.publish(uav1_target);
        
//         if(cur_local0.pose.position.x>rightlower.x&&leftupper.x<cur_local1.pose.position.x){
//             uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
//             uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

//         }
//     }
// }