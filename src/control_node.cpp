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
#define xmargin 1.7
#define ymargin 1.5
#define positionerror 0.05
#define position_diff_x 0
#define position_diff_y 2

geometry_msgs::Point leftlower,rightupper;
geometry_msgs::PoseStamped cur_local0,cur_local1;
geometry_msgs::PoseStamped uav0_target;
geometry_msgs::PoseStamped uav1_target;

// void ctrl_func(offb::cmdsrv::Request &req,offb::cmdsrv::Response &res){
//     leftlower.x=req->x1; //a1
//     leftlower.y=req->y1; //a2
//     rightupper.x=req->x2; //b1
//     rightupper.y=req->y2; //b2


// }


class pubclass{
    public:
        pubclass(){
            targetpub0 = nh_.advertise<geometry_msgs::PoseStamped>
            ("uav0/target_position",1);
        
            targetpub1 = nh_.advertise<geometry_msgs::PoseStamped>
            ("uav1/target_position",1);
            
            while(cur_local0.pose.position.x-uav0_target.pose.position.x<positionerror&&cur_local0.pose.position.y-uav0_target.pose.position.y<positionerror&&cur_local1.pose.position.x-uav1_target.pose.position.x<positionerror&&cur_local1.pose.position.y-uav1_target.pose.position.y<positionerror){
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
    }
    
        }
        void uav0(float x, float y){
            uav0_target.pose.position.x = x;
            uav0_target.pose.position.y = y;
            uav0_target.pose.position.z = height;
        }

        void uav1(float x, float y){
            uav1_target.pose.position.x = x;
            uav1_target.pose.position.y = y-position_diff_y;
            uav1_target.pose.position.z = height;
        }

    
        void runOp(){
            if(cur_local0.pose.position.x-leftlower.y<rightupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-leftlower.y<rightupper.y-cur_local1.pose.position.y){
                uav0(cur_local0.pose.position.x,rightupper.y-ymargin);
                uav1(cur_local1.pose.position.y,rightupper.y-ymargin);
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
                
                if(cur_local0.pose.position.x>leftlower.x&&rightupper.x<cur_local1.pose.position.x){
                    uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
                    uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

                }
            }
            else if(cur_local0.pose.position.y-leftlower.y>rightupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-leftlower.y>rightupper.y-cur_local1.pose.position.y){
                uav0(cur_local0.pose.position.x,leftlower.y+ymargin);
                uav1(cur_local1.pose.position.x,leftlower.y+ymargin);
                targetpub0.publish(uav0_target);
                targetpub1.publish(uav1_target);
                
                if(cur_local0.pose.position.x>leftlower.x&&rightupper.x<cur_local1.pose.position.x){
                    uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
                    uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

                }
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher targetpub0;
        ros::Publisher targetpub1;
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

    pubclass pubclass;

    nh_private.getParam("leftlower_x",leftlower.x);
    nh_private.getParam("leftlower_y",leftlower.y);
    nh_private.getParam("rightupper_x",rightupper.x);
    nh_private.getParam("rightupper_y",rightupper.y);



    // ros::Publisher targetpub0 = nh.advertise<geometry_msgs::PoseStamped>
    //     ("uav0/target_position",1);
    
    // ros::Publisher targetpub1 = nh.advertise<geometry_msgs::PoseStamped>
    //     ("uav1/target_position",1);
    // ros::Subscriber cmd = nh.subscribe
    ros::Subscriber cur_position0 = nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav0/mavros/local_position/pose", 10, cur_positionCB0);

    ros::Subscriber cur_position1 = nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav1/mavros/local_position/pose", 10, cur_positionCB1);    

    // ros::ServiceServer cmd = nh.advertiseService<offb::cmdsrv>
    //     ("cmdsrv",ctrl_func); 

    //시작점으로가기
    pubclass.uav0((leftlower.x+rightupper.x)/2-xmargin,leftlower.y+ymargin);
    pubclass.uav1((leftlower.x+rightupper.x)/2+xmargin,leftlower.y+ymargin);


    // while(cur_local0.pose.position.x-uav0_target.pose.position.x<positionerror&&cur_local0.pose.position.y-uav0_target.pose.position.y<positionerror&&cur_local1.pose.position.x-uav1_target.pose.position.x<positionerror&&cur_local1.pose.position.y-uav1_target.pose.position.y<positionerror){
    //     pubclass.targetpub0.publish(uav0_target);
    //     pubclass.targetpub1.publish(uav1_target);
    // }
    
    //사이클 시작
    ros::Rate rate(2.0);

    while(ros::ok()){
        pubclass.runOp();
        ros::spinOnce();
        rate.sleep();
        if(cur_local0.pose.position.x<leftlower.x&&cur_local1.pose.position.x>rightupper.x){
            pubclass.runOp();
            break;
        }

    }


    

}









// void runOp(){
//     if(cur_local0.pose.position.x-leftlower.y<rightupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-leftlower.y<rightupper.y-cur_local1.pose.position.y){
//         uav0(cur_local0.pose.position.x,rightupper.y-ymargin);
//         uav1(cur_local1.pose.position.y,rightupper.y-ymargin);
//         targetpub0.publish(uav0_target);
//         targetpub1.publish(uav1_target);
        
//         if(cur_local0.pose.position.x>leftlower.x&&rightupper.x<cur_local1.pose.position.x){
//             uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
//             uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

//         }
//     }
//     else if(cur_local0.pose.position.y-leftlower.y>rightupper.y-cur_local0.pose.position.y&&cur_local1.pose.position.y-leftlower.y>rightupper.y-cur_local1.pose.position.y){
//         uav0(cur_local0.pose.position.x,leftlower.y+ymargin);
//         uav1(cur_local1.pose.position.x,leftlower.y+ymargin);
//         targetpub0.publish(uav0_target);
//         targetpub1.publish(uav1_target);
        
//         if(cur_local0.pose.position.x>leftlower.x&&rightupper.x<cur_local1.pose.position.x){
//             uav0(cur_local0.pose.position.x-xmargin*2,cur_local0.pose.position.y);
//             uav1(cur_local1.pose.position.x+xmargin*2,cur_local1.pose.position.y);

//         }
//     }
// }