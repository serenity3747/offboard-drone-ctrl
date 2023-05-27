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


#define position_diff_x 0
#define position_diff_y -2



geometry_msgs::PoseStamped cur_local0,cur_local1;
geometry_msgs::PoseStamped uav0_target;
geometry_msgs::PoseStamped uav1_target;
geometry_msgs::TwistStamped uav0_veltarget;
geometry_msgs::TwistStamped uav1_veltarget;


std_msgs::Float64 caputre_sign;
float height=5,xmoving, ymoving;



// void ctrl_func(offb::cmdsrv::Request &req,offb::cmdsrv::Response &res){
//     rightlower.x=req->x1; //a1
//     rightlower.y=req->y1; //a2
//     leftupper.x=req->x2; //b1
//     leftupper.y=req->y2; //b2


// }


class pubclass{
    public:
        // geometry_msgs::PoseStamped cur_local0,cur_local1;
        // geometry_msgs::PoseStamped uav0_target;
        // geometry_msgs::PoseStamped uav1_target;
        
        


        geometry_msgs::Point rightlower,leftupper;    

        pubclass(){
            targetpub0 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav0/target_position",10);
        
            targetpub1 = nh_.advertise<geometry_msgs::PoseStamped>
                ("uav1/target_position",10);

            // targetpub0 = nh_.advertise<geometry_msgs::PoseStamped>
            //     ("uav0/target_vel",10);
        
            // targetpub1 = nh_.advertise<geometry_msgs::PoseStamped>
            //     ("uav1/target_vel",10);




                nh_.getParam("/control_node/rightlower/x",rightlower.x);
                nh_.getParam("/control_node/rightlower/y",rightlower.y);
                nh_.getParam("/control_node/leftupper/x",leftupper.x);
                nh_.getParam("/control_node/leftupper/y",leftupper.y);

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


        // void ready2start(){
        //         uav0(rightlower.x-xmargin,(leftupper.y+rightlower.y)/2+ymargin);
        //         uav1(rightlower.x+xmargin,(leftupper.y+rightlower.y)/2+ymargin);
        //         targetpub0.publish(uav0_target);
        //         targetpub1.publish(uav1_target);
        // }

        void goTo0(){
            targetpub0.publish(uav0_target);
        }

        void goTo1(){
            targetpub1.publish(uav1_target);
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
        bool isArriving(){
            if((fabs(uav0_target.pose.position.x - cur_local0.pose.position.x) < 0.5)
            && (fabs(uav0_target.pose.position.y - cur_local0.pose.position.y) < 0.5)
            && (fabs(uav0_target.pose.position.z - cur_local0.pose.position.z) < 0.5)
            && (fabs(uav1_target.pose.position.x - cur_local1.pose.position.x) < 0.5)
            && (fabs(uav1_target.pose.position.y - cur_local1.pose.position.y) < 0.5)
            && (fabs(uav1_target.pose.position.z - cur_local1.pose.position.z) < 0.5)){
                return true;
            }

            else return false;
        }
        bool isArrivingOnly0(){
            if((fabs(uav0_target.pose.position.x - cur_local0.pose.position.x) < 0.5)
            && (fabs(uav0_target.pose.position.y - cur_local0.pose.position.y) < 0.5)
            && (fabs(uav0_target.pose.position.z - cur_local0.pose.position.z) < 0.5)){
                return true;
            }

            else return false;
        }
                        

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




    ros::Subscriber cur_position0= nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav0/mavros/local_position/pose", 10, cur_positionCB0);

    ros::Subscriber cur_position1= nh.subscribe<geometry_msgs::PoseStamped> 
        ("uav1/mavros/local_position/pose", 10, cur_positionCB1);    
    
    ros::Publisher capture_sign_pub = nh.advertise<std_msgs::Float64>
        ("/capture_now", 1);

    caputre_sign.data = 0.0;    

    //사이클 시작
    ros::Rate rate(20.0);

    pubclass pub;


    float x_march, y_march;
    float x_uav0, x_uav1;
    float y_uav0, y_uav1;
    float x_length, y_length;
    int index,index2,index3;
    int flag,mode;

    while(ros::ok){

        nh.getParam("/control_node/height",height);
        nh.getParam("/control_node/Xmoving",xmoving);
        nh.getParam("/control_node/Ymoving",ymoving);


        // 높이 비례하여 march 거리 연산
        x_march=height*xmoving;
        y_march=height*ymoving;

        // 영역 길이 
        y_length = pub.leftupper.y - pub.rightlower.y;
        x_length = pub.leftupper.x - pub.rightlower.x;

        // loop에 사용될 index 설정
        index = ceil(x_length / x_march);
        index2= ceil((y_length/2) / y_march);
        index3= ceil(y_length/y_march);


        nh.getParam("/control_node/flag",flag);
        nh.getParam("/control_node/mode",mode);
        


        if(!flag){
                pub.uav0(0.0,0.0);
                pub.uav1(0.0,-2.0);
                pub.goTo0(); pub.goTo1();
        }

        if(flag&&mode==0){ //ㄹ형식 탐색
            x_uav0 = x_march + pub.rightlower.x;
            x_uav1 = x_uav0;
            y_uav0 = (pub.rightlower.y+pub.leftupper.y)/2+y_march;//4.2*height + pub.rightlower.y;
            y_uav1 = (pub.rightlower.y+pub.leftupper.y)/2-y_march ;//2*height + pub.rightlower.y;

            for(int j=0; j<index2; j++){
                for(int i=0; i<index; i++){
                    for(int count=0; count<50; ){
                        pub.uav0(x_uav0,y_uav0);
                        pub.uav1(x_uav1,y_uav1);

                        pub.goTo0(); pub.goTo1();


                        if(pub.isArriving()){
                            count++;
                        }

                        if(count > 25){
                            capture_sign_pub.publish(caputre_sign);
                        }

                        ros::spinOnce();
                        rate.sleep();
                    }
                    ROS_INFO("March!");
                    x_uav0 = x_uav0 + x_march;
                    x_uav1 = x_uav1 + x_march;
                    caputre_sign.data = caputre_sign.data + 1.0;
                }
                x_uav0 = x_uav0 - x_march;
                x_uav1 = x_uav1 - x_march;
                x_march=-x_march;
                y_uav0 += y_march;
                y_uav1 -= y_march;
            }

            // nh.setParam("/control_node/start",false);
        }

        else if(flag&&mode==1){// 일자탐색
            x_uav0 = x_march;//x_march + pub.rightlower.x;
            x_uav1 = x_uav0;
            y_uav0 = (pub.rightlower.y+pub.leftupper.y)/2+y_march;//4.2*height + pub.rightlower.y;
            y_uav1 = (pub.rightlower.y+pub.leftupper.y)/2-y_march ;//2*height + pub.rightlower.y;

            for(int i=0; i<index; i++){
                for(int count=0; count<50; ){
                    pub.uav0(x_uav0,y_uav0);
                    pub.uav1(x_uav1,y_uav1);

                    pub.goTo0(); pub.goTo1();


                    if(pub.isArriving()){
                        count++;
                    }

                    if(count > 25){
                        capture_sign_pub.publish(caputre_sign);
                    }

                    ros::spinOnce();
                    rate.sleep();
                }
                ROS_INFO("March!");
                x_uav0 = x_uav0 + x_march;
                x_uav1 = x_uav1 + x_march;
                caputre_sign.data = caputre_sign.data + 1.0;
            }
            // nh.setParam("/control_node/start",false);

        }

        else if(flag&&mode==2){ //같이가는 ㄹ형식 탐색
            x_uav0 = x_march + pub.rightlower.x;
            x_uav1 = x_uav0;
            y_uav0 = pub.rightlower.y+3*y_march;//4.2*height + pub.rightlower.y;
            y_uav1 = pub.rightlower.y+y_march ;//2*height + pub.rightlower.y;
            for(int j=0; j<index2; j++){
                for(int i=0; i<index; i++){
                    for(int count=0; count<50; ){
                        pub.uav0(x_uav0,y_uav0);
                        pub.uav1(x_uav1,y_uav1);

                        pub.goTo0(); pub.goTo1();


                        if(pub.isArriving()){
                            count++;
                        }

                        if(count > 25){
                            capture_sign_pub.publish(caputre_sign);
                        }

                        ros::spinOnce();
                        rate.sleep();
                    }
                    ROS_INFO("March!");
                    x_uav0 = x_uav0 + x_march;
                    x_uav1 = x_uav1 + x_march;
                    caputre_sign.data = caputre_sign.data + 1.0;
                }
                x_uav0 = x_uav0 - x_march;
                x_uav1 = x_uav1 - x_march;
                x_march=-x_march;
                y_uav0 += 3*y_march;
                y_uav1 += 3*y_march;
            }

            // nh.setParam("/control_node/start",false);

        }
    //     else if(flag&&mode==3){//uav0만 멈추지않고 움직이기
    //         x_uav0=pub.leftupper.x;
    //         y_uav0=pub.leftupper.y;

    //         for(int j=0;j<index3;j++){
    //             pub.uav0(x_uav0,y_uav0);


    //             while(!(pub.isArrivingOnly0())){
    //                     capture_sign_pub.publish(caputre_sign);
    //             }
    //             y_uav0+=y_march;
    //             pub.uav0(x_uav0,y_uav0);

    //             if(x_uav0==pub.leftupper.x) x_uav0=pub.rightlower.x;
    //             else if(x_uav0==pub.rightlower.x) x_uav0=pub.leftupper.x;

    //             ros::spinOnce();
    //             rate.sleep();

    //         }
    //    }

        nh.setParam("/control_node/flag",0);
        ros::spinOnce();
        rate.sleep();
    }

}
