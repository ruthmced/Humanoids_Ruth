#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float32MultiArray msgArmLeftGoalPose;
std_msgs::Float32MultiArray msgArmRightGoalPose;

float acc_x;

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    std::cout << "x: " << (float)msg->linear_acceleration.x << std::endl;
    acc_x = (float)msg->linear_acceleration.x;
}

int main(int argc, char** argv){
    std::cout << "\n---- ARMS ARE AWAKE ------\n" << std::endl;
    ros::init(argc, argv, "arms_stabilizer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("imu", 1, ImuCallback);
    
    ros::Publisher pubArmLeftGoalPose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_left_goal_pose", 1); 
    ros::Publisher pubArmRightGoalPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/arm_right_goal_pose", 1);
    
    while(ros::ok()){
        if(acc_x > 10 || acc_x < -10){
            msgArmLeftGoalPose.data = {-M_PI/2,0,0};
            msgArmRightGoalPose.data = {-M_PI/2,0,0};
            pubArmRightGoalPose.publish(msgArmRightGoalPose);
            pubArmLeftGoalPose.publish(msgArmLeftGoalPose);
        }
        ros::spinOnce();
    }
}