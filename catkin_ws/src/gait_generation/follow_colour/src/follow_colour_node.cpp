#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Float32MultiArray.h>
#include<sensor_msgs/JointState.h>

#define vertical_threshold     0.15
#define horizontal_threshold   0.20

#define angle_view   0.680678

using namespace std;



ros::Publisher yaw_pub;
ros::Publisher pitch_pub;


float  goal_pan;
float goal_tilt;



void angles_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    cout<<"["<<msg->data[0]<<","<<msg->data[1]<<"]"<<endl;

    std_msgs::Float64 yaw_msg, pitch_msg;

    
    yaw_msg.data   += 0.5 * (msg->data[0]);
    pitch_msg.data += 0.5 * (msg->data[1]);
    
    yaw_pub.publish(yaw_msg);
    pitch_pub.publish(pitch_msg);
}


int main(int argc, char **argv)
{
    cout<<"Starting follow_colour_node by Luis Näva..."<<endl;
    ros::init(argc, argv, "follow_colour_node");
    ros::NodeHandle nh;

    ros::Subscriber angles_sub = nh.subscribe("/vision/get_ball_position/vision_angles", 1000 , angles_callback);
                     pitch_pub = nh.advertise<std_msgs::Float64>("/nimbro/head_pitch_position_controller/command", 1);
                       yaw_pub = nh.advertise<std_msgs::Float64>("/nimbro/neck_yaw_position_controller/command"  , 1);





    ros::spin();
    return 0;
}