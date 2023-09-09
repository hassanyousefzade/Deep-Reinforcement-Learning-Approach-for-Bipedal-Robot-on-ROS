#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include<sensor_msgs/JointState.h>
#include <vector>
#define deg2rad(alpha) (alpha*(3.14/180))

std_msgs::Float64 LAnkleRollPlot_msg;
std_msgs::Float64 LHipRollPlot_msg;
std_msgs::Float64 LHipPitchPlot_msg;
std_msgs::Float64 LKneePitchPlot_msg;
std_msgs::Float64 LAnklePitchPlot_msg;

std_msgs::Float64 RAnkleRollPlot_msg;
std_msgs::Float64 RHipRollPlot_msg;
std_msgs::Float64 RHipPitchPlot_msg;
std_msgs::Float64 RKneePitchPlot_msg;
std_msgs::Float64 RAnklePitchPlot_msg;

void LAnkleRoll_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  LAnkleRollPlot_msg.data=msg->position[1];
  LHipRollPlot_msg.data=msg->position[3];
  LHipPitchPlot_msg.data=msg->position[2];
  LKneePitchPlot_msg.data=msg->position[4];
  LAnklePitchPlot_msg.data=msg->position[0];
  RAnkleRollPlot_msg.data=msg->position[6];
  RHipRollPlot_msg.data=msg->position[8];
  RHipPitchPlot_msg.data=msg->position[7];
  RKneePitchPlot_msg.data=msg->position[9];
  RAnklePitchPlot_msg.data=msg->position[5];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amirmmv_jointdatalogger_node");
  ros::NodeHandle nh;
  ros::Publisher LAnkleRollPlot_pub = nh.advertise<std_msgs::Float64>("l_ankle_roll_position_plot", 100);
  ros::Publisher LHipRollPlot_pub = nh.advertise<std_msgs::Float64>("l_hip_roll_position_plot", 100);
  ros::Publisher LHipPitchPlot_pub = nh.advertise<std_msgs::Float64>("l_hip_pitch_position_plot", 100);
  ros::Publisher LKneePitchPlot_pub = nh.advertise<std_msgs::Float64>("l_knee_pitch_position_plot", 100);
  ros::Publisher LAnklePitchPlot_pub = nh.advertise<std_msgs::Float64>("l_ankle_pitch_position_plot", 100);
  ros::Publisher RAnkleRollPlot_pub = nh.advertise<std_msgs::Float64>("r_ankle_roll_position_plot", 100);
  ros::Publisher RHipRollPlot_pub = nh.advertise<std_msgs::Float64>("r_hip_roll_position_plot", 100);
  ros::Publisher RHipPitchPlot_pub = nh.advertise<std_msgs::Float64>("r_hip_pitch_position_plot", 100);
  ros::Publisher RKneePitchPlot_pub = nh.advertise<std_msgs::Float64>("r_knee_pitch_position_plot", 100);
  ros::Publisher RAnklePitchPlot_pub = nh.advertise<std_msgs::Float64>("r_ankle_pitch_position_plot", 100);

  ros::Subscriber LAnkleRoll_sub=nh.subscribe("/amirmmv_bipedrobot_ns/joint_states", 100, LAnkleRoll_Callback);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    LAnkleRollPlot_pub.publish(LAnkleRollPlot_msg);
    LHipRollPlot_pub.publish(LHipRollPlot_msg);
    LHipPitchPlot_pub.publish(LHipPitchPlot_msg);
    LKneePitchPlot_pub.publish(LKneePitchPlot_msg);
    LAnklePitchPlot_pub.publish(LAnklePitchPlot_msg);
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
