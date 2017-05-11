#ifndef LEADER_FOLLOWER_H_
#define LEADER_FOLLOWER_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerial_robot_base/FlightNav.h>
#include <eigen3/Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <unistd.h>

class LeaderFollower
{
public:
  LeaderFollower(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~LeaderFollower(){}

private:
  geometry_msgs::Pose m_snake_pose;
  bool m_task_start_flag;
  /* Subscriber */
  ros::NodeHandle m_nh, m_nhp;
  ros::Subscriber m_sub_snake_task_start_flag;
  ros::Subscriber m_sub_snake_pose;

  /* Publisher */
  ros::Publisher m_pub_snake_start_flag;
  ros::Publisher m_pub_snake_takeoff_flag;
  ros::Publisher m_pub_snake_land_flag;
  ros::Publisher m_pub_snake_joint_states;
  ros::Publisher m_pub_snake_flight_nav;

  /* Topic name */
  std::string m_sub_snake_pose_topic_name;
  std::string m_pub_snake_start_flag_topic_name;
  std::string m_pub_snake_takeoff_flag_topic_name;
  std::string m_pub_snake_land_flag_topic_name;
  std::string m_pub_snake_joint_states_topic_name;
  std::string m_pub_snake_flight_nav_topic_name;

  void snakeInitPose();
  void taskStartCallback(std_msgs::Empty msg);
  void snakePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg);
};
#endif
