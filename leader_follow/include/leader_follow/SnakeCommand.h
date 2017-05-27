#ifndef SNAKE_COMMAND_H_
#define SNAKE_COMMAND_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>

/* math */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* local library */
#include <aerial_robot_base/FlightNav.h>
#include <bspline_ros/bsplineGenerate.h>

#include <iostream>
using namespace Eigen;

namespace snake_command{
  class SnakeCommand
  {
  public:
    SnakeCommand(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~SnakeCommand(){}
    ros::NodeHandle m_nh, m_nhp;
    bool m_move_start_flag;
    int m_n_links;
    double m_link_length;
    double m_control_period;
    tf::Vector3* m_links_pos_ptr;
    tf::Vector3* m_links_vel_ptr;
    double* m_joints_ang_ptr;
    double* m_joints_ang_vel_ptr;
    nav_msgs::Odometry m_base_link_odom;
    tf::Vector3 m_base_link_pos;
    tf::Vector3 m_base_link_vel;
    tf::Vector3 m_base_link_ang;

    std::string m_pub_flight_nav_topic_name;
    std::string m_pub_joints_ctrl_topic_name;
    std::string m_sub_move_start_flag_topic_name;
    std::string m_sub_joint_states_topic_name;
    std::string m_sub_base_link_odom_topic_name;

    tf::TransformListener m_tf_listener;

    /* spline */
    bsplineGenerate *m_bspline_traj_ptr;
    double m_traj_start_time;
    double m_traj_current_time;
    double m_traj_bias_start_time;
    double m_spline_segment_time;
    tf::Vector3 m_traj_track_i_term_accumulation;

    /* Publisher */
    ros::Publisher m_pub_flight_nav;
    ros::Publisher m_pub_joints_ctrl;

    /* Subscriber */
    ros::Subscriber m_sub_move_start_flag;
    ros::Subscriber m_sub_joint_states;
    ros::Subscriber m_sub_base_link_odom;
    ros::Timer m_timer;

    void moveStartFlagCallback(const std_msgs::Empty msg);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg);
    void baseLinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void controlCallback(const ros::TimerEvent& e);
    void directTrackGlobalTrajectory();
    void transformTrackGlobalTrajectory();
    inline tf::Vector3 vectorToVector3(std::vector<double> vec);
    void getNextLink(tf::Vector3 &next_link, tf::Vector3 cur_link, double &joint_ang, tf::Vector3 cur_link_real, double cur_time);
    void getNextLink(tf::Vector3 &next_link, tf::Vector3 cur_link, double &joint_ang, double start_time);
    void getNextLinkFromSpline(tf::Vector3 &next_link, tf::Vector3 cur_link, double link_length, double start_time);
    void getPreviousLink(tf::Vector3 &prev_link, tf::Vector3 cur_link, double &joint_ang, double start_time);
    void getPreviousLink(tf::Vector3 &prev_link, tf::Vector3 cur_link, double &joint_ang, tf::Vector3 cur_link_real, double cur_time);
    void getPreviousLinkFromSpline(tf::Vector3 &prev_link, tf::Vector3 cur_link, double link_length, double start_time);
  };
}


#endif
