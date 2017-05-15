#ifndef LEADER_FOLLOWER_SPLINE_H_
#define LEADER_FOLLOWER_SPLINE_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
//#include <aerial_robot_base/FlightNav.h>
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

/* local library */
#include <bspline_ros/bsplineGenerate.h>

using namespace Eigen;

namespace leader_follower_spline{
  class LeaderFollowerSpline
  {
  public:
    LeaderFollowerSpline(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~LeaderFollowerSpline(){}

  private:
    nav_msgs::Odometry m_snake_odom;
    bool m_task_start_flag;
    /* Subscriber */
    ros::NodeHandle m_nh, m_nhp;
    ros::Subscriber m_sub_snake_task_start_flag;
    ros::Subscriber m_sub_snake_odom;
    ros::Subscriber m_sub_control_points;

    /* Publisher */
    ros::Publisher m_pub_snake_start_flag;
    ros::Publisher m_pub_snake_takeoff_flag;
    ros::Publisher m_pub_snake_land_flag;
    ros::Publisher m_pub_snake_joint_states;
    ros::Publisher m_pub_snake_flight_nav;
    ros::Publisher m_pub_control_points_markers;

    /* Topic name */
    std::string m_sub_snake_odom_topic_name;
    std::string m_pub_snake_start_flag_topic_name;
    std::string m_pub_snake_takeoff_flag_topic_name;
    std::string m_pub_snake_land_flag_topic_name;
    std::string m_pub_snake_joint_states_topic_name;
    std::string m_pub_snake_flight_nav_topic_name;
    std::string m_pub_snake_traj_path_topic_name;

    /* Snake states */
    int m_snake_traj_order;
    int m_n_snake_links;
    double m_snake_link_length;
    double m_snake_average_vel;
    double *m_snake_joint_states_vel_ptr;
    double *m_snake_joint_states_ang_ptr;

    /* bspline generator */
    bool m_snake_traj_bspline_mode;
    bsplineGenerate m_bspline_generator;
    double m_spline_segment_time;
    std::vector<geometry_msgs::Point32> m_control_point_vec;

    void snakeInitPose();
    void taskStartCallback(std_msgs::Empty msg);
    void snakeOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void controlPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
    bool generateTrajectory();
    void splineInputParam();
    void visualizeControlPoints();
  };
}
#endif
