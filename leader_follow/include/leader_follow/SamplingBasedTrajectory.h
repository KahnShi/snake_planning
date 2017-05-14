#ifndef SAMPLING_BASED_TRAJECTORY_H_
#define SAMPLING_BASED_TRAJECTORY_H_

#include <iostream>
/* qp solver */
#include <qpOASES.hpp>
/* math */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>
/* ros */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
using namespace Eigen;
USING_NAMESPACE_QPOASES

namespace sampling_based_trajectory
{
  class SamplingBasedTrajectory
  {
  public:
    SamplingBasedTrajectory(ros::NodeHandle nh, ros::NodeHandle nhp, int dimensions, std::string traj_path_pub_topic_name=std::string("/traj_path"), double lambda_H = 0.1, int n_wsr=30, double debug = true, double visualize_unit_time=0.05);
    ~SamplingBasedTrajectory(){}

    ros::NodeHandle m_nh, m_nhp;
    bool m_debug;
    /* Trajectory */
    int m_dim;
    int m_traj_order;
    int m_traj_dev_order;
    int m_n_samples;
    int m_n_wsr;
    VectorXd *m_sample_x_pos_ptr;
    VectorXd *m_sample_y_pos_ptr;
    VectorXd *m_sample_z_pos_ptr;
    VectorXd *m_sample_x_vel_ptr;
    VectorXd *m_sample_y_vel_ptr;
    VectorXd *m_sample_z_vel_ptr;
    VectorXd *m_sample_time_ptr;
    VectorXd *m_traj_param_x_ptr;
    VectorXd *m_traj_param_y_ptr;
    VectorXd *m_traj_param_z_ptr;
    double m_lambda_H;

    /* Visualization */
    std::string m_traj_path_pub_topic_name;
    ros::Publisher m_pub_traj_path;
    double m_visualize_unit_time;

    void trajectoryVisualization();
    inline int permutation(int n, int order);
    bool generateTrajectory(VectorXd *sample_pos_ptr, VectorXd *sample_vel_ptr, VectorXd *sample_time_ptr, VectorXd *traj_param_ptr);
    bool generateTrajectory();
    Vector3d getnOrderPointFromTrajectory(int order, double t);
    double getnOrderPointFromTrajectory(int order, double t, int sample_id, VectorXd *traj_param_ptr);
  };
}
#endif
