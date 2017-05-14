#include <leader_follow/LeaderFollower.h>

namespace leader_follower
{
  LeaderFollower::LeaderFollower(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("sub_snake_odom_topic_name", m_sub_snake_odom_topic_name, std::string("/uav/state"));
    m_nhp.param("pub_snake_start_flag", m_pub_snake_start_flag_topic_name, std::string("/teleop_command/start"));
    m_nhp.param("pub_snake_takeoff_flag", m_pub_snake_takeoff_flag_topic_name, std::string("/teleop_command/takeoff"));
    m_nhp.param("pub_snake_land_flag", m_pub_snake_land_flag_topic_name, std::string("/teleop_command/land"));
    m_nhp.param("pub_snake_joint_states_topic_name", m_pub_snake_joint_states_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("pub_snake_flight_nav_topic_name", m_pub_snake_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("snake_traj_order", m_snake_traj_order, 10);
    m_nhp.param("snake_traj_dev_order", m_snake_traj_dev_order, 4);
    m_nhp.param("snake_links_number", m_n_snake_links, 4);
    m_nhp.param("snake_link_length", m_snake_link_length, 0.44);
    m_nhp.param("snake_average_vel", m_snake_average_vel, 0.5);

    /* subscriber & publisher */
    m_sub_snake_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_snake_odom_topic_name, 1, &LeaderFollower::snakeOdomCallback, this);
    m_sub_snake_task_start_flag = m_nh.subscribe<std_msgs::Empty>(std::string("/task_start"), 1, &LeaderFollower::taskStartCallback, this);
    m_sub_sampling_points = m_nh.subscribe<geometry_msgs::PolygonStamped>(std::string("/sampling_points"), 1, &LeaderFollower::samplingPointsCallback, this);

    m_pub_snake_start_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_start_flag_topic_name, 1);
    m_pub_snake_takeoff_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_takeoff_flag_topic_name, 1);
    m_pub_snake_land_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_land_flag_topic_name, 1);
    m_pub_snake_joint_states = m_nh.advertise<sensor_msgs::JointState>(m_pub_snake_joint_states_topic_name, 1);
    // m_pub_snake_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_snake_flight_nav_topic_name, 1);
    m_pub_sample_points = m_nh.advertise<visualization_msgs::MarkerArray>("/sample_points_markers", 1);

    /* Init value */
    m_task_start_flag = false;
    m_snake_traj_param_x_ptr = new VectorXd(1);
    m_snake_traj_param_y_ptr = new VectorXd(1);
    m_snake_traj_param_z_ptr = new VectorXd(1);
    m_snake_sample_pos_x_ptr = new VectorXd(1);
    m_snake_sample_pos_y_ptr = new VectorXd(1);
    m_snake_sample_pos_z_ptr = new VectorXd(1);
    m_snake_sample_vel_x_ptr = new VectorXd(1);
    m_snake_sample_vel_y_ptr = new VectorXd(1);
    m_snake_sample_vel_z_ptr = new VectorXd(1);
    m_snake_sample_time_ptr = new VectorXd(1);
    m_snake_joint_states_vel_ptr = new double[m_n_snake_links];
    m_snake_joint_states_ang_ptr = new double[m_n_snake_links];
    m_snake_traj_ptr = new SamplingBasedTrajectory(m_nh, m_nhp, 2);

    usleep(2000000);
    ROS_INFO("[LeaderFollower] Initialization finished.");
  }

  void LeaderFollower::snakeInitPose()
  {
    std_msgs::Empty msg;
    m_pub_snake_start_flag.publish(msg);
    usleep(300000);
    m_pub_snake_takeoff_flag.publish(msg);
    usleep(5000000);
    ROS_INFO("[LeaderFollower] Snake takeoff finished.");
    sensor_msgs::JointState joints_msg;
    joints_msg.position.push_back(0.0);
    joints_msg.position.push_back(0.0);
    joints_msg.position.push_back(1.57);
    m_pub_snake_joint_states.publish(joints_msg);
    usleep(3000000);
    ROS_INFO("[LeaderFollower] Snake reach initial joints state.");
  }

  void LeaderFollower::taskStartCallback(std_msgs::Empty msg)
  {
    m_task_start_flag = true;
    snakeInitPose();
  }

  void LeaderFollower::snakeOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    m_snake_odom = *msg;
  }

  void LeaderFollower::samplingPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
  {
    int n_points = msg->polygon.points.size();
    MatrixXd sample_points = MatrixXd::Zero(n_points, 3);
    for (int i = 0; i < n_points; ++i){
      sample_points(i, 0) = msg->polygon.points[i].x;
      sample_points(i, 1) = msg->polygon.points[i].y;
      sample_points(i, 2) = msg->polygon.points[i].z;
    }
    getSamplePoints(sample_points);
    visualizeSamplePoints();

    /* Generate trajectory */
    m_snake_traj_ptr->m_traj_order = m_snake_traj_order;
    m_snake_traj_ptr->m_traj_dev_order = m_snake_traj_dev_order;
    m_snake_traj_ptr->m_n_samples = m_n_snake_samples;
    m_snake_traj_ptr->m_sample_x_pos_ptr = m_snake_sample_pos_x_ptr;
    m_snake_traj_ptr->m_sample_y_pos_ptr = m_snake_sample_pos_y_ptr;
    m_snake_traj_ptr->m_sample_z_pos_ptr = m_snake_sample_pos_z_ptr;
    m_snake_traj_ptr->m_sample_x_vel_ptr = m_snake_sample_vel_x_ptr;
    m_snake_traj_ptr->m_sample_y_vel_ptr = m_snake_sample_vel_y_ptr;
    m_snake_traj_ptr->m_sample_z_vel_ptr = m_snake_sample_vel_z_ptr;
    m_snake_traj_ptr->m_sample_time_ptr = m_snake_sample_time_ptr;
    m_snake_traj_ptr->m_traj_param_x_ptr = m_snake_traj_param_x_ptr;
    m_snake_traj_ptr->m_traj_param_y_ptr = m_snake_traj_param_y_ptr;
    m_snake_traj_ptr->m_traj_param_z_ptr = m_snake_traj_param_z_ptr;

    bool is_traj_available = m_snake_traj_ptr->generateTrajectory();
    ROS_INFO("[LeaderFollower] Trajectory generation finished.");
    if (is_traj_available){
      ROS_INFO("[LeaderFollower] Trajectory generation succeeded.");
      m_snake_traj_ptr->trajectoryVisualization();
    }
    else
      ROS_ERROR("[LeaderFollower] Trajectory generation failed");
  }

  void LeaderFollower::getSamplePoints(MatrixXd& points)
  {
    delete m_snake_sample_pos_x_ptr;
    delete m_snake_sample_pos_y_ptr;
    delete m_snake_sample_pos_z_ptr;
    delete m_snake_sample_vel_x_ptr;
    delete m_snake_sample_vel_y_ptr;
    delete m_snake_sample_vel_z_ptr;
    delete m_snake_sample_time_ptr;
    delete m_snake_traj_param_x_ptr;
    delete m_snake_traj_param_y_ptr;
    delete m_snake_traj_param_z_ptr;

    /* Add joints as points before sampling points in the trajectory */
    int n_samples = points.rows() / 2;
    m_n_snake_samples = n_samples + m_n_snake_links;
    m_snake_sample_pos_x_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_pos_y_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_pos_z_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_vel_x_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_vel_y_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_vel_z_ptr = new VectorXd(m_n_snake_samples);
    m_snake_sample_time_ptr = new VectorXd(m_n_snake_samples);
    m_snake_traj_param_x_ptr = new VectorXd(m_snake_traj_order * (m_n_snake_samples - 1));
    m_snake_traj_param_y_ptr = new VectorXd(m_snake_traj_order * (m_n_snake_samples - 1));
    m_snake_traj_param_z_ptr = new VectorXd(m_snake_traj_order * (m_n_snake_samples - 1));
    /* add links positions */
    // todo: manually set as L shape
    // todo: here assume 4 links
    // (*m_snake_sample_pos_x_ptr)[3] = m_snake_odom.pose.pose.position.x;
    // (*m_snake_sample_pos_y_ptr)[3] = m_snake_odom.pose.pose.position.y;
    // (*m_snake_sample_pos_z_ptr)[3] = m_snake_odom.pose.pose.position.z;
    (*m_snake_sample_pos_x_ptr)[3] = 0.0;
    (*m_snake_sample_pos_y_ptr)[3] = 0.0;
    (*m_snake_sample_pos_z_ptr)[3] = 2.0;
    (*m_snake_sample_pos_x_ptr)[2] = -0.44;
    (*m_snake_sample_pos_y_ptr)[2] = 0.0;
    (*m_snake_sample_pos_z_ptr)[2] = 2.0;
    (*m_snake_sample_pos_x_ptr)[1] = -0.88;
    (*m_snake_sample_pos_y_ptr)[1] = 0.0;
    (*m_snake_sample_pos_z_ptr)[1] = 2.0;
    (*m_snake_sample_pos_x_ptr)[0] = -0.88;
    (*m_snake_sample_pos_y_ptr)[0] = -0.44;
    (*m_snake_sample_pos_z_ptr)[0] = 2.0;

    // todo: set joiints initial velocity
    (*m_snake_sample_vel_x_ptr)[3] = -10000.0;
    (*m_snake_sample_vel_y_ptr)[3] = -10000.0;
    (*m_snake_sample_vel_z_ptr)[3] = -10000.0;
    (*m_snake_sample_vel_x_ptr)[2] = -10000.0;
    (*m_snake_sample_vel_y_ptr)[2] = -10000.0;
    (*m_snake_sample_vel_z_ptr)[2] = -10000.0;
    (*m_snake_sample_vel_x_ptr)[1] = -10000.0;
    (*m_snake_sample_vel_y_ptr)[1] = -10000.0;
    (*m_snake_sample_vel_z_ptr)[1] = -10000.0;
    (*m_snake_sample_vel_x_ptr)[0] = 0.0;
    (*m_snake_sample_vel_y_ptr)[0] = 0.0;
    (*m_snake_sample_vel_z_ptr)[0] = 0.0;
    /* add sampling points from normal planning algorithm */
    for (int i = 0; i < n_samples; ++i){
      (*m_snake_sample_pos_x_ptr)[i+m_n_snake_links] = points(2*i, 0);
      (*m_snake_sample_pos_y_ptr)[i+m_n_snake_links] = points(2*i, 1);
      (*m_snake_sample_pos_z_ptr)[i+m_n_snake_links] = points(2*i, 2);
      (*m_snake_sample_vel_x_ptr)[i+m_n_snake_links] = points(2*i+1, 0);
      (*m_snake_sample_vel_y_ptr)[i+m_n_snake_links] = points(2*i+1, 1);
      (*m_snake_sample_vel_z_ptr)[i+m_n_snake_links] = points(2*i+1, 2);
    }

    /* roughly estimate travel time */
    (*m_snake_sample_time_ptr)[m_n_snake_links-1] = 0.0;
    for (int i = m_n_snake_links-2; i >= 0; --i){
      double dist = getPointsDistance(i+1, i);
      (*m_snake_sample_time_ptr)[i] = (*m_snake_sample_time_ptr)[i+1] - dist / m_snake_average_vel;
    }
    for (int i = m_n_snake_links; i < m_n_snake_samples; ++i){
      double dist = getPointsDistance(i, i-1);
      (*m_snake_sample_time_ptr)[i] = (*m_snake_sample_time_ptr)[i-1] + dist / m_snake_average_vel;
    }
  }

  inline double LeaderFollower::getPointsDistance(int id_1, int id_2)
  {
    return sqrt(pow((*m_snake_sample_pos_x_ptr)[id_1] - (*m_snake_sample_pos_x_ptr)[id_2], 2.0)
                +pow((*m_snake_sample_pos_y_ptr)[id_1] - (*m_snake_sample_pos_y_ptr)[id_2], 2.0)
                +pow((*m_snake_sample_pos_z_ptr)[id_1] - (*m_snake_sample_pos_z_ptr)[id_2], 2.0)
                );
  }

  void LeaderFollower::visualizeSamplePoints()
  {
    visualization_msgs::MarkerArray sample_point_markers;
    visualization_msgs::Marker sample_point_marker;
    sample_point_marker.ns = "sample_points";
    sample_point_marker.header.frame_id = std::string("/world");
    sample_point_marker.header.stamp = ros::Time().now();
    sample_point_marker.action = visualization_msgs::Marker::ADD;
    sample_point_marker.type = visualization_msgs::Marker::SPHERE;

    for (int i = 0; i < m_n_snake_samples; ++i){
      sample_point_marker.id = i;
      sample_point_marker.pose.position.x = (*m_snake_sample_pos_x_ptr)[i];
      sample_point_marker.pose.position.y = (*m_snake_sample_pos_y_ptr)[i];
      sample_point_marker.pose.position.z = (*m_snake_sample_pos_z_ptr)[i];
      sample_point_marker.pose.orientation.x = 0.0;
      sample_point_marker.pose.orientation.y = 0.0;
      sample_point_marker.pose.orientation.z = 0.0;
      sample_point_marker.pose.orientation.w = 1.0;
      sample_point_marker.scale.x = 0.15;
      sample_point_marker.scale.y = 0.15;
      sample_point_marker.scale.z = 0.15;
      sample_point_marker.color.a = 1;
      sample_point_marker.color.r = 0.0f;
      sample_point_marker.color.g = 1.0f;
      sample_point_marker.color.b = 0.0f;
      if (i < m_n_snake_links){
        sample_point_marker.color.a = 0.4;
      }
      sample_point_markers.markers.push_back(sample_point_marker);
    }
    m_pub_sample_points.publish(sample_point_markers);
  }
}
