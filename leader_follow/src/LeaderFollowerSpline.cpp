#include <leader_follow/LeaderFollowerSpline.h>

namespace leader_follower_spline
{
  LeaderFollowerSpline::LeaderFollowerSpline(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("sub_snake_odom_topic_name", m_sub_snake_odom_topic_name, std::string("/uav/state"));
    m_nhp.param("pub_snake_start_flag", m_pub_snake_start_flag_topic_name, std::string("/teleop_command/start"));
    m_nhp.param("pub_snake_takeoff_flag", m_pub_snake_takeoff_flag_topic_name, std::string("/teleop_command/takeoff"));
    m_nhp.param("pub_snake_land_flag", m_pub_snake_land_flag_topic_name, std::string("/teleop_command/land"));
    m_nhp.param("pub_snake_joint_states_topic_name", m_pub_snake_joint_states_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("pub_snake_flight_nav_topic_name", m_pub_snake_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("pub_snake_traj_path_topic_name", m_pub_snake_traj_path_topic_name, std::string("/traj_path"));
    m_nhp.param("snake_traj_order", m_snake_traj_order, 10);
    m_nhp.param("snake_links_number", m_n_snake_links, 4);
    m_nhp.param("snake_link_length", m_snake_link_length, 0.44);
    m_nhp.param("snake_average_vel", m_snake_average_vel, 0.5);
    m_nhp.param("spline_segment_time", m_spline_segment_time, 1.0);

    /* subscriber & publisher */
    m_sub_snake_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_snake_odom_topic_name, 1, &LeaderFollowerSpline::snakeOdomCallback, this);
    m_sub_snake_task_start_flag = m_nh.subscribe<std_msgs::Empty>(std::string("/task_start"), 1, &LeaderFollowerSpline::taskStartCallback, this);
    m_sub_control_points = m_nh.subscribe<geometry_msgs::PolygonStamped>(std::string("/control_points"), 1, &LeaderFollowerSpline::controlPointsCallback, this);

    m_pub_snake_start_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_start_flag_topic_name, 1);
    m_pub_snake_takeoff_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_takeoff_flag_topic_name, 1);
    m_pub_snake_land_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_land_flag_topic_name, 1);
    m_pub_snake_joint_states = m_nh.advertise<sensor_msgs::JointState>(m_pub_snake_joint_states_topic_name, 1);
    m_pub_snake_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_snake_flight_nav_topic_name, 1);
    m_pub_control_points_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/control_points_markers", 1);

    /* Init value */
    m_task_start_flag = false;
    m_snake_joint_states_vel_ptr = new double[m_n_snake_links + 1];
    m_snake_joint_states_ang_ptr = new double[m_n_snake_links + 1];
    m_snake_command_ptr = new SnakeCommand(m_nh, m_nhp);

    /* bspline */
    m_bspline_generator.onInit(m_snake_traj_order, true, m_pub_snake_traj_path_topic_name);
    m_snake_command_ptr->m_bspline_traj_ptr = &m_bspline_generator;
    m_snake_command_ptr->m_spline_segment_time = m_spline_segment_time;

    usleep(2000000);
    ROS_INFO("[LeaderFollowerSpline] Initialization finished.");
  }

  void LeaderFollowerSpline::snakeInitPose()
  {
    std_msgs::Empty msg;
    m_pub_snake_start_flag.publish(msg);
    usleep(300000);
    m_pub_snake_takeoff_flag.publish(msg);
    usleep(5000000);
    ROS_INFO("[LeaderFollowerSpline] Snake takeoff finished.");
    sensor_msgs::JointState joints_msg;
    joints_msg.position.push_back(0.0);
    joints_msg.position.push_back(1.57);
    joints_msg.position.push_back(0.0);
    m_pub_snake_joint_states.publish(joints_msg);
    usleep(5000000);
    /* Fly to initial position */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_x = 0.0;
    nav_msg.target_pos_y = 0.0;
    m_pub_snake_flight_nav.publish(nav_msg);
    usleep(5000000);
    /* Change yaw angle to initial value */
    aerial_robot_base::FlightNav nav_yaw_msg;
    nav_yaw_msg.header.frame_id = std::string("/world");
    nav_yaw_msg.header.stamp = ros::Time::now();
    nav_yaw_msg.header.seq = 2;
    nav_yaw_msg.psi_nav_mode = nav_msg.POS_MODE;
    nav_yaw_msg.target_psi = 0.0;
    m_pub_snake_flight_nav.publish(nav_yaw_msg);
    usleep(5000000);
    /* Adjust to initial position again*/
    m_pub_snake_flight_nav.publish(nav_msg);
    usleep(2000000);
    ROS_INFO("[LeaderFollowerSpline] Snake reach initial joints state.");
  }

  void LeaderFollowerSpline::taskStartCallback(std_msgs::Empty msg)
  {
    m_task_start_flag = true;
    snakeInitPose();
  }

  void LeaderFollowerSpline::snakeOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    m_snake_odom = *msg;
  }

  void LeaderFollowerSpline::controlPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
  {
    if (!m_control_point_vec.empty())
      m_control_point_vec.clear();
    for (int i = 0; i < msg->polygon.points.size(); ++i){
      m_control_point_vec.push_back(msg->polygon.points[i]);
    }
    visualizeControlPoints();
    splineInputParam();
  }

  void LeaderFollowerSpline::splineInputParam()
  {
    geometry_msgs::PolygonStamped control_polygon_points;
    for (int i = 0; i < m_control_point_vec.size(); ++i){
      geometry_msgs::Point32 time_point;
      // todo: segment_time should be decided by control points topic
      time_point.x = m_spline_segment_time * i;
      control_polygon_points.polygon.points.push_back(time_point);
      control_polygon_points.polygon.points.push_back(m_control_point_vec[i]);
      std::cout << "[" << m_control_point_vec[i].x << ", " << m_control_point_vec[i].y << ", " << m_control_point_vec[i].z << "]\n";
    }
    m_bspline_generator.bsplineParamInput(&control_polygon_points);
    m_bspline_generator.getDerive();
    //m_uav.m_bspline_traj_ptr = &m_bspline_generator;
    //m_uav.m_traj_updated = true;
  }

  void LeaderFollowerSpline::visualizeControlPoints()
  {
    visualization_msgs::MarkerArray control_point_markers;
    visualization_msgs::Marker control_point_marker, cylinder_marker;
    control_point_marker.ns = "control_points";
    control_point_marker.header.frame_id = std::string("/world");
    control_point_marker.header.stamp = ros::Time().now();
    control_point_marker.action = visualization_msgs::Marker::ADD;
    control_point_marker.type = visualization_msgs::Marker::SPHERE;

    cylinder_marker.ns = "cylinders";
    cylinder_marker.header.frame_id = std::string("/world");
    cylinder_marker.header.stamp = ros::Time().now();
    cylinder_marker.action = visualization_msgs::Marker::ADD;
    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
    
    for (int i = 0; i < m_control_point_vec.size()-1; ++i){
      control_point_marker.id = i;
      control_point_marker.pose.position.x = m_control_point_vec[i].x;
      control_point_marker.pose.position.y = m_control_point_vec[i].y;
      control_point_marker.pose.position.z = m_control_point_vec[i].z;
      control_point_marker.pose.orientation.x = 0.0;
      control_point_marker.pose.orientation.y = 0.0;
      control_point_marker.pose.orientation.z = 0.0;
      control_point_marker.pose.orientation.w = 1.0;
      control_point_marker.scale.x = 0.15;
      control_point_marker.scale.y = 0.15;
      control_point_marker.scale.z = 0.15;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      if (i < m_n_snake_links + 1){
        control_point_marker.color.a = 0.4;
      }
      control_point_markers.markers.push_back(control_point_marker);
    }

    // add cylinder representing trees
    // tree: -1.0, 0.3.  - 2.5, -0.82
    cylinder_marker.id = control_point_marker.id + 1;
    cylinder_marker.pose.position.x = 0.0;
    cylinder_marker.pose.position.y = 0.6;
    cylinder_marker.pose.position.z = 0.0;
    cylinder_marker.pose.orientation.x = 0.0;
    cylinder_marker.pose.orientation.y = 0.0;
    cylinder_marker.pose.orientation.z = 0.0;
    cylinder_marker.pose.orientation.w = 1.0;
    cylinder_marker.scale.x = 0.3;
    cylinder_marker.scale.y = 0.3;
    cylinder_marker.scale.z = 10.0;
    cylinder_marker.color.a = 1.0;
    cylinder_marker.color.r = 162.0f / 255.0f;
    cylinder_marker.color.g = 154.0f / 255.0f;
    cylinder_marker.color.b = 103.0f / 255.0f;
    control_point_markers.markers.push_back(cylinder_marker);

    cylinder_marker.id += 1;
    cylinder_marker.pose.position.x = -0.0;
    cylinder_marker.pose.position.y = -0.6;
    control_point_markers.markers.push_back(cylinder_marker);
    
    m_pub_control_points_markers.publish(control_point_markers);
  }


}
