#include <leader_follow/SnakeCommand.h>

namespace snake_command{
  SnakeCommand::SnakeCommand(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("pub_flight_nav_topic_name", m_pub_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("pub_joints_ctrl_topic_name", m_pub_joints_ctrl_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("sub_move_start_flag_topic_name", m_sub_move_start_flag_topic_name, std::string("/move_start"));
    m_nhp.param("sub_joint_states_topic_name", m_sub_joint_states_topic_name, std::string("/hydrus3/joint_states"));
    m_nhp.param("sub_base_link_odom_topic_name", m_sub_base_link_odom_topic_name, std::string("/uav/state"));
    m_nhp.param("snake_links_number", m_n_links, 4);
    m_nhp.param("snake_link_length", m_link_length, 0.44);
    m_nhp.param("control_period", m_control_period, 0.05);

    m_move_start_flag = false;
    m_links_pos_ptr = new tf::Vector3[m_n_links + 1];
    m_links_vel_ptr = new tf::Vector3[m_n_links + 1];
    m_joints_ang_ptr = new double[m_n_links + 1];
    m_joints_ang_vel_ptr = new double[m_n_links + 1];

    // /hydrus3/joint_states get joints angle and angle vel
    // /uav/state get uav link2's global position, velocity, orientation
    m_pub_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_flight_nav_topic_name, 1);
    m_pub_joints_ctrl = m_nh.advertise<sensor_msgs::JointState>(m_pub_joints_ctrl_topic_name, 1);

    m_sub_move_start_flag = m_nh.subscribe<std_msgs::Empty>(m_sub_move_start_flag_topic_name, 1, &SnakeCommand::moveStartFlagCallback, this);
    m_sub_joint_states = m_nh.subscribe<sensor_msgs::JointState>(m_sub_joint_states_topic_name, 1, &SnakeCommand::jointStatesCallback, this);
    m_sub_base_link_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_base_link_odom_topic_name, 1, &SnakeCommand::baseLinkOdomCallback, this);
    m_timer = m_nh.createTimer(ros::Duration(m_control_period), &SnakeCommand::controlCallback, this);

    m_traj_start_time = -1.0;
    m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
  }

  void SnakeCommand::controlCallback(const ros::TimerEvent& e)
  {
    if (!m_move_start_flag)
      return;
    if (m_traj_start_time < 0)
      m_traj_start_time = e.current_real.toSec();

    m_traj_current_time = e.current_real.toSec();

    //directTrackGlobalTrajectory();
    transformTrackGlobalTrajectory();

    /* velocity transform between link1 and link2 */
    // tf::Matrix3x3 rot_mat;
    // rot_mat.setRPY(0.0, 0.0, m_joints_ang_ptr[1]);
    // tf::Vector3 v_l1(-0.44, 0.0, 0.0), v_l2;
    // v_l2 = v_l1 * rot_mat;
    // std::cout << "v_l2: " << v_l2.x() << ", " << v_l2.y() << ", " << v_l2.z() << "\n";
  }

  void SnakeCommand::getPreviousLink(tf::Vector3 &prev_link, tf::Vector3 cur_link, double &joint_ang, double start_time)
  {
    getPreviousLinkFromSpline(prev_link, cur_link, m_link_length, start_time);
    joint_ang = atan2(prev_link.y() - cur_link.y(), prev_link.x() - cur_link.x());
    if (joint_ang < 0)
      joint_ang = -(3.14159 + joint_ang);
    else
      joint_ang = 3.14159 - joint_ang;
  }

  void SnakeCommand::getPreviousLinkFromSpline(tf::Vector3 &prev_link, tf::Vector3 cur_link, double link_length, double start_time)
  {
    double time_gap = 0.05;
    while (1){
      start_time += time_gap;
      if (start_time > m_bspline_traj_ptr->m_tn){
        start_time = m_bspline_traj_ptr->m_tn;
        break;
      }
      tf::Vector3 point = vectorToVector3(m_bspline_traj_ptr->evaluate(start_time));
      double dist = cur_link.distance(point);
      if (link_length - dist > 0)
        continue;
      else
        break;

    }
    prev_link = vectorToVector3(m_bspline_traj_ptr->evaluate(start_time));
  }

  void SnakeCommand::transformTrackGlobalTrajectory()
  {
    int link_id = 1; // start from 0 to 4
    // todo: mannually set
    m_traj_bias_start_time = m_spline_segment_time * (m_n_links + 1 - link_id);
    bool yaw_mode = false;
    // link2
    m_traj_bias_start_time = m_spline_segment_time * (m_n_links - link_id);
    double time_factor = 1.0;
    double current_traj_time = (m_traj_current_time - m_traj_start_time) / time_factor + m_traj_bias_start_time;
    if (current_traj_time >= m_bspline_traj_ptr->m_tn){
      if (current_traj_time - m_bspline_traj_ptr->m_tn < 0.1)
        ROS_INFO("\nArrived at last control point. \n");
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.header.frame_id = std::string("/world");
      nav_msg.header.stamp = ros::Time::now();
      nav_msg.header.seq = 1;
      nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
      nav_msg.target_vel_x = 0.0;
      nav_msg.target_vel_y = 0.0;
      nav_msg.psi_nav_mode = nav_msg.VEL_MODE;
      nav_msg.target_psi = 0.0;
      m_pub_flight_nav.publish(nav_msg);
      return;
    }
    // link2
    tf::Vector3 des_world_vel[5];
    des_world_vel[link_id] = vectorToVector3(m_bspline_traj_ptr->evaluateDerive(current_traj_time)) / time_factor;
    tf::Vector3 des_world_pos[5];
    des_world_pos[link_id] = vectorToVector3(m_bspline_traj_ptr->evaluate(current_traj_time));
    tf::Vector3 real_world_pos[5];
    double des_joint_ang[5];
    for (int i = 0; i < 5; ++i)
      real_world_pos[i] = m_links_pos_ptr[i];
    for (int i = link_id-1; i >= 0; --i){
      //getPreviousLink(des_world_pos[i], des_world_pos[i+1], des_joint_ang[i+1], current_traj_time);
      getPreviousLink(des_world_pos[i], real_world_pos[i+1], des_joint_ang[i+1], current_traj_time);
      std::cout << "id " << i << "] ang: " << des_joint_ang[i+1] << ".\n";
      std::cout << "prev_link:  " << des_world_pos[i].x() << ", " <<des_world_pos[i].y()
                << ". cur_link: " << real_world_pos[i+1].x() << ", " << real_world_pos[i+1].y() << "\n";
    }
    std::cout << "\n";
    for (int i = link_id; i <= 2; ++i)
      des_joint_ang[i+1] = m_joints_ang_ptr[i+1];
    sensor_msgs::JointState joints_msg;
    joints_msg.position.push_back(des_joint_ang[1]);
    joints_msg.position.push_back(des_joint_ang[2]);
    joints_msg.position.push_back(des_joint_ang[3]);
    m_pub_joints_ctrl.publish(joints_msg);

    std::vector<double> des_yaw = m_bspline_traj_ptr->evaluateYaw(current_traj_time);

    tf::Matrix3x3 r_z; r_z.setRPY(0, 0, m_base_link_ang.z());

    /* pid control in trajectory tracking */
    tf::Vector3 traj_track_p_term =  (des_world_pos[1] - real_world_pos[1]) * 0.45;
    /* feedforward */
    tf::Vector3 vel = des_world_vel[1] + traj_track_p_term;

    /* yaw */
    double yaw_p_term = des_yaw[1] - m_base_link_ang.getZ();
    // std::cout << "snake yaw: " << m_base_link_ang.z() / 3.14 * 180.0 << ", des yaw: " << des_yaw[1] / 3.14 * 180.0 << ", yaw vel: " << des_yaw[0] / 3.14 * 180.0 << "\n";
    if (yaw_p_term > 1.57)
      yaw_p_term -= 3.14;
    else if (yaw_p_term < -1.57)
      yaw_p_term += 3.14;
    double yaw_vel = des_yaw[0] + yaw_p_term * 1.0;
    double yaw_pos = des_yaw[1];
    /* Judge if the controller is locally based on uav coordinate. */
    // if (!m_global_coordinate_control_mode){
    //   uav_vel = r_z.inverse() * uav_vel;
    // }

    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
    nav_msg.target_vel_x = vel.getX();
    nav_msg.target_vel_y = vel.getY();
    if (yaw_mode){
      // nav_msg.psi_nav_mode = nav_msg.VEL_MODE;
      // nav_msg.target_psi = yaw_vel;
      nav_msg.psi_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_psi = yaw_pos;
    }
    else{
      nav_msg.psi_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_psi = 0.0;
    }
    m_pub_flight_nav.publish(nav_msg);

  }

  void SnakeCommand::directTrackGlobalTrajectory()
  {
    // todo: mannually set
    m_traj_bias_start_time = m_spline_segment_time * m_n_links;
    bool yaw_mode = false;
    double current_traj_time = (m_traj_current_time - m_traj_start_time) + m_traj_bias_start_time;
    if (current_traj_time >= m_bspline_traj_ptr->m_tn){
      if (current_traj_time - m_bspline_traj_ptr->m_tn < 0.1)
        ROS_INFO("\nArrived at last control point. \n");
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.header.frame_id = std::string("/world");
      nav_msg.header.stamp = ros::Time::now();
      nav_msg.header.seq = 1;
      nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
      nav_msg.target_vel_x = 0.0;
      nav_msg.target_vel_y = 0.0;
      nav_msg.psi_nav_mode = nav_msg.VEL_MODE;
      nav_msg.target_psi = 0.0;
      m_pub_flight_nav.publish(nav_msg);
      return;
    }
    tf::Vector3 des_world_vel = vectorToVector3(m_bspline_traj_ptr->evaluateDerive(current_traj_time));
    tf::Vector3 des_world_pos = vectorToVector3(m_bspline_traj_ptr->evaluate(current_traj_time));
    tf::Vector3 real_world_pos;
    // link1
    real_world_pos = m_links_pos_ptr[0];
    std::cout << "des world: " << current_traj_time << ": " << des_world_pos.x() << ", " << des_world_pos.y() << ", vel: "
              << des_world_vel.x() << ", " << des_world_vel.y() << "\n" ;
    std::cout << "real world: " << real_world_pos.x() << ", " << real_world_pos.y() << "\n\n";

    std::vector<double> des_yaw = m_bspline_traj_ptr->evaluateYaw(current_traj_time);
    tf::Matrix3x3 r_z; r_z.setRPY(0, 0, m_base_link_ang.z());

    /* pid control in trajectory tracking */
    tf::Vector3 traj_track_p_term =  (des_world_pos - real_world_pos) * 0.45;
    m_traj_track_i_term_accumulation += (des_world_pos - real_world_pos) * m_control_period;
    tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * 0.0 * 0.1;

    /* feedforward */
    tf::Vector3 vel = des_world_vel + traj_track_p_term + traj_track_i_term;

    /* yaw */
    double yaw_p_term = des_yaw[1] - m_base_link_ang.getZ();
    // std::cout << "snake yaw: " << m_base_link_ang.z() / 3.14 * 180.0 << ", des yaw: " << des_yaw[1] / 3.14 * 180.0 << ", yaw vel: " << des_yaw[0] / 3.14 * 180.0 << "\n";
    if (yaw_p_term > 1.57)
      yaw_p_term -= 3.14;
    else if (yaw_p_term < -1.57)
      yaw_p_term += 3.14;
    double yaw_vel = des_yaw[0] + yaw_p_term * 1.0;
    double yaw_pos = des_yaw[1] + des_yaw[0] * 2 * m_control_period;
    /* Judge if the controller is locally based on uav coordinate. */
    // if (!m_global_coordinate_control_mode){
    //   uav_vel = r_z.inverse() * uav_vel;
    // }

    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
    nav_msg.target_vel_x = vel.getX();
    nav_msg.target_vel_y = vel.getY();
    if (yaw_mode){
      // nav_msg.psi_nav_mode = nav_msg.VEL_MODE;
      // nav_msg.target_psi = yaw_vel;
      nav_msg.psi_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_psi = yaw_pos;
    }
    else{
      nav_msg.psi_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_psi = 0.0;
    }
    m_pub_flight_nav.publish(nav_msg);

  }

  void SnakeCommand::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
  {
    for (int i = 0; i < joints_msg->position.size(); ++i){
      m_joints_ang_ptr[i+1] = joints_msg->position[i];
      m_joints_ang_vel_ptr[i+1] = joints_msg->velocity[i];
    }
    /* calculate every links data from base_link */
    for (int i = 1; i <= 5; ++i){
      tf::StampedTransform transform;
      geometry_msgs::Twist link_twist;
      m_tf_listener.lookupTransform("/world", "/link" + std::to_string(i), ros::Time(0), transform);
      m_tf_listener.lookupTwist("/world", "/link" + std::to_string(i), ros::Time(0), ros::Duration(0.1), link_twist);
      m_links_pos_ptr[i-1].setValue(transform.getOrigin().x(),
                                    transform.getOrigin().y(),
                                    transform.getOrigin().z());
      m_links_vel_ptr[i-1].setValue(link_twist.linear.x,
                                    link_twist.linear.y,
                                    link_twist.linear.z);
      // std::cout << "/link" + std::to_string(i) << ": " << link_twist.linear.x << ", "
      //           << link_twist.linear.y << ", " << link_twist.linear.z << "\n";
    }
  }

  void SnakeCommand::baseLinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    m_base_link_odom = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    m_base_link_ang.setValue(r, p, y);
    m_base_link_pos.setValue(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
    m_base_link_vel.setValue(odom_msg->twist.twist.linear.x,
                             odom_msg->twist.twist.linear.y,
                             odom_msg->twist.twist.linear.z);
  }

  void SnakeCommand::moveStartFlagCallback(const std_msgs::Empty msg)
  {
    m_move_start_flag = true;
  }

  inline tf::Vector3 SnakeCommand::vectorToVector3(std::vector<double> vec)
  {
    tf::Vector3 vec3(vec[0], vec[1], vec[2]);
    return vec3;
  }

}
