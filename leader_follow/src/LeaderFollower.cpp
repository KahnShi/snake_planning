#include <leader_follow/LeaderFollower.h>
LeaderFollower::LeaderFollower(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
{
  m_nhp.param("sub_snake_pose_topic_name", m_sub_snake_pose_topic_name, std::string("/hydrus3/ground_truth"));
  m_nhp.param("pub_snake_start_flag", m_pub_snake_start_flag_topic_name, std::string("/teleop_command/start"));
  m_nhp.param("pub_snake_takeoff_flag", m_pub_snake_takeoff_flag_topic_name, std::string("/teleop_command/takeoff"));
  m_nhp.param("pub_snake_land_flag", m_pub_snake_land_flag_topic_name, std::string("/teleop_command/land"));
  m_nhp.param("pub_snake_joint_states_topic_name", m_pub_snake_joint_states_topic_name, std::string("/hydrus3/joints_ctrl"));
  m_nhp.param("pub_snake_flight_nav_topic_name", m_pub_snake_flight_nav_topic_name, std::string("/uav/nav"));

  /* subscriber & publisher */
  m_sub_snake_pose = m_nh.subscribe<geometry_msgs::PoseStamped>(m_sub_snake_pose_topic_name, 1, &LeaderFollower::snakePoseCallback, this);
  m_sub_snake_task_start_flag = m_nh.subscribe<std_msgs::Empty>(std::string("/task_start"), 1, &LeaderFollower::taskStartCallback, this);

  m_pub_snake_start_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_start_flag_topic_name, 1);
  m_pub_snake_takeoff_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_takeoff_flag_topic_name, 1);
  m_pub_snake_land_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_land_flag_topic_name, 1);
  m_pub_snake_joint_states = m_nh.advertise<sensor_msgs::JointState>(m_pub_snake_joint_states_topic_name, 1);
  m_pub_snake_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_snake_flight_nav_topic_name, 1);

  /* Init value */
  m_task_start_flag = false;

  usleep(2000000);
  ROS_INFO("LeaderFollower initialization finished.");
}

void LeaderFollower::snakeInitPose()
{
  std_msgs::Empty msg;
  m_pub_snake_start_flag.publish(msg);
  usleep(300000);
  m_pub_snake_takeoff_flag.publish(msg);
  usleep(5000000);
  ROS_INFO("Takeoff finished.");
  sensor_msgs::JointState joints_msg;
  joints_msg.position.push_back(1.57);
  joints_msg.position.push_back(0.0);
  joints_msg.position.push_back(0.0);
  m_pub_snake_joint_states.publish(joints_msg);
  usleep(3000000);
  ROS_INFO("Get to initial joints state.");
}

void LeaderFollower::taskStartCallback(std_msgs::Empty msg)
{
  m_task_start_flag = true;
  snakeInitPose();
}

void LeaderFollower::snakePoseCallback(const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  m_snake_pose = pose_msg->pose;
}
