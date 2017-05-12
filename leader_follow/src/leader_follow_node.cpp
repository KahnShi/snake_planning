#include <leader_follow/LeaderFollower.h>
using namespace leader_follower;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "leader_follower");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  LeaderFollower* leader_follower = new LeaderFollower(nh, nhp);

  ros::spin();
  delete leader_follower;
  return 0;
}
