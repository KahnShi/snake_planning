#include <leader_follow/LeaderFollowerSpline.h>
using namespace leader_follower_spline;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "leader_follower_spline");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  LeaderFollowerSpline* leader_follower_spline = new LeaderFollowerSpline(nh, nhp);

  ros::spin();
  delete leader_follower_spline;
  return 0;
}
