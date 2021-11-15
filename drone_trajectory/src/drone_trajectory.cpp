#include "ros/ros.h"
#include <ignition/math.hh>
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_trajectory");
  ros::NodeHandle n;
  ros::Publisher drone_traj_pub = n.advertise<ignition::mat:Vector3d>("drone_trajectory", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  ignition::math:Vector3d point(0, 0, 0);
  while (ros::ok())
  {
    point.X() = point.X() + count;
    chatter_pub.publish(point);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
