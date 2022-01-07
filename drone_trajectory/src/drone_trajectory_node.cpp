#include "ros/ros.h"
//#include <ignition/math.hh>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  // initialize node name
  ros::init(argc, argv, "drone_trajectory_node");
  ros::NodeHandle n;
  ros::Publisher drone_traj_pub = n.advertise<geometry_msgs::Vector3>("drone_trajectory", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  geometry_msgs::Vector3 point;
  point.x = 0;
  point.y = 0;
  point.z = 0;

  while (ros::ok())
  {
    point.x = point.x + 0.1*count;
    drone_traj_pub.publish(point);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
