#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include <sstream>

int main(int argc, char **argv)
{
  // initialize node name
  ros::init(argc, argv, "drone_trajectory_node");
  ros::NodeHandle n;
  ros::Publisher drone_traj_pub = n.advertise<geometry_msgs::Vector3>("tension_direction", 1000);  //topic name
  ros::Publisher drone_tension_mag_pub = n.advertise<std_msgs::Float32>("tension_mag",1000);
  ros::Rate loop_rate(10);
  int count = 0;
  std_msgs::Float32 magnitude;
  magnitude.data = 1000.0;
  geometry_msgs::Vector3 point;
  point.x = -532;
  point.y = 162;
  point.z = 2;

  while (ros::ok())
  {
    point.x = point.x + 0.0001*count;
    point.y = point.y - 0.00001*count;
    
    drone_traj_pub.publish(point);
    drone_tension_mag_pub.publish(magnitude);
 
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
