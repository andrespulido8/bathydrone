#include "ros/ros.h"
//#include <ignition/math.hh>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include <sstream>

int main(int argc, char **argv)
{
  // initialize node name
  ros::init(argc, argv, "drone_trajectory_node");
  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::Publisher drone_traj_pub = n1.advertise<geometry_msgs::Vector3>("tension_direction", 1000);  //topic name
  ros::Publisher drone_tension_mag_pub = n2.advertise<std_msgs::Float32>("tension_mag",1000);
  ros::Rate loop_rate(10);
  int count = 0;
  std_msgs::Float32 magnitude;
  magnitude.data = 100000.0;
  geometry_msgs::Vector3 point;
  point.x = -532;
  point.y = 162;
  point.z = 2;

  while (ros::ok())
  {
    point.x = point.x + 0.0001*count;
    
    drone_traj_pub.publish(point);
    drone_tension_mag_pub.publish(magnitude);
 
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
