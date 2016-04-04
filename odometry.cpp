/**
* Odometry code adapted from the arduino readEncodersNew.ino files
* and http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
*
* Written for VIP: Wheelchair project @ Gatech
*
**/



/*
* recommended add below packages in manifest
*<depend package="tf"/>
*<depend package="nav_msgs"/>
*/

// below file publishes odometry info for a set of specified conditions

// objective is a way to incorporate a subscriber to pull in information
// also process the data

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <cmath>

int main(int argc, char** argv){
  //ros::init(argc, argv, "odometry_publisher");
  ros::init(argc, argv, "Encoder")

  ros::NodeHandle n;
  //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  //tf::TransformBroadcaster odom_broadcaster;

  // variables to be connected to input from the arduinos
  int inptLeft = 0;
  int inptRight = 0;

  double diameter = 0.304; // diameter of the wheel in meters
  double circ = M_PI * 0.304; // the circumference of the wheel in meters
  int res = 2500; // resolution of the encoder in (pulse/sec)

  //dummy variables to be re-written
  // x,y,& theta will be published at the end of the file
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  //change of x, y, and theta
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;

  // time components relevent
  // ros::Time current_time, last_time;
  // current_time = ros::Time::now();
  // last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();// check for incoming messages
    current_time = ros::Time::now();

    // below is code adapted from readEncodersNew.ino
    double dsl = inptLeft * circ / res; // distance change in left wheel
    double dsr = inptRight * circ / res; // distance change in right wheel

    // dth, dx, dy are change in theta, x, and y
    double ds = (dsr + dsl) / 2.0;
    double dth = (dsr - dsl) / 0.9;
    double dx = ds * cos(th + dth/2);
    double dy = ds * sin(th + dth/2);
    // end of arduino adapted code

    x += dx;
    y += dy;
    th += dth;

    // use to ensure end value of theta is between 0 and 2PI
    if (th >= 2 * M_PI) {
      th -= 2 * M_PI;
    } else if (th < 0) {
      th += 2 * M_PI;
    }



    // below is odometry package specific code


    /**
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

The Code Expl
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    //below I changed vx, vy, & vth to dx, dy, and dth
    // will need to adress this problem at some point in time
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = dy;
    odom.twist.twist.angular.z = dth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
    dummy;
    **/
  }
}
