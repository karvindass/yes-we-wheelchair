/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 #include <cmath>
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/Int32.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// include global variables that store values of x,y,z
double XVal = 0;
double YVal = 0;
double ThetaVal = 0;

// include global variables for constants
double wheelDiameter = 0.304; // diameter of wheel in meters
double circ = M_PI * wheelDiameter; // circumference of wheel
int res = 2500; // resolution of the encoder in (pulse/rotation)
double wheelDist = 0.9; // distance between wheels

 // function called for each message received on Encoder topic
void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
	// flag for which encoder is sending the message
  bool leftEnc = false;
  // changes in X,Y and theta
  double deltX = 0, deltY = 0, deltTheta = 0, deltS = 0;
  double deltR = 0, deltL = 0;

  int encoderReturn = msg->data;
  int encoderValue = (encoderReturn - (encoderReturn % 10)) / 10;

  deltS = (encoderValue / 2) * (circ / res);

  // determine which encoder is sending data
  if ((encoderReturn % 10) == 0) {
    // Data is coming in from left encoder
    leftEnc = true;
  }
  else if((encoderReturn % 10) == 1) {
    // Data is coming in from right encoder
    leftEnc = false; // redundant but keep
  }

  // determine change in theta
  if (leftEnc) {
    deltTheta = -(circ / res) * encoderValue / wheelDist;
  }
  else {
    deltTheta = (circ / res) * encoderValue / wheelDist;
  }

  // Determine delta x and y
  deltX = deltS * cos(ThetaVal + (deltTheta / 2) );
  deltY = deltS * sin(ThetaVal + (deltTheta / 2) );

  // add to existing values for X, Y and theta
  XVal += deltX;
  YVal += deltY;
  ThetaVal += deltTheta;

  if (ThetaVal >= 2 * M_PI)) {
    ThetaVal -= 2 * M_PI;
  } else if (ThetaVal < 0) {
    ThetaVal += 2 * M_PI;
  }

	// use ROS_INFO to send information
  ROS_INFO("%d",YVal);
  // Currently sending only Y value
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "OdomCreator");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
   ros::Subscriber sub = n.subscribe("Encoder", 1000, chatterCallback);
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("OdomTopic", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

   }
// %EndTag(FULLTEXT)%
