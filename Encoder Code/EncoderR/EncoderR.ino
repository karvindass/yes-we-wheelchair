//ros dependency files
#include <ros.h>
#include <std_msgs/Int32.h>

// defines pins we will be pulling input from
#define encoderRPinA 1
#define encoderRPinB 2

//stores the interups encountered by encoder during time interval 100
volatile int encoderRCount = 0;

unsigned long interval = 100; // once this interval is reached, odometry motion measurement is taken
unsigned long previousTime = 0; // var used to check interval - compared to currentTime var

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
ros::Publisher chatter("chatter" ,&int_msg);

void setup() {
    nh.initNode();
    nh.advertise(chatter);

    pinMode(encoderRPinA, INPUT_PULLUP); // configuring defined pins as inputs
    pinMode(encoderRPinB, INPUT_PULLUP);

    attachInterrupt(encoderRPinA, doEncoder0, RISING);
}

void doEncoder0() {
        if(digitalRead(encoderRPinB) == LOW) {
            encoderRCount++;
        } else {
            encoderRCount--;
        }

}

void loop()
{
  if (currentTime - previousTime > interval) {

      previousTime = currentTime;
      int_msg.data = encoderRCount;
      chatter.publish( &int_msg);
      nh.spinOnce();
      encoderRCount = 0;
  }
}
