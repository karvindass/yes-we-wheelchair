//ros dependency files
#include <ros.h>
#include <std_msgs/Int32.h>

// defines pins we will be pulling input from
#define encoderRPinA 1
#define encoderRPinB 2

//stores the interups encountered by encoder during time interval 100
//######## Below variable is commented out for testing ########
//volatile int encoderRCount = 0;

//testing variables
int dummy = 0;
int encoderRCount = 0;

unsigned long interval = 100; // once this interval is reached, odometry motion measurement is taken
unsigned long previousTime = 0; // var used to check interval - compared to currentTime var

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
ros::Publisher encoderR("Encoder" ,&int_msg);

void setup() {
    nh.initNode();
    nh.advertise(encoderR);

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
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {

      // testing code
      dummy += 1;
      encoderRCount = dummy;
      //end test code

      // data structure.
      // Messages published from this encoder end in 1
      encoderRCount = (encoderRCount * 10) + 1;

      previousTime = currentTime;
      int_msg.data = encoderRCount;
      chatter.publish( &int_msg);
      nh.spinOnce();
      encoderRCount = 0;
  }
}
