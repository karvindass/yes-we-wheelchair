//ros dependency files
#include <ros.h>
#include <std_msgs/Int32.h>

// defines pins we will be pulling input from
#define encoderLPinA 1
#define encoderLPinB 2

//stores the interups encountered by encoder during time interval 100
//######## Below variable is commented out for testing ########
//volatile int encoderLCount = 0;

//testing variables 
int dummy = 0;
int encoderLCount = 0;

unsigned long interval = 100; // once this interval is reached, odometry motion measurement is taken
unsigned long previousTime = 0; // var used to check interval - compared to currentTime var

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
ros::Publisher encoderL("Encoder" ,&int_msg);

void setup() {
    nh.initNode();
    nh.advertise(Encoder);

    pinMode(encoderLPinA, INPUT_PULLUP); // configuring defined pins as inputs
    pinMode(encoderLPinB, INPUT_PULLUP);

    attachInterrupt(encoderLPinA, doEncoder0, RISING);
}

void doEncoder0() {
        if(digitalRead(encoderLPinB) == LOW) {
            encoderLCount++;
        } else {
            encoderLCount--;
        }

}

void loop()
{
  unsigned long currentTime = millis();
  if (currentTime - previousTime > interval) {
      
      // testing code 
      dummy += 1;
      encoderLCount = dummy;
      //end test code

      // data structure. 
      // Messages published from this encoder end in 0
      encoderLCount = (encoderLCount * 10) + 0;
      
      previousTime = currentTime;
      int_msg.data = encoderLCount;      
      encoderL.publish( &int_msg);
      nh.spinOnce();
      encoderLCount = 0;
  }
}
