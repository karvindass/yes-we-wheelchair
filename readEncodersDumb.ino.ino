//ros dependency files
#include <ros.h>
#include <std_msgs/Int32.h>

// defines pins we will be pulling input from
#define encoder0PinA 1 
#define encoder0PinB 2 

//stores the interups encountered by encoder during time interval 100
volatile int encoder0Count = 0; 

unsigned long interval = 100; // once this interval is reached, odometry motion measurement is taken
unsigned long previousTime = 0; // var used to check interval - compared to currentTime var

ros::NodeHandle nh;

std_msgs::Int32 int_msg;
ros::Publisher chatter("chatter" ,&int_msg);

void setup() {
    nh.initNode();
    nh.advertise(chatter);
  
    pinMode(encoder0PinA, INPUT_PULLUP); // configuring defined pins as inputs
    pinMode(encoder0PinB, INPUT_PULLUP);

    attachInterrupt(encoder0PinA, doEncoder0, RISING);
}

void doEncoder0() {
        if(digitalRead(encoder0PinB) == LOW) {
            encoder0Count++;
        } else {
            encoder0Count--;
        }
    
}

void loop()
{
  if (currentTime - previousTime > interval) {
      
      previousTime = currentTime;
      int_msg.data = encoder0Count;
      chatter.publish( &int_msg);
      nh.spinOnce();
      encoder0Count = 0;
  }
}

