// ROS Odom Includes
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// defining the pins. encoder0 is on the left, 1 is on the right
#define encoder0PinA 2
#define encoder0PinB 5
#define encoder1PinA 3
#define encoder1PinB 4

volatile long encoder0Count = 0;
volatile long encoder1Count = 0;

unsigned long interval = 100;
unsigned long previousTime = 0;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

// testing ROS serial values
double testx = 5;
double testy = 10;
double testTheta = 0.77;

double diameter = 0.304; // diameter of the wheel in meter
int res = 2500; // resolution of the encoder in (pulse/sec)

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
  // ROS
    nh.initNode();
  broadcaster.init(nh);
  
  // Old Odom
    pinMode(encoder0PinA, INPUT_PULLUP);
    pinMode(encoder0PinB, INPUT_PULLUP);
    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);

    attachInterrupt(0, doEncoder0, RISING);
    attachInterrupt(1, doEncoder1, RISING);
    //Serial.begin (9600);
    //Serial.println("START READING");
}

void doEncoder0() {
        if(digitalRead(encoder0PinB) == LOW) {
            encoder0Count++;
        }
        else {
            encoder0Count--;
        }
    
}

void doEncoder1() {
  
  
    //if(digitalRead(encoder1PinA) == HIGH) {
    if(digitalRead(encoder1PinB) == LOW) {
        encoder1Count++;
       
    }
    else {
        encoder1Count--;
    }
}

void loop () {
    unsigned long currentTime = millis();


        

    if (currentTime - previousTime > interval) {
        previousTime = currentTime;
        
        double dsl = encoder0Count * diameter / res;
        encoder0Count = 0; // delta_s_left
        double dsr = encoder1Count * diameter / res; // delta_s_right
        encoder1Count = 0;
        double ds = (dsr + dsl) / 2.0;
        double dtheta = (dsr-dsl) / 0.9;
        double dx = ds * cos(theta + dtheta/2);
        double dy = ds * sin(theta + dtheta/2);

        x += dx;
        y += dy;
        theta += dtheta;
        
        if (theta >= PI) {
          theta -= 2 * PI;
        } else if (theta <= -PI) {
          theta += 2 * PI;
        }
          
            // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  delay(10);
//        
//         
//       Serial.print(x,DEC);
//       Serial.print("\t");
//       Serial.print(y,DEC);
//       Serial.print("\t");
//       Serial.println(theta,DEC);
//    
//      // Serial.println(millis());

    }
}
