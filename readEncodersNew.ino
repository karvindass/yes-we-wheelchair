// defining the pins. encoder0 is on the left, 1 is on the right
#define encoder0PinA 2 // Output A Encoder L
#define encoder0PinB 5 // Output A Encoder L
#define encoder1PinA 3 // Output B Encoder R
#define encoder1PinB 4 // Output B Encoder R

volatile long encoder0Count = 0; // changing this var to -1 or +1, rotation in Encoder L is indicated
volatile long encoder1Count = 0; // changing this var to -1 or +1, rotation in Encoder R is indicated

unsigned long interval = 100; // once this interval is reached, odometry motion measurement is taken
unsigned long previousTime = 0; // var used to check interval - compared to currentTime var

double x = 0.0; // initial x position when arduino is started
double y = 0.0; // initial y position when arduino is started
double theta = 0.0; // initial theta (orientation) when arduino is started

double diameter = 0.304; // diameter of the wheel in meters
int res = 2500; // resolution of the encoder in (pulse/sec)

void setup() {
    pinMode(encoder0PinA, INPUT_PULLUP); // configuring defined pins as inputs
    pinMode(encoder0PinB, INPUT_PULLUP);
    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);

    attachInterrupt(0, doEncoder0, RISING);
    attachInterrupt(1, doEncoder1, RISING);
    Serial.begin (9600);
    Serial.println("START READING");
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
        
        double dsl = encoder0Count * diameter / res; // distance change in left wheel
        encoder0Count = 0; // resetting to allow for next computation in next loop
        double dsr = encoder1Count * diameter / res; // distance change in right wheel
        encoder1Count = 0; // resetting to allow for next computation in next loop
        double ds = (dsr + dsl) / 2.0;
        double dtheta = (dsr-dsl) / 0.9;
        double dx = ds * cos(theta + dtheta/2);
        double dy = ds * sin(theta + dtheta/2);

        x += dx;
        y += dy;
        theta += dtheta;
        
        if (theta >= PI) { // if loop works to make sure theta values stay in range from -pi to +pi
          theta -= 2 * PI;
        } else if (theta <= -PI) {
          theta += 2 * PI;
        }
          
        
        // printing to serial port
       Serial.print(x,DEC);
       Serial.print("\t");
       Serial.print(y,DEC);
       Serial.print("\t");
       Serial.println(theta,DEC);
      // Serial.println(millis());

    }
}
