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

double diameter = 0.304; // diameter of the wheel in meter
int res = 2500; // resolution of the encoder in (pulse/sec)

void setup() {
    pinMode(encoder0PinA, INPUT_PULLUP);
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
          
        
         
       Serial.print(x,DEC);
       Serial.print("\t");
       Serial.print(y,DEC);
       Serial.print("\t");
       Serial.println(theta,DEC);
      // Serial.println(millis());

    }
}
