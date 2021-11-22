#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12 
#define PIN_ECHO 13

// Distance sensor
#define _DIST_ALPHA 0.5 //DIST_ALPHA 값 설정

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define _INTERVAL_DIST 25  // USS interval (unit: ms)
#define _INTERVAL_SERVO 20  // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.3
#define INTERVAL 25 // sampling interval (unit: ms)

#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)


Servo myservo;
int a, b; // unit: mm
int duty_interval;
int duty_;
float dist_min, dist_max, dist_raw, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float timeout; // unit: us


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1500);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  

  duty_ = 1500;
  
  a = 75; //70;
  b = 332; //300;
  Serial.begin(57600);
  last_sampling_time = 0;
}

float ir_distance(void){ // 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}



void loop() {

  if(millis() < last_sampling_time + INTERVAL) return;
  
  dist_raw = ir_distance();
  dist_ema = alpha*dist_raw + (1-alpha)*dist_ema;
  double dist_raw_prev;
  if (dist_min <= dist_raw && dist_raw <= dist_max) {
    dist_raw_prev = dist_raw;
  } else {
    dist_raw = dist_raw_prev;
  }

  float dist_cali = 100 + 300.0 / (b - a) * (dist_ema - a);
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(dist_raw);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > 220 && dist_cali <317) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
  

  if(dist_cali > 295) {
    duty_ = 1330;
  }
  else {
    duty_ = 1670;
  }


  if(duty_ > 2100) {
    duty_ = 2100;
  }
  else if(duty_ < 900) {
    duty_  = 900;
  }

  
  
  myservo.writeMicroseconds(duty_);
  last_sampling_time += INTERVAL;
}
