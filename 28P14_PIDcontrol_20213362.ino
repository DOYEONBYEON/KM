#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12 
#define PIN_ECHO 13

// Distance sensor
#define _DIST_ALPHA 0.01  //DIST_ALPHA 값 설정

#define _DIST_TARGET 255    //정지하려는 위치 목표값
#define _DUTY_MIN 300 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2600 // servo full counterclockwise position (180 degree)

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define _INTERVAL_DIST 25  // USS interval (unit: ms)
#define _INTERVAL_SERVO 5  // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)
#define INTERVAL 1 // sampling interval (unit: ms)

// Servo speed control
#define _SERVO_ANGLE 30 // 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 13 // 서보 속도 설정
#define _RAMPUP_TIME 360

#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

#define _KP 2        //비례제어 값
#define _KD 103        //미분제어 값
#define _KI 1

#define _ITERM_MAX 30

Servo myservo;
int a, b; // unit: mm
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha, dist_min, dist_max; //거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //마지막으로 측정한 거리, 마지막으로 측정한 서보 각도(각 이벤트별로 업데이트를 위해 측정하는 시간)
bool event_dist, event_servo, event_serial; //이벤트 별로 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; //interval 당 최대 duty 변화량
int duty_target, duty_curr; //목표 duty와 현재 duty 값
unsigned long last_sampling_time; // unit: ms

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
//error_curr: 현재 주기 오차값 / error_prev : 이전 주기 오차 값 / control : PID 제어량 / pterm, dterm, iterm : 비례,적분,미분 이득값


void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1500);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;

  
  dist_raw = 0.0; // raw distance output from USS (unit: mm)

  

  duty_curr = 1499;
  
  duty_curr=_DUTY_MIN; // duty_curr 값 초기화
  dist_raw = dist_ema = ir_distance_filtered();
  error_curr = error_prev = dist_target - dist_ema;
  dist_target=_DIST_TARGET; //dist_target 값 초기화
  a = 71.5; //70;
  b = 300; //300;

  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  
  Serial.begin(57600);
  last_sampling_time = 0;

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)*(_SERVO_SPEED/float(_SERVO_ANGLE))*(_INTERVAL_DIST/1000.0); //duty_chg_per_interval 값 설정 
}

float ir_distance(void){ // 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}





void loop() {

  if(millis() < last_sampling_time + INTERVAL) return;
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
          last_sampling_time_dist += _INTERVAL_DIST;
          event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
          last_sampling_time_servo += _INTERVAL_SERVO;
          event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
          last_sampling_time_serial += _INTERVAL_SERIAL;
          event_serial = true;
  }

  dist_raw = ir_distance_filtered();
  dist_ema = alpha*dist_raw + (1-alpha)*dist_ema;
  double dist_raw_prev;
  if (dist_min <= dist_raw && dist_raw <= dist_max) {
    dist_raw_prev = dist_raw;
  } else {
    dist_raw = dist_raw_prev;
  }
  float dist_cali = 100 + 300.0 / (b - a) * (dist_ema - a);
  
  
  


  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor

  // PID control logic
    error_curr = (dist_target - dist_cali);
    pterm = _KP * error_curr ;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;

    if(iterm>_ITERM_MAX)iterm=_ITERM_MAX;
    if(iterm<-_ITERM_MAX)iterm=-_ITERM_MAX;

    control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}
    if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}

    error_prev = error_curr;
  }

  if(event_servo) {
    event_servo=false;  

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr); 
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_cali);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }

}
