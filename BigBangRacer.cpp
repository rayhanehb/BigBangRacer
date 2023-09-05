#include <Servo.h>
// GLOBALS

// PINS
int input_1 = 7;
int input_2 = 8;
          
int DIR_L_F = 13;    
int DIR_L_B = 12;    
int PWM_L = 11;      

int DIR_R_F = 4;     // Right Direction pin that will indicate forewards movement (1 for forewards, 0 for backwards)
int DIR_R_B = 2;     // Left Direction pin that will indicate backwards movement (1 for backwards, 0 for forewards).
int PWM_R = 3; 

void run_motor(int in1, int in2, int enA, int speed);

int Bucket_pin = 9;
// ultrasonic sensor pin
int trigPin = 6; 
int echoPin = 5;

int sonar_servo_pin = 10;
constexpr unsigned int MAX_MOTOR_SPEED = 255;
constexpr float MAX_ROTATION_RATE = 180.0;
static int angle;


class Controller {
public:
   
  // CONTROLLER PARAMS
  int P;
  int I = 0;
  int D = 0;
  int base_speed = 50;
  float Kp = 20;
  float Ki = 10;
  float Kd = 0;
  int integral_cap = 3;
  int right_motor_speed = base_speed;
  int left_motor_speed = base_speed;  
  bool square = false;
  int square_number = 0;
  long time = 0;
  long square_detection_time = 1000;

  Controller() {
    
  }

  int set_error() {
    // read IR sensor and get the current position
    // get error to be the difference between the current position and the desired position
    //int error = desired_position - curr_position;
    
    // reads 0 for white surface, 1 for black surface
    int out1 = digitalRead(input_1);
    int out2 = digitalRead(input_2);
    int prev_err = P;
    P = out2 - out1; // error positive if line is on right
    I += P;
    if (I > integral_cap) {
      I = integral_cap;
    } else if (I < -integral_cap) {
      I = -integral_cap;
    }
    D = P - prev_err;

    if (out1 == 1 && out2 == 1 && square == false){
      square = true;
      square_number++;
      time = millis();
    }
    if (millis() - time > square_detection_time){
      square = false;
    }

    // Serial.println(out1);
    // Serial.println(out2);
    // Serial.println(P);
    // Serial.println("\n");

  }
  
  void run_pid() {
    // calculate correction from PID terms 
    int correction = Kp*P + Ki*I + Kd*D;
    left_motor_speed = base_speed + correction;
    right_motor_speed = base_speed - correction;

    // limit left_motor_speed and right_motor_speed to be between max motor speed of 255 and min motor speed of 0
    // note upper and lower limits are adjustable 
    left_motor_speed = max(0, left_motor_speed);
    left_motor_speed = min(255, left_motor_speed);
    right_motor_speed = max(0, right_motor_speed);
    right_motor_speed = min(255, right_motor_speed);
  }

  void bang_bang(){
    if (P > 0){
      left_motor_speed = 0;
      right_motor_speed = 255;
    } else if (P < 0){
      left_motor_speed = 255;
      right_motor_speed = 0;
    } else {
      left_motor_speed = 255;
      right_motor_speed = 200;
    }
  }
    void fancy_bang_bang(){ // this function will get called when the error (integral term) gets too high - spin the opposite wheel backwards to enable faster turning
    if (P > 0){
      left_motor_speed = -255;
      right_motor_speed = 255;
    } else if (P < 0){
      left_motor_speed = 255;
      right_motor_speed = -255;
    } else { // keep travelling straight at full speed
      left_motor_speed = 255;
      right_motor_speed = 200;
    }
  }
}; // class Controller



  
class Ultrasonic{
  
public:
  double pingTime; // variable to hold
  double DIST_SCALE = 1.085767; // **MODIFY TO YOUR OWN**
  double SPEED_OF_SOUND = 331.0; // Approx. speed of sound
  double convert = 1000000.0; // factor to scale pingTime
  double dist;
  
  int start = 1; /// check
  int j = 90;
  static constexpr int step = 20;
  static constexpr double increment = 180/step;
  //int step_2 = step/2;
  static constexpr double threshold = 0.2;
  bool object_dected = false;
  double point_array[step];

  Ultrasonic() { // constructor
    
  }

  double get_distance(){
    digitalWrite(trigPin, LOW); // Set pin to low before pulse
    delayMicroseconds(2000);
    digitalWrite(trigPin, HIGH); // Send ping
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW); // Set pin to low again
    pingTime = pulseIn(echoPin, HIGH); // Receive the echo from the pulse 
    dist = DIST_SCALE*0.5*(SPEED_OF_SOUND*(pingTime/convert)); // Convert to distance
    return dist;
  }

  void scanner(Servo myservo){
    int i;
    Serial.println(start);
    if(start == 0){
      myservo.write(90);
      start +=1;
      delay(100);
    }
    else if(start == 1){
      myservo.write(180);
      j = 180;
      start +=1;
      delay(100);
    }
    else{
      for(i=step; i >= 0; i--)
      {
        j -= increment;
        myservo.write(j);
        delay(100);
        get_distance();
        point_array[i] = dist;  
      }
      start = 0;
    }
  }

  void perform_sweep(Servo myservo){
    int i;
    for (i=0;i<3;i++){
      scanner(myservo);
      delay(500);
    }
  }

  void polling(){
    dist = get_distance();
    if (dist< threshold){
      Serial.println("obstacle detected");
      object_dected=true;
    }
    else 
      object_dected = false;
  }

  void select_section() {
    constexpr int n = 3; //5 // (-90, -54), (-54, -18), (-18, 18), (18, 54), (54, 90)
    constexpr int angle[n] = {-72, 0, 72};//{-72, -36, 0, 36, 72};
    constexpr int num_per_section = this->step / n; // 20 / 5 = 4 point per section
    // step gives the number of points to check, which we sort into n sections

    // find the average of each section
    double section_average[n] = {0};
    for (int i=0; i < n; i++) { // in section i
      int j;
      for (j=0; j < num_per_section; j++) {
        section_average[i] += point_array[i*num_per_section + j];
      }
      point_array[i] /= num_per_section;
    }

    // find the section with the smallest average
    int min_index = 0;
    for (int i=1; i < n; i++) {
      if (section_average[i] < section_average[min_index]) {
        min_index = i;
      }
    }

    // return the angle of the section with the smallest average
    int select_angle = angle[min_index];
    if (select_angle == 0) {
      // this should not happen
      select_angle = 360;
      run_motor(DIR_L_F, DIR_L_B ,PWM_L, MAX_MOTOR_SPEED);
      run_motor(DIR_R_F, DIR_R_B ,PWM_R, -MAX_MOTOR_SPEED);
    } else if (select_angle > 0) {
      run_motor(DIR_L_F, DIR_L_B ,PWM_L, MAX_MOTOR_SPEED);
      run_motor(DIR_R_F, DIR_R_B ,PWM_R, -MAX_MOTOR_SPEED);
    } else {
      run_motor(DIR_L_F, DIR_L_B ,PWM_L, -MAX_MOTOR_SPEED);
      run_motor(DIR_R_F, DIR_R_B ,PWM_R, MAX_MOTOR_SPEED);
    }
    delay(5 * abs(select_angle));
    run_motor(DIR_L_F, DIR_L_B ,PWM_L, 0);
    run_motor(DIR_R_F, DIR_R_B ,PWM_R, 0);
    delay(1000);
  }
}; // class Ultrasonic

void run_motor(int in1, int in2, int enA, int speed) {
  // 
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    //analogWrite(enA, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  // set speed of motor
  analogWrite(enA, abs(speed));
}

//code from slides

// Global instance
Controller controller;
Ultrasonic sonar;
Servo ser;
   


void setup() {
  Serial.begin(9600); 

  pinMode(input_1, INPUT);
  pinMode(input_2, INPUT);
  pinMode(DIR_L_F, OUTPUT);
  pinMode(DIR_L_B, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_R_F, OUTPUT);
  pinMode(DIR_R_B, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  controller = Controller(/*constructor args*/);
  sonar = Ultrasonic();
  ser.attach(sonar_servo_pin);
  ser.write(90);
}

void loop() {   
  controller.set_error();
  Serial.println(abs(controller.I));
  if (abs(controller.I) >= controller.integral_cap){ // if one sensor detects black 3 times in a row
    controller.fancy_bang_bang();
    //controller.I = 0 // reset integral to 0//
  } else{
  controller.bang_bang();
  }

  run_motor(DIR_L_F, DIR_L_B ,PWM_L, controller.left_motor_speed);
  run_motor(DIR_R_F, DIR_R_B ,PWM_R, controller.right_motor_speed);
  delay(50);
  run_motor(DIR_L_F, DIR_L_B ,PWM_L, 0);
  run_motor(DIR_R_F, DIR_R_B ,PWM_R, 0);
  delay(50);

  sonar.polling();
  if (sonar.object_dected==true){
    sonar.perform_sweep(ser);
    sonar.select_section();
  }
}