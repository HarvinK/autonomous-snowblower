#define USE_USBCON  
#include <Arduino.h>
#include <Servo.h>
#include <DueTimer.h>
#include <ros.h>
#include <std_msgs/Int64MultiArray.h> //is this com
#include <std_msgs/Float32MultiArray.h>

//Initialize motors
Servo FL; //Front Left Wheel
Servo BL; //Back Left Wheel
Servo FR; //Front Right Wheel 
Servo BR; //Back Right Wheel
Servo Auger; //Auger Motor

//Encoder pin setup
#define ENCODER_FL_PINA 26 
#define ENCODER_FL_PINB 25
#define ENCODER_BL_PINA 28 
#define ENCODER_BL_PINB 27
#define ENCODER_FR_PINA 34 
#define ENCODER_FR_PINB 33
#define ENCODER_BR_PINA 36 
#define ENCODER_BR_PINB 35

//Encoder and wheel specifications via measurements
#define TICKS_PER_M 1476.375
#define TICKS_PER_REV 856.8
#define RADIUS 0.09525 //nah change

//Timer interupt frequency
//#define dt 0.01 //10ms

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0; //may also have to be unsigned 
long changeMillis = 0;
float loopTime = 10; //10ms

//Speed Control PID Initialize 
//Setpoint is target m/s
//Output is PWM for motor
//prevError is difference between setpoint and current speed
volatile double Setpoint_FL = 0, Output_FL = 0, prevError_FL = 0, ErrorSum_FL = 0, Speed_FL = 0;
volatile double Setpoint_BL = 0, Output_BL = 0, prevError_BL = 0, ErrorSum_BL = 0, Speed_BL = 0;
volatile double Setpoint_FR = 0, Output_FR = 0, prevError_FR = 0, ErrorSum_FR = 0, Speed_FR = 0;
volatile double Setpoint_BR = 0, Output_BR = 0, prevError_BR = 0, ErrorSum_BR = 0, Speed_BR = 0;

//Create local variables for PID function, consider to be global
volatile double Error_FL = 0, changeInError_FL = 0, PID_FL = 0;
volatile double Error_BL = 0, changeInError_BL = 0, PID_BL = 0;
volatile double Error_FR = 0, changeInError_FR = 0, PID_FR = 0;
volatile double Error_BR = 0, changeInError_BR = 0, PID_BR = 0;

//PID gains
const double Kp_FL=10, Ki_FL=0.1, Kd_FL=0.2;
const double Kp_BL=0, Ki_BL=0, Kd_BL=0; 
const double Kp_FR=0, Ki_FR=0, Kd_FR=0; 
const double Kp_BR=0, Ki_BR=0, Kd_BR=0; 

//Initialize encoder tick values to 0 and track change
volatile long FLTicks = 0, BLTicks = 0, FRTicks = 0, BRTicks = 0;
volatile long prevFLTicks = 0, prevBLTicks = 0, prevFRTicks = 0, prevBRTicks = 0;
volatile double changeFLTicks = 0, changeBLTicks, changeFRTicks = 0, changeBRTicks = 0;

//Track change in distance traveled per wheel 
volatile double changeFLMeters = 0, changeBLMeters, changeFRMeters = 0, changeBRMeters = 0;

//Left motors quadrature encoder pulse 
void FLpulse() {
  static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static uint8_t enc_val1 = 0;

  enc_val1 = enc_val1 << 2;
  enc_val1 = enc_val1 | ((REG_PIOD_PDSR & 0b00000011));

  FLTicks = FLTicks + lookup_table[enc_val1 & 0b00001111];
}

void BLpulse() {
  static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static uint8_t enc_val2 = 0;

  enc_val2 = enc_val2 << 2;
  enc_val2 = enc_val2 | ((REG_PIOD_PDSR & 0b00001100) >> 2);

  BLTicks = BLTicks + lookup_table[enc_val2 & 0b00001111];
}

//Right motor quadrature encoder pulse
void FRpulse() {
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val3 = 0;

  enc_val3 = enc_val3 << 2;
  enc_val3 = enc_val3 | ((REG_PIOC_PDSR & 0b00000110) >> 1);
  
  FRTicks = FRTicks + lookup_table[enc_val3 & 0b00001111]; 
}

void BRpulse() {
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val4 = 0;

  enc_val4 = enc_val4 << 2;
  enc_val4 = enc_val4 | ((REG_PIOC_PDSR & 0b00011000) >> 3);
  
  BRTicks = BRTicks + lookup_table[enc_val4 & 0b00001111]; 
}

//Convert request cmd_vel setpoint to PWM
double FLspeedToPWM(double setpoint){  
  return 0.0535*sq(setpoint) + 17.8*(setpoint) + 92.9;
}

double BLspeedToPWM(double setpoint){  
  return 0.0535*sq(setpoint) + 17.8*(setpoint) + 92.9;
}

double FRspeedToPWM(double setpoint){  
  return 0.0535*sq(setpoint) + 17.8*(setpoint) + 92.9;
}

double BRspeedToPWM(double setpoint){  
  return 0.0535*sq(setpoint) + 17.8*(setpoint) + 92.9;
}

void SpeedPID(long dt){
  //Change in encoder ticks since last function call to find distance travelled          
  changeFLTicks = FLTicks - prevFLTicks; //change in ticks
  changeFLMeters = changeFLTicks/TICKS_PER_M; //distance travelled
  
  changeBLTicks = BLTicks - prevBLTicks;
  changeBLMeters = changeBLTicks/TICKS_PER_M;

  changeFRTicks = FRTicks - prevFRTicks;
  changeFRMeters = changeFRTicks/TICKS_PER_M;
  
  changeBRTicks = BRTicks - prevBRTicks; //Can have its own PID as long as RH setpoint is equal
  changeBRMeters = changeBRTicks/TICKS_PER_M;
  
  //Lower resolution means less accurate approximation 
  //changeFLMeters = (2 * PI * RADIUS * changeFLTicks) / TICKS_PER_REV;

  // Calculates speed for left and right wheel in m/s
  Speed_FL = changeFLMeters/dt; 
  Speed_BL = changeBLTicks/dt;
  Speed_FR = changeFRTicks/dt;
  Speed_BR = changeBRTicks/dt;

  // sets old count to new count for next calculation
  prevFLTicks = FLTicks;
  prevBLTicks = BLTicks;
  prevFRTicks = FRTicks;
  prevBRTicks = BRTicks;
  
  //calculating Error terms for PID controller
  Error_FL = Setpoint_FL - Speed_FL;
  Error_BL = Setpoint_BL - Speed_BR;
  Error_FR = Setpoint_FR - Speed_FR;
  Error_BR = Setpoint_BR - Speed_BR;

  changeInError_FL = Error_FL - prevError_FL; 
  changeInError_BL = Error_BL - prevError_BL;
  changeInError_FR = Error_FR - prevError_FR;
  changeInError_BR = Error_BR - prevError_BR;

  prevError_FL = Error_FL;
  prevError_BL = Error_BL;
  prevError_FR = Error_FR;
  prevError_BR = Error_BR;

  ErrorSum_FL += Error_FL;
  ErrorSum_BL += Error_BL;
  ErrorSum_FR += Error_FR;
  ErrorSum_BR += Error_BR;

  //Total PID calculation using set KP, KI, KD values
  PID_FL = (Kp_FL*Error_FL) + (Ki_FL*ErrorSum_FL*dt) + ((Kd_FL*changeInError_FL)/dt);   //dt is your time step or time spent for each cycle
  PID_BL = (Kp_BL*Error_BL) + (Ki_BL*ErrorSum_BL*dt) + ((Kd_BL*changeInError_BL)/dt); 
  PID_FR = (Kp_FR*Error_FR) + (Ki_FR*ErrorSum_FR*dt) + ((Kd_FR*changeInError_FR)/dt); 
  PID_BL = (Kp_BR*Error_BR) + (Ki_BR*ErrorSum_BR*dt) + ((Kd_BR*changeInError_BR)/dt); 

  //send setpoints to be converted to their respective PWM values     
  Output_FL = FLspeedToPWM(Setpoint_FL) + PID_FL;
  Output_BL = BLspeedToPWM(Setpoint_BL) + PID_BL;
  Output_FR = FRspeedToPWM(Setpoint_FR) + PID_FR;
  Output_BR = BRspeedToPWM(Setpoint_BR) + PID_BR;
   
  //constrain outputs to not exceed max/min PWM values   
  Output_FL = constrain(Output_FL, 0, 180);  
  Output_BL = constrain(Output_BL, 0, 180);  
  Output_FR = constrain(Output_FR, 0, 180);  
  Output_BR = constrain(Output_BR, 0, 180);   
   
  //Sends speed to motor control to be outputed to the motors
  FL.write(Output_FL); 
  BL.write(Output_BL);
  FR.write(Output_FR);
  BR.write(Output_BR);
} 

//Set up hardware interrupts for channel A and B of drive motors
void attachInterrupts() {           
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PINA), FLpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PINB), FLpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PINA), BLpulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BL_PINB), BLpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PINA), FRpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PINB), FRpulse, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PINA), BRpulse, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_BR_PINB), BRpulse, CHANGE); 

  //Better to compute within loop 
  //Timer1.attachInterrupt(SpeedPID);
  //Timer1.start(10000); // 0.01s (10ms) frequency timer interrupt. convert sec > microsecond dt*1000000
}

//ROS Parameters
ros::NodeHandle nh;

//ROS msg and pub topic declaration 
std_msgs::Int64MultiArray enc_ticks;
ros::Publisher enc_ticks_pub("encoder_ticks", &enc_ticks);

//diff drive controller callback
void velCallback(const std_msgs::Float32MultiArray& msg){
  float left_speed = msg.data[0]; //check units, needs m/s 
  float right_speed = msg.data[1];
  Setpoint_FL = left_speed;
  Setpoint_BL = left_speed;
  Setpoint_FR = right_speed;
  Setpoint_BR = right_speed;
}

ros::Subscriber<std_msgs::Float32MultiArray> cmd_sub("set_vel", &velCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  pinMode(7, OUTPUT);
  FL.attach(7);

  pinMode(8, OUTPUT);
  BL.attach(8);

  pinMode(6, OUTPUT);
  FR.attach(6);

  pinMode(5, OUTPUT);
  BR.attach(5);

  pinMode(ENCODER_FL_PINA, INPUT_PULLUP);
  pinMode(ENCODER_FL_PINB, INPUT_PULLUP);

  pinMode(ENCODER_BL_PINA, INPUT_PULLUP);
  pinMode(ENCODER_BL_PINB, INPUT_PULLUP);

  pinMode(ENCODER_FR_PINA, INPUT_PULLUP);
  pinMode(ENCODER_FR_PINB, INPUT_PULLUP);

  pinMode(ENCODER_BR_PINA, INPUT_PULLUP);
  pinMode(ENCODER_BR_PINB, INPUT_PULLUP);

  Serial.begin(115200); 
  attachInterrupts();

  //encoder ticks array initialization 
  char dim0_label[] = "encoder_ticks";
  enc_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2); 
  enc_ticks.layout.dim[0].label = dim0_label;
  enc_ticks.layout.dim[0].size = 4;
  enc_ticks.layout.dim[0].stride = 1*4;
  enc_ticks.data = (long long int *)malloc(sizeof(long long int)*4);
  enc_ticks.layout.dim_length = 0;
  enc_ticks.data_length = 4;

  nh.advertise(enc_ticks_pub);
  nh.subscribe(cmd_sub);
}

void loop() {
  nh.spinOnce(); //spin the ros node

  //Goal is to keep loop time consistent, delay as req or ommp method
  currentMillis = millis();
  changeMillis = currentMillis - previousMillis;

  if (changeMillis >= loopTime) {  // run a loop every 10ms          
    previousMillis = currentMillis; // reset the clock to time it

    SpeedPID(changeMillis); //ROS node spun so if setpoint set, should run motors
    enc_ticks.data[0]=FLTicks; //Feedback encoder data, for state estimatation
    enc_ticks.data[1]=BLTicks;
    enc_ticks.data[2]=FRTicks;
    enc_ticks.data[3]=BRTicks;
    enc_ticks_pub.publish(&enc_ticks); //pub int64 multi-array
  }
}  
  //delay(25); //may need delay if overflow 
  // Serial.print("FLTicks: "); Serial.println(FLTicks); 
  // Serial.print("BLTicks: "); Serial.println(BLTicks); 
  // Serial.print("FLTicks: "); Serial.println(FLTicks); 
  // Serial.print("BLTicks: "); Serial.println(BLTicks); 
  //Setpoint_FL = 1;
  //Serial.println(Error_FL); 
  //Serial.println(Output_FL); 
  //Serial.println(PID_FL);

  //Serial.println(Speed_FL);
  //Serial.println(changeFLTicks); 

  //Serial.print("OutputFL: "); Serial.println(Output_FL); 
  //Serial.print("PID_FL: "); Serial.println(PID_FL); 
  //Serial.print("Speed: "); Serial.println(Speed_FL);
  //  Serial.println("----------------------------------");
