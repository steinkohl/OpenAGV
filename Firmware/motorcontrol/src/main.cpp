#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <mcp_can.h>
//#include <EEPROM.h>
#include <EEPROMex.h>

#define VERSION "0.0.4"
#define SELECTED_WHEEL 3 // FR=0, FL=1, BR=2, BL=3

// set up can
// speeds for all
#define CAN_ID_SETSPEED 100
#define CAN_SETSPEED_MAX_CYCLETIME 200  //ms
// config - front right
#define CAN_ID_CONFIG 113

// useful constants
// gearreduction=27, beltreduction=3, magneticchanges=34*2
#define STEPS_PER_REV 5508      // 27*3*34*2
#define CIRCUMFERENCE 640       // ~640mm wheel circumference
#define STEPS_PER_MM  0.116194626 // (CIRCUMFERENCE / STEPS_PER_REV)

// define pins
#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_VCC 4
#define R_IS 5
#define L_IS 6
#define R_EN 7
#define L_EN 8
#define RPWM 9
#define LPWM 10
#define CS_PIN A5

// EEPROM
#define ADDR_KP 0   //0-7
#define ADDR_KI 8   //8-15
#define ADDR_KD 16  //16-23
#define ADDR_REV_DIR 24
#define ADDR_PID_EN 25


struct config_msg {
  int16_t Kp;
  int16_t Ki;
  int16_t Kd;
  bool pid_enable;
  bool reverse_direction;
};

typedef union {
  struct speeds {
    // int16_t        -->   -32768..32767
    // -20000..20000  -->   -2000..2000mm/s or -100..100%
    // other          -->   disable motor and driver
    int16_t fr;
    int16_t fl;
    int16_t br;
    int16_t bl;
  } speed_vals;
  int16_t wheel[4];
} speed_msg;

typedef union {
  config_msg config;
  speed_msg speed;
  uint8_t bytes[8];
} can_msg;


double doubleFromInt16(int16_t val){
  return ((double) val) / 1000.0;
}

int16_t int16FromDouble(double val){
  return (int16_t) (val * 1000);
}

MCP_CAN CAN(CS_PIN);

//Define Variables we'll be connecting to
double SetpointSpeed, InputSpeed, OutputPWM;

//Specify the links and initial tuning parameters
double Kp, Ki, Kd;
PID myPID(&InputSpeed, &OutputPWM, &SetpointSpeed, 0, 0, 0, DIRECT);

volatile long counter = 0;
double pos=0, last_pos, dur_s;
unsigned long time=1, last_time, timeLastMSG=0;
int16_t rawSpeed;
can_msg msg;
bool reverse_direction = false;
bool pid_enable = false;

void isrA(){
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if((a && b)||(!a && !b)) counter--;
  else counter++;
}

void isrB(){
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if((a && !b)||(!a && b)) counter--;
  else counter++;
}

int sign(int value, int maxvalue){
  if(value > maxvalue){
    return maxvalue;
  }
  if(value < -maxvalue){
    return -maxvalue;
  }
  return value;
}

int zeroMin(int value, int minValue){
  if(value < minValue && value > -minValue) return 0;
  return value;
}

// enables motor on driver
void enableMotor(){
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

// disables motor on driver
void disableMotor(){
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

void blockMotor(){
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}


void printStats(){
  Serial.print("Position: ");
  Serial.print(pos);
  Serial.print("mm, Speed: ");
  Serial.print(InputSpeed);
  Serial.print("mm/s, Setpoint: ");
  Serial.print(SetpointSpeed);
  if (pid_enable) Serial.println("mm/s");
  else Serial.println("%");
}

// motorPWM -255..255
// --> motor has to be enabled
void writeMotorPWM(int motorPWM){
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  if (motorPWM > 0){
    analogWrite(RPWM, sign(motorPWM, 255));
    digitalWrite(LPWM, LOW);
  } else {
    digitalWrite(RPWM, LOW);
    analogWrite(LPWM, sign(-motorPWM, 255));
  }
}

void setup() {
  // set pwm frequency to 31.3kHz (pin 9 & 10)
  TCCR1B = TCCR1B & 0b11111000 | 0x01; 

  // set pin modes, as well as static pins
  // setup encoder
  pinMode(ENCODER_VCC, OUTPUT);
  digitalWrite(ENCODER_VCC, HIGH);
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), isrA, CHANGE);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), isrB, CHANGE);
  // setup motor driver
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_IS, INPUT);
  pinMode(L_IS, INPUT);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
  // setup led

  //turn the PID on
  SetpointSpeed = 0.0;
  Kp = EEPROM.readDouble(ADDR_KP);
  Ki = EEPROM.readDouble(ADDR_KI);
  Kd = EEPROM.readDouble(ADDR_KD);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-255,255);

  // read control bytes from eeprom
  pid_enable = EEPROM.readByte(ADDR_PID_EN);
  reverse_direction = EEPROM.readByte(ADDR_REV_DIR);

  // set up serial
  Serial.begin(115200);
  Serial.print("Version ");
  Serial.print(VERSION);
  Serial.print(" from ");
  Serial.println(__DATE__);

  // init can
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
      Serial.println("CAN BUS - Init Failed");
      delay(100);
  }
  Serial.println("CAN BUS - Init OK!");
}

void loop() {
  last_time = time;
  time = millis();
  if(CAN_MSGAVAIL == CAN.checkReceive())
  {
    unsigned char len = 0;
    CAN.readMsgBuf(&len, msg.bytes);
    unsigned long canId = CAN.getCanId();
    switch (canId)
    {
      case CAN_ID_SETSPEED:
        timeLastMSG = time;
        if (reverse_direction) rawSpeed = -msg.speed.wheel[SELECTED_WHEEL]; 
        else rawSpeed = msg.speed.wheel[SELECTED_WHEEL]; 
        break;

      case CAN_ID_CONFIG:
        Kp = doubleFromInt16(msg.config.Kp);
        Ki = doubleFromInt16(msg.config.Ki);
        Kd = doubleFromInt16(msg.config.Kd);
        pid_enable = msg.config.pid_enable;
        reverse_direction = msg.config.reverse_direction;
        EEPROM.updateDouble(ADDR_KP, Kp);
        EEPROM.updateDouble(ADDR_KI, Ki);
        EEPROM.updateDouble(ADDR_KD, Kd);
        EEPROM.updateByte(ADDR_PID_EN, pid_enable);
        EEPROM.updateByte(ADDR_REV_DIR, reverse_direction);
        myPID.SetTunings(Kp, Ki, Kd);
        myPID.SetMode(pid_enable ? AUTOMATIC : MANUAL);

        break;
      
      default:
        Serial.print("\nReceived message from ID: ");
        Serial.println(canId, HEX);
        break;
    }
  }

  // block motor if der is no continuous SETSPEED stream
  if ((time - timeLastMSG) > CAN_SETSPEED_MAX_CYCLETIME) rawSpeed = -32768;

  last_pos = pos;
  pos = ((double) counter)*STEPS_PER_MM;
  dur_s = ((double) time-last_time)/1000;
  InputSpeed = (pos-last_pos)/dur_s;
  myPID.Compute();

  // int16_t        -->   -32768..32767
  // -32768         -->   block motor
  if (rawSpeed == -32768) blockMotor();
  // -20000..20000  -->   -2000..2000mm/s or -100..100%
  else if (rawSpeed >= -20000 && rawSpeed <= 20000) {
    if (pid_enable) {
      SetpointSpeed = rawSpeed/10.0;
      writeMotorPWM((int) OutputPWM);
    } 
    else writeMotorPWM((int) rawSpeed / 78.431); // scales to -255..255
  // other          -->   disable motor and driver
  } else disableMotor();

  printStats();
}
